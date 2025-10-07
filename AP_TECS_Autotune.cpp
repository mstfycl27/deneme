// libraries/AP_TECS/AP_TECS_Autotune.cpp
#include "AP_TECS_Autotune.h"
#include <cmath>

extern const AP_HAL::HAL& hal;

AP_TECS_Autotune::AP_TECS_Autotune(AP_TECS_AT_IO& io, const AP_TECS_AT_Config& cfg)
: _io(io), _cfg(cfg) {}

void AP_TECS_Autotune::begin(AP_TECS_AT_Step type)
{
    if (running()) return;
    _type = type;
    reset();
    read_params_once();
    _state = State::ApplyStep;
}

void AP_TECS_Autotune::reset()
{
    _mx = Metrics{};
    _t_rel = 0.0f;
    _n_samp = _n_thr_sat = _n_pch_sat = 0;

    _h0 = _io.get_alt();
    _h_ref0 = _io.get_alt_ref();
    _v0 = _io.get_arsp();
    _v_ref0 = _io.get_arsp_ref();

    _peak_y = -1e9f; _trough_y = 1e9f;
    _y_last = 0.0f;

    _t0_ms = hal.scheduler->millis();
    _last_ms = _t0_ms;
}

bool AP_TECS_Autotune::in_band(float y, float y_final, float band) const
{
    return (y >= (y_final - band)) && (y <= (y_final + band));
}

void AP_TECS_Autotune::apply_step()
{
    // Step tasarımı
    if (_type == AP_TECS_AT_Step::STE_ALT_STEP) {
        _h_ref_final = _h_ref0 + _cfg.step_dh_m;
        _io.set_alt_ref(_h_ref_final);
        _v_ref_final = _v_ref0; // hız sabit
        _io.set_arsp_ref(_v_ref_final);
    } else {
        // SBE: eşlenik (h,V) — toplam enerji ~ sabit olsun
        _v_ref_final = _v_ref0 + _cfg.step_dv_ms;
        float dh = (_v0 * _cfg.step_dv_ms) / 9.81f; // ≈ V0*ΔV/g
        _h_ref_final = _h_ref0 - dh;
        _io.set_arsp_ref(_v_ref_final);
        _io.set_alt_ref(_h_ref_final);
    }
    _state = State::Measure;
    hal.console->printf("TECS-AT: step applied (h_ref=%.1f, v_ref=%.1f)\n",
                        (double)_h_ref_final, (double)_v_ref_final);
}

void AP_TECS_Autotune::measure(float dt)
{
    _t_rel += dt;

    // ölçüm: seçilen kanalın çıktısını izleyelim
    float y_meas = (_type == AP_TECS_AT_Step::STE_ALT_STEP) ? _io.get_alt() : _io.get_arsp();
    float y_ref  = (_type == AP_TECS_AT_Step::STE_ALT_STEP) ? _h_ref_final : _v_ref_final;

    // IAE/ITAE
    float e = (y_ref - y_meas);
    _e_abs_int += fabsf(e) * dt;
    _t_e_abs_int += (_t_rel) * fabsf(e) * dt;

    // overshoot ölçümü için pik/çukur
    if (y_meas > _peak_y)   _peak_y = y_meas;
    if (y_meas < _trough_y) _trough_y = y_meas;
    _y_last = y_meas;

    // doygunluk sayacı (yaklaşık)
    _n_samp++;
    if (_io.get_thr_out() >= 0.98f || _io.get_thr_out() <= 0.02f) _n_thr_sat++;
    // Pitch için ±25 deg varsaydık; IO tarafında kendi limite göre normalize etmek daha iyi
    if (fabsf(_io.get_pitch_cmd()) > 24.5f) _n_pch_sat++;

    // timeout
    if (_t_rel > (_cfg.window_pre_s + _cfg.window_post_s) || _t_rel > _cfg.timeout_s) {
        _state = State::Update;
    }

    // erken yerleşme kontrolü (band içinde kalma)
    const float step_size = ( (_type==AP_TECS_AT_Step::STE_ALT_STEP) ? (_h_ref_final - _h0) : (_v_ref_final - _v0) );
    const float band = fmaxf(fabsf(step_size) * _cfg.settle_band, (_type==AP_TECS_AT_Step::STE_ALT_STEP) ? 1.0f : 0.3f);
    if (in_band(y_meas, y_ref, band) && (_t_rel > _cfg.window_pre_s)) {
        // bir daha çıkmadığını “garanti” etmek için basit bir ek süre şartı koy
        // (gerçekte kaydırmalı pencereyle kontrol etmek daha sağlam)
        static const float guard = 1.0f;
        if ((_t_rel + guard) >= _cfg.window_pre_s && (_t_rel + guard) <= _cfg.timeout_s) {
            _mx.settle_time = _t_rel;
            // yine de ölçüm penceresini sonuna kadar kullanıp Update'e geçelim
        }
    }
}

void AP_TECS_Autotune::compute_and_decide()
{
    // adım büyüklüğü ve overshoot
    const float step_size = ( (_type==AP_TECS_AT_Step::STE_ALT_STEP) ? (_h_ref_final - _h0) : (_v_ref_final - _v0) );
    _mx.step_size = step_size;

    float y_final = (_type==AP_TECS_AT_Step::STE_ALT_STEP) ? _h_ref_final : _v_ref_final;

    if (step_size > 0.0f) {
        _mx.overshoot_pct = fmaxf(0.0f, (_peak_y - y_final) / fabsf(step_size) * 100.0f);
    } else if (step_size < 0.0f) {
        _mx.overshoot_pct = fmaxf(0.0f, (y_final - _trough_y) / fabsf(step_size) * 100.0f);
    } else {
        _mx.overshoot_pct = 0.0f;
    }

    _mx.iae  = _e_abs_int;
    _mx.itae = _t_e_abs_int;
    _mx.sse  = (y_final - _y_last);   // pencere sonunda kalan hata (kabaca)

    // doygunluk oranları
    if (_n_samp > 0) {
        _mx.sat_thr_pct = 100.0f * (float)_n_thr_sat / (float)_n_samp;
        _mx.sat_pch_pct = 100.0f * (float)_n_pch_sat / (float)_n_samp;
    }

    hal.console->printf("TECS-AT: step=%.2f, ts=%.1fs, ov=%.1f%%, IAE=%.2f, ITAE=%.2f, SSE=%.2f, sat(T,P)=%.0f%%,%.0f%%\n",
                        (double)_mx.step_size, (double)_mx.settle_time, (double)_mx.overshoot_pct,
                        (double)_mx.iae, (double)_mx.itae, (double)_mx.sse,
                        (double)_mx.sat_thr_pct, (double)_mx.sat_pch_pct);
}

void AP_TECS_Autotune::tweak_params()
{
    // Param pointer’ları bulunamadıysa dokunma
    if (!_p_time_const || !_p_ptch_damp || !_p_thr_damp || !_p_integ_gain) {
        hal.console->printf("TECS-AT: param pointers missing; skip update\n");
        return;
    }

    const float settle = _mx.settle_time;
    const float overs  = _mx.overshoot_pct;

    bool changed = false;

    // Basit kural: yerleşme yavaşsa hızlandır (TIME_CONST ↓)
    if (!std::isnan(settle) && settle > _cfg.target_settle_s) {
      _p_time_const->set(MAX(1.0f, _p_time_const->get() - _cfg.d_time_const));
      hal.console->printf("TECS_TIME_CONST -= %.2f -> %.2f\n", (double)_cfg.d_time_const, (double)_p_time_const->get());
      changed = true;
    }

    // Overshoot yüksekse sönümü artır
    if (overs > _cfg.target_overshoot_pct) {
      _p_ptch_damp->set(_p_ptch_damp->get() + _cfg.d_damp);
      _p_thr_damp->set(_p_thr_damp->get() + _cfg.d_damp);
      hal.console->printf("TECS_PTCH_DAMP/THR_DAMP += %.2f -> (%.2f,%.2f)\n",
                          (double)_cfg.d_damp, (double)_p_ptch_damp->get(), (double)_p_thr_damp->get());
      changed = true;
    }

    // Kalıcı ofset (SSE büyük) için biraz integratör
    if (fabsf(_mx.sse) > 0.5f) {
      _p_integ_gain->set(_p_integ_gain->get() + _cfg.d_integ_gain);
      hal.console->printf("TECS_INTEG_GAIN += %.3f -> %.3f\n",
                          (double)_cfg.d_integ_gain, (double)_p_integ_gain->get());
      changed = true;
    }

    if (!changed) {
        hal.console->printf("TECS-AT: no param change (within targets)\n");
    }
}

void AP_TECS_Autotune::read_params_once()
{
    // Param’ları isimden bul (AP_Param API’si hedef sürümüne göre değişir)
    // Aşağıdaki yol ArduPilot’ta yaygındır: AP_Param::find ve AP_Float*’a cast.
    AP_Param::ParamToken t;

    _p_time_const = (AP_Float*)AP_Param::find("TECS_TIME_CONST", &t);
    _p_ptch_damp  = (AP_Float*)AP_Param::find("TECS_PTCH_DAMP",  &t);
    _p_thr_damp   = (AP_Float*)AP_Param::find("TECS_THR_DAMP",   &t);
    _p_integ_gain = (AP_Float*)AP_Param::find("TECS_INTEG_GAIN", &t);
}

void AP_TECS_Autotune::update()
{
    if (_state == State::Idle) return;

    const uint32_t now = hal.scheduler->millis();
    const float dt = (now - _last_ms) * 0.001f;
    _last_ms = now;

    switch (_state) {
    case State::ApplyStep:
        apply_step();
        break;
    case State::Measure:
        measure(dt);
        // süremiz dolduysa Update’e
        if ((_t_rel > _cfg.window_pre_s + _cfg.window_post_s) || (_t_rel > _cfg.timeout_s)) {
            _state = State::Update;
        }
        break;
    case State::Update:
        compute_and_decide();
        tweak_params();
        _state = State::Idle; // tek atış; istersen burada yeni test serisine dönebilirsin
        break;
    default:
        _state = State::Idle;
        break;
    }
}
