// libraries/AP_TECS/AP_TECS_Autotune.h
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

// Çok hafif bir IO arayüzü: Mode katmanı bu callback'leri doldurur.
// Böylece setpoint'leri nasıl verdiğimiz (TECS/do_your_way) buraya sızmaz.
class AP_TECS_AT_IO {
public:
    virtual ~AP_TECS_AT_IO() {}
    // ölçümler
    virtual float get_alt() const = 0;         // m (relative/amsl: tutarlı olsun)
    virtual float get_alt_ref() const = 0;     // m (mevcut hedef)
    virtual float get_arsp() const = 0;        // m/s
    virtual float get_arsp_ref() const = 0;    // m/s
    virtual float get_thr_out() const = 0;     // 0..1 (veya 0..100 normalize edebilirsin)
    virtual float get_pitch_cmd() const = 0;   // deg (komut ya da attitude)

    // setpoint yazıcılar
    virtual void set_alt_ref(float h_cmd) = 0;
    virtual void set_arsp_ref(float v_cmd) = 0;
};

// Basit konfig
struct AP_TECS_AT_Config {
    float step_dh_m      = 50.0f;  // STE testi için irtifa step
    float step_dv_ms     = 0.0f;   // SBE için eşlenik hız step
    float settle_band    = 0.02f;  // ±% band (step büyüklüğüne göre)
    float window_pre_s   = 2.0f;   // ölçüme başlamadan önce bekleme
    float window_post_s  = 20.0f;  // step sonrası pencere
    float timeout_s      = 25.0f;  // güvenlik zaman aşımı
    // Parametre ayar adımları
    float d_time_const   = 0.2f;
    float d_damp         = 0.05f;
    float d_integ_gain   = 0.005f;

    // Hedef performans (basit kurallar)
    float target_settle_s = 8.0f;
    float target_overshoot_pct = 10.0f;
};

// Basit step tipi: STE (irtifa) ya da SBE (eşlenik h/V)
enum class AP_TECS_AT_Step {
    STE_ALT_STEP,
    SBE_COUPLED
};

class AP_TECS_Autotune {
public:
    AP_TECS_Autotune(AP_TECS_AT_IO& io, const AP_TECS_AT_Config& cfg);

    void begin(AP_TECS_AT_Step type);
    void update();                 // scheduler’dan ~50–100 ms çağır
    bool running() const { return _state != State::Idle; }

private:
    enum class State { Idle, ApplyStep, Measure, Update };

    // iç metrikler
    struct Metrics {
        float step_size = 0.0f;
        float settle_time = NAN;
        float overshoot_pct = 0.0f;
        float iae = 0.0f;   // ∫|e|dt
        float itae = 0.0f;  // ∫t|e|dt
        float sse = 0.0f;   // steady-state error (pencere sonu)
        float sat_thr_pct = 0.0f;
        float sat_pch_pct = 0.0f;
    };

    void apply_step();
    void measure(float dt);
    void compute_and_decide();
    void tweak_params();
    void reset();

    // küçük yardımcılar
    bool in_band(float y, float y_final, float band) const;
    void read_params_once(); // TECS param pointers’ı bul

    // zaman/sayaç
    State        _state = State::Idle;
    uint32_t     _t0_ms = 0, _last_ms = 0;
    float        _t_rel = 0.0f;

    // step ve referanslar
    AP_TECS_AT_Step _type = AP_TECS_AT_Step::STE_ALT_STEP;
    float        _h0 = 0.0f, _h_ref0 = 0.0f, _h_ref_final = 0.0f;
    float        _v0 = 0.0f, _v_ref0 = 0.0f, _v_ref_final = 0.0f;

    // ölçüm akümülatörleri
    float        _e_abs_int = 0.0f;
    float        _t_e_abs_int = 0.0f;
    float        _peak_y = -1e9f, _trough_y = 1e9f;
    float        _y_last = 0.0f;

    // doygunluk sayacı
    uint32_t     _n_samp = 0, _n_thr_sat = 0, _n_pch_sat = 0;

    // config & io
    AP_TECS_AT_IO&     _io;
    AP_TECS_AT_Config  _cfg;

    // Param pointer’ları (bulunamazsa NAN kalır)
    AP_Float* _p_time_const = nullptr; // TECS_TIME_CONST
    AP_Float* _p_ptch_damp  = nullptr; // TECS_PTCH_DAMP
    AP_Float* _p_thr_damp   = nullptr; // TECS_THR_DAMP
    AP_Float* _p_integ_gain = nullptr; // TECS_INTEG_GAIN

    Metrics _mx;
};
