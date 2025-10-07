// ArduPlane/mode_tecs_tune.cpp
#include "mode_tecs_tune.h"

void ModeTecsTune::run()
{
    // örnek tetik: RC7 yüksekse bir STE adımı başlat
    if (!_tecs_at.running() && (plane.rc().read(7) > 1800)) {
        _cfg.step_dh_m = 50.0f;         // sahada paramlaştır
        _cfg.settle_band = 0.02f;
        _cfg.target_settle_s = 8.0f;
        _cfg.target_overshoot_pct = 10.0f;

        _tecs_at.begin(AP_TECS_AT_Step::STE_ALT_STEP);
    }

    _tecs_at.update(); // ~50–100 ms’de bir çağrılır
}
