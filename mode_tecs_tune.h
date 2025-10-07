// ArduPlane/mode_tecs_tune.h
#pragma once
#include "Mode.h"
#include <AP_TECS/AP_TECS_Autotune.h>

class PlaneTECSIO : public AP_TECS_AT_IO {
public:
    PlaneTECSIO(Plane& plane) : _plane(plane) {}
    float get_alt() const override       { return _plane.relative_altitude(); }
    float get_alt_ref() const override   { return _plane.auto_state.target_altitude; }
    float get_arsp() const override      { return _plane.ahrs.groundspeed(); /* veya ARSPD */ }
    float get_arsp_ref() const override  { return _plane.auto_state.target_airspeed; }
    float get_thr_out() const override   { return _plane.throttle_output; }
    float get_pitch_cmd() const override { return degrees(_plane.attitude_control->get_pitch_target_rad()); }

    void set_alt_ref(float h_cmd) override  { _plane.set_target_altitude(h_cmd); }
    void set_arsp_ref(float v_cmd) override { _plane.set_target_airspeed(v_cmd); }

private:
    Plane& _plane;
};

class ModeTecsTune : public Mode
{
public:
    ModeTecsTune(Plane& plane) : Mode(plane), _io(plane), _tecs_at(_io, _cfg) {}
    bool init(bool) override { return true; }
    void run() override;

private:
    PlaneTECSIO _io;
    AP_TECS_AT_Config _cfg;
    AP_TECS_Autotune _tecs_at;
};
