// Minimal UnitreeJoystick stub for Booster K1 builds.
// unitree_sdk2 is not required on Booster systems; this header satisfies
// the includes in unitree_joystick_dsl.hpp and observations.h without
// pulling in any Unitree SDK.
//
// All fields default to zero/false — the K1 joystick is driven by keyboard.
// AxisKey derives from KeyBase and adds operator() so that axis reads like
// joystick->rx() (used in observations.h velocity_commands) return 0.0f.

#pragma once

namespace unitree { namespace common {

struct KeyBase {
    bool  pressed{false};
    bool  on_pressed{false};
    bool  on_released{false};
    float pressed_time{0.f};
};

// Axis / trigger keys: castable to KeyBase& (for the DSL), and callable
// as a function returning the current axis value (for observations.h).
struct AxisKey : public KeyBase {
    float operator()() const { return 0.f; }
};

struct UnitreeJoystick {
    // Buttons
    KeyBase back, start;
    KeyBase LS, RS;
    KeyBase LB, RB;
    KeyBase A, B, X, Y;
    KeyBase up, down, left, right;
    KeyBase F1, F2;
    // Axes / triggers (callable, derived from KeyBase for DSL compatibility)
    AxisKey lx, ly, rx, ry;
    AxisKey LT, RT;
};

}} // namespace unitree::common
