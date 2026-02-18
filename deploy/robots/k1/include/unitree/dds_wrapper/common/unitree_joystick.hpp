// Minimal UnitreeJoystick stub for Booster K1 builds.
// unitree_sdk2 is not required on Booster systems; this header satisfies
// the include in unitree_joystick_dsl.hpp without pulling in any Unitree SDK.
// All fields default to zero/false — the K1 joystick is driven by keyboard.

#pragma once

namespace unitree { namespace common {

struct KeyBase {
    bool  pressed{false};
    bool  on_pressed{false};
    bool  on_released{false};
    float pressed_time{0.f};
};

struct UnitreeJoystick {
    KeyBase back, start;
    KeyBase LS, RS;
    KeyBase LB, RB;
    KeyBase A, B, X, Y;
    KeyBase up, down, left, right;
    KeyBase F1, F2;
    KeyBase lx, ly, rx, ry;
    KeyBase LT, RT;
};

}} // namespace unitree::common
