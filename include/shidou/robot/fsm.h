#pragma once

// FSM vocabulary shared with the robot side: the control modes, the tokens
// the run_command service accepts in its `mode` field, and the state names
// published on fsm_state. The robot receives commands from one service and
// publishes state names uppercase.
//
// One control mode owns one FSM state, and the state name is the mode name:
// MIT / CSP / PP. A joint target is accepted only while the robot is in its
// mode's state, and only if the drives really are in that mode. The mode is
// set exclusively through the run_command service; target streams never
// change it. ENABLED is the damping hold (no joint targets) and STOP the
// disabled state, left only by an enable command.

namespace shidou::robot {

enum class ControlMode {
    kMit, // JointMITTarget streams; accepted in MIT
    kCsp, // JointCSPTarget streams; accepted in CSP
    kPp,  // JointPositionTarget streams; accepted in PP
};

// Mode token for the run_command service's `mode` field.
inline const char* ToModeToken(ControlMode mode) {
    switch (mode) {
        case ControlMode::kMit: return "mit";
        case ControlMode::kCsp: return "csp";
        case ControlMode::kPp: return "pp";
    }
    return "";
}

// fsm_state value while the mode is active.
inline const char* ToFsmState(ControlMode mode) {
    switch (mode) {
        case ControlMode::kMit: return "MIT";
        case ControlMode::kCsp: return "CSP";
        case ControlMode::kPp: return "PP";
    }
    return "";
}

// FSM states without a mode of their own.
inline constexpr const char* kFsmStateEnabled = "ENABLED";
inline constexpr const char* kFsmStateStop = "STOP";

} // namespace shidou::robot
