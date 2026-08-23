#pragma once

// FSM vocabulary shared with the robot side: the command strings
// accepted on fsm_command and the state names published on fsm_state.
// The robot receives commands case-insensitively but publishes states
// uppercase.

namespace shidou::robot {

// Control modes selectable after the ENABLED handshake.
enum class ControlMode {
    kCsp,        // CSP drives: JointCSPTarget streams
    kPosition,   // CSP drives: JointPositionTarget streams
    kTrajectory, // run the trajectory uploaded via the joint_trajectory service
};

// fsm_command payload for a mode.
inline const char* ToCommand(ControlMode mode) {
    switch (mode) {
        case ControlMode::kCsp: return "csp";
        case ControlMode::kPosition: return "position";
        case ControlMode::kTrajectory: return "trajectory";
    }
    return "";
}

// fsm_state value while the mode is active.
inline const char* ToFsmState(ControlMode mode) {
    switch (mode) {
        case ControlMode::kCsp: return "CSP";
        case ControlMode::kPosition: return "POSITION";
        case ControlMode::kTrajectory: return "TRAJECTORY";
    }
    return "";
}

// FSM state names; STOP only accepts the "enabled" command.
inline constexpr const char* kFsmStateEnabled = "ENABLED";
inline constexpr const char* kFsmStateStop = "STOP";

} // namespace shidou::robot
