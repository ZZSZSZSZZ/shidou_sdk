#pragma once

// Snapshot of the robot as seen by GetRobotState: the get_state service
// response plus the joint_states telemetry cache at the moment of the
// call.

#include <cstdint>
#include <string>
#include <vector>

namespace shidou::robot {

struct RobotState {
    // get_state service response. arm_info carries the layout and dof only
    // ("{layout}_{dof}"); the model name is its own field.
    std::string fsm_state;
    std::string arm_info;
    std::string gripper_info;
    std::vector<uint32_t> motor_ids;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> torques;
    uint32_t library_status = 0;
    double control_freq_hz = 0.0;
    double control_cycle_avg_ms = 0.0;
    double control_cycle_max_ms = 0.0;
    uint32_t cycle_overruns = 0;
    // Model name (arm_s1 / robot_d1); empty on a hand-written config, and
    // against a robot that predates the field.
    std::string model;
    // Body classes the robot drives (none / wheels / pushrod /
    // wheels_pushrod); empty against a robot that predates the field.
    std::string body_info;

    // joint_states telemetry at call time; false fields when no sample
    // has arrived yet.
    bool has_feedback = false;
    uint64_t feedback_seq = 0;
    double feedback_age_ms = -1.0;
};

} // namespace shidou::robot
