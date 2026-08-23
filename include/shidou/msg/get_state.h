#pragma once

// get_state service: request is empty; response carries FSM state, arm and
// gripper info, joint state arrays and control-loop statistics.

#include <cstdint>
#include <string>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct GetStateRequest {
    // No fields: encodes to an empty CDR body.

    static constexpr size_t kCdrAlignment = 1;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const GetStateRequest&) const { return true; }
};

struct GetStateResponse {
    std::string fsm_state;   // STOP / ENABLED / CSP / POSITION / TRAJECTORY
    // "{model}_{layout}_{dof}", e.g. arm_dual_7; "none" when no arm.
    std::string arm_info;
    // right / left / dual / none
    std::string gripper_info;

    std::vector<uint32_t> motor_ids;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> torques;

    uint32_t library_status = 0;     // 0x0000 = normal; nonzero = see robot code table
    double control_freq_hz = 0.0;    // EWMA of actual control frequency
    double control_cycle_avg_ms = 0.0;
    double control_cycle_max_ms = 0.0;
    uint32_t cycle_overruns = 0;     // cycles exceeding dt * 1.5

    static constexpr size_t kCdrAlignment = 8;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const GetStateResponse& other) const {
        return fsm_state == other.fsm_state && arm_info == other.arm_info &&
               gripper_info == other.gripper_info && motor_ids == other.motor_ids &&
               positions == other.positions && velocities == other.velocities &&
               torques == other.torques && library_status == other.library_status &&
               control_freq_hz == other.control_freq_hz &&
               control_cycle_avg_ms == other.control_cycle_avg_ms &&
               control_cycle_max_ms == other.control_cycle_max_ms &&
               cycle_overruns == other.cycle_overruns;
    }
};

} // namespace shidou::msg
