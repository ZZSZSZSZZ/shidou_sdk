#pragma once

// Profile Position mode target. Arrays are index-aligned with motor_ids;
// optional arrays may be empty (robot applies config defaults).

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct JointPositionTarget {
    // Target sequence: +1 per frame; 0 = unset (the robot then accepts the
    // frame unconditionally). A non-zero value must advance every frame, or
    // the robot drops the frame as stale.
    uint32_t sequence = 0;

    std::vector<uint32_t> motor_ids;
    std::vector<double> positions;       // rad
    std::vector<double> velocities;      // rad/s, optional
    std::vector<double> torques;         // Nm, optional
    std::vector<double> accelerations;   // rad/s^2, optional

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const JointPositionTarget& other) const {
        return sequence == other.sequence && motor_ids == other.motor_ids &&
               positions == other.positions && velocities == other.velocities &&
               torques == other.torques && accelerations == other.accelerations;
    }
};

} // namespace shidou::msg
