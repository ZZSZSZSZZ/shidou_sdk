#pragma once

// Cyclic-synchronous position (CSP) target. There is no kp/kd/accel in
// CSP; only position/velocity/torque may be supplied. Arrays are
// index-aligned with motor_ids; optional arrays may be empty.

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct JointCSPTarget {
    std::vector<uint32_t> motor_ids;
    std::vector<double> positions;   // rad
    std::vector<double> velocities;  // rad/s, optional
    std::vector<double> torques;     // Nm, optional

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const JointCSPTarget& other) const {
        return motor_ids == other.motor_ids && positions == other.positions &&
               velocities == other.velocities && torques == other.torques;
    }
};

} // namespace shidou::msg
