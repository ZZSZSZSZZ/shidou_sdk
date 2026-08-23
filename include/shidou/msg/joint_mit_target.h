#pragma once

// MIT mode joint target (impedance control). Arrays are index-aligned with
// motor_ids; optional arrays may be empty (robot applies config defaults).

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct JointMITTarget {
    std::vector<uint32_t> motor_ids;
    std::vector<double> positions;   // rad
    std::vector<double> velocities;  // rad/s, optional
    std::vector<double> torques;     // Nm feedforward, optional
    std::vector<double> kps;         // stiffness, optional
    std::vector<double> kds;         // damping, optional

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const JointMITTarget& other) const {
        return motor_ids == other.motor_ids && positions == other.positions &&
               velocities == other.velocities && torques == other.torques && kps == other.kps &&
               kds == other.kds;
    }
};

} // namespace shidou::msg
