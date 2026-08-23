#pragma once

// Gripper target: open is mapped by robot config (0 = closed, 1 = open).
// Arrays are index-aligned with motor_ids.

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct GripperTarget {
    std::vector<uint32_t> motor_ids;
    std::vector<double> open;  // [0.0, 1.0], mapped by config pos_min/max
    std::vector<double> kps;   // stiffness
    std::vector<double> kds;   // damping

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const GripperTarget& other) const {
        return motor_ids == other.motor_ids && open == other.open && kps == other.kps &&
               kds == other.kds;
    }
};

} // namespace shidou::msg
