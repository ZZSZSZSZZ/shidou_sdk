#pragma once

// Feedback snapshot published by the robot at 100 Hz.
// All arrays are index-aligned: motor_ids[i] describes the i-th motor and
// every other array carries the parallel i-th value.

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct JointFeedback {
    std::vector<uint32_t> motor_ids;
    std::vector<double> position;    // rad
    std::vector<double> velocity;    // rad/s
    std::vector<double> effort;      // Nm

    // CDR bools are one byte each; uint8_t avoids std::vector<bool>'s
    // bit-packing, which is not wire-compatible. Values are 0 or 1.
    std::vector<uint8_t> enabled;    // FSM enabled per motor
    std::vector<uint8_t> online;     // not disabled and state is fresh
    std::vector<uint32_t> fault_code;  // alarm code passthrough, 0 = no fault
    std::vector<float> temperature;    // motor temperature

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const JointFeedback& other) const {
        return motor_ids == other.motor_ids && position == other.position &&
               velocity == other.velocity && effort == other.effort && enabled == other.enabled &&
               online == other.online && fault_code == other.fault_code &&
               temperature == other.temperature;
    }
};

} // namespace shidou::msg
