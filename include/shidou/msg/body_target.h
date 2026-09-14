#pragma once

// Body target (composite): hub wheel velocities and a pushrod position in
// one message, independent of the joint target streams.
//
// Arrays are index-aligned: velocities[i] belongs to wheel_ids[i];
// max_currents may be shorter or empty (the robot applies the device
// default). pushrod_id = 0 means "no pushrod target in this frame".
//
// Robot-side gating (robot_movement body component): the wheels move in
// POSITION only and are forced to zero in every other state, and they fall
// back to zero velocity when no message arrives for 500 ms (dead man, timed
// on message arrival); the pushrod works in ENABLED + POSITION and holds its
// position once there. Body feedback is NOT a separate topic: it arrives in
// joint_states with mixed units distinguished by motor_id (pushrod mm,
// wheels rad and rad/s).

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct BodyTarget {
    std::vector<uint32_t> wheel_ids;   // hub motor IDs (IWS)
    std::vector<double> velocities;    // rad/s, index-aligned with wheel_ids
    std::vector<double> max_currents;  // A, optional; empty or short -> device default

    uint32_t pushrod_id = 0;    // 0 = no pushrod target in this frame
    double position = 0.0;      // mm
    double velocity = 0.0;      // mm/s, profile velocity
    double acceleration = 0.0;  // mm/s^2

    // The sequences align to 4; the direct float64 members make the struct
    // itself 8-aligned (as for GetStateResponse).
    static constexpr size_t kCdrAlignment = 8;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const BodyTarget& other) const {
        return wheel_ids == other.wheel_ids && velocities == other.velocities &&
               max_currents == other.max_currents && pushrod_id == other.pushrod_id &&
               position == other.position && velocity == other.velocity &&
               acceleration == other.acceleration;
    }
};

} // namespace shidou::msg
