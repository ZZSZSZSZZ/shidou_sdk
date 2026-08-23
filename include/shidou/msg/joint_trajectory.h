#pragma once

// Joint trajectory for both MIT and CSP drives. Uploaded
// via the joint_trajectory service before switching the FSM to TRAJECTORY
// mode. mit_points and csp_points are index-aligned with joint_id.

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"
#include "shidou/msg/header.h"

namespace shidou::msg {

struct MITWaypoint {
    std::vector<double> positions;   // rad, per joint order
    std::vector<double> velocities;  // rad/s, empty = zero
    std::vector<double> torques;     // Nm feedforward, empty = zero
    bool stop_point = false;         // stops here (last point stops automatically)

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const MITWaypoint& other) const {
        return positions == other.positions && velocities == other.velocities &&
               torques == other.torques && stop_point == other.stop_point;
    }
};

struct CSPWaypoint {
    std::vector<double> positions;         // rad
    std::vector<double> max_velocity;      // rad/s limit for Ruckig
    std::vector<double> max_acceleration;  // rad/s^2 limit for Ruckig
    std::vector<double> torques;           // Nm

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const CSPWaypoint& other) const {
        return positions == other.positions && max_velocity == other.max_velocity &&
               max_acceleration == other.max_acceleration && torques == other.torques;
    }
};

struct JointTrajectory {
    Header header;
    std::vector<uint32_t> joint_id;
    std::vector<MITWaypoint> mit_points;
    std::vector<CSPWaypoint> csp_points;

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const JointTrajectory& other) const {
        return header == other.header && joint_id == other.joint_id &&
               mit_points == other.mit_points && csp_points == other.csp_points;
    }
};

} // namespace shidou::msg
