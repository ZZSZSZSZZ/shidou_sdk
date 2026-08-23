#pragma once

// joint_trajectory service: request carries the trajectory to upload,
// response is a success flag.

#include "shidou/codec/buffer.h"
#include "shidou/msg/joint_trajectory.h"

namespace shidou::msg {

struct JointTrajectoryRequest {
    JointTrajectory trajectory;

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const JointTrajectoryRequest& other) const {
        return trajectory == other.trajectory;
    }
};

struct JointTrajectoryResponse {
    bool success = false;

    static constexpr size_t kCdrAlignment = 1;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const JointTrajectoryResponse& other) const {
        return success == other.success;
    }
};

} // namespace shidou::msg
