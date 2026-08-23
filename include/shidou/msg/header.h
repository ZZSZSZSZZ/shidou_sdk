#pragma once

// std_msgs/Header equivalent: timestamp + frame_id. The stamp is a fixed
// 8-byte pair (int32 sec + uint32 nanosec) with no padding.

#include <cstdint>
#include <string>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct Header {
    int32_t stamp_sec = 0;
    uint32_t stamp_nanosec = 0;
    std::string frame_id;

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const Header& other) const {
        return stamp_sec == other.stamp_sec && stamp_nanosec == other.stamp_nanosec &&
               frame_id == other.frame_id;
    }
};

} // namespace shidou::msg
