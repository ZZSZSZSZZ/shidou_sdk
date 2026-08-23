#pragma once

// std_msgs/msg/String equivalent. Used by fsm_state (published) and
// fsm_command (subscribed): payload is lowercase "enabled" / "csp" /
// "position" / "trajectory" / "stop", and fsm_state carries the uppercase
// FSM state name.

#include <string>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct String {
    std::string data;

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const String& other) const { return data == other.data; }
};

} // namespace shidou::msg
