#pragma once

// std_msgs/msg/String equivalent. Used by fsm_state, whose payload is the
// uppercase FSM state name (ENABLED / MIT / CSP / PP / STOP). Commands no
// longer travel as strings: they go through the run_command service.

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
