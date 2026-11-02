#pragma once

// run_command service: the robot's only source of control modes.
//
// Request:
//   command = "enable"   enable and hold in ENABLED; `mode` optionally names
//                        the hold law (mit / csp / pp); empty = the family's
//                        auto-detected law. An unknown or unsupported law
//                        refuses the whole command.
//             "disable"  disable immediately; always accepted
//             "set_mode" enter the mode named in `mode` (mit / csp / pp),
//                        queued until the current motion settles
//
// Response: `accepted` plus a code (ok / rejected_moving / unsupported_mode /
// busy) and a human-readable message. The layout mirrors the robot-side
// RunCommand.srv: two strings in, then bool + string + string out.

#include <string>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct RunCommandRequest {
    std::string command;
    std::string mode; // set_mode's target, or enable's hold law; empty = unset

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const RunCommandRequest& other) const {
        return command == other.command && mode == other.mode;
    }
};

struct RunCommandResponse {
    bool accepted = false;
    std::string code;    // ok / rejected_moving / unsupported_mode / busy
    std::string message; // human-readable reason, empty on a bare success

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const RunCommandResponse& other) const {
        return accepted == other.accepted && code == other.code && message == other.message;
    }
};

} // namespace shidou::msg
