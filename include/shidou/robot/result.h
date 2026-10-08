#pragma once

// Outcome of one Robot operation: the code and the message belong to the call
// that returned this value, so a caller never has to combine a bool with a
// sticky error record to explain a failure.
//
// Invariant: a successful value has code kOk and an empty message; a failed
// value has a non-empty message. The two halves agree, so either one may be
// used to tell success from failure.

#include <string>

#include "shidou/comm/error.h"

namespace shidou::robot {

// Converts to true only on success. The conversion is explicit: conditional
// call sites (`if (!robot.Enable())`, `!robot.SendBodyTarget(t)`) keep
// compiling unchanged, while a value cannot silently degrade into a bool that
// drops the code and the message.
struct Result {
    ErrorCode code = ErrorCode::kOk;
    std::string message;  // non-empty when the code is not kOk

    explicit operator bool() const { return code == ErrorCode::kOk; }
};

} // namespace shidou::robot
