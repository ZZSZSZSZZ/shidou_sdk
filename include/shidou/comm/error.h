#pragma once

// SDK error codes. The public API never throws; every operation reports
// failure through a bool return plus the last error code and message.

namespace shidou {

enum class ErrorCode : int {
    kOk = 0,
    kInvalidArgument,    // bad name, keyexpr, namespace or option value
    kNotInitialized,     // no open session, or the robot is not ready
    kAlreadyInitialized, // repeated Open on an already-open session
    kSessionError,       // zenoh session creation or connect failure
    kEncodeError,        // message failed to encode
    kDecodeError,        // payload failed to decode (malformed or truncated)
    kTimeout,            // request/response did not complete in time
    kNoReply,            // query reached no queryable
    kNotConnected,       // session is not alive; operation rejected
    kInternal,
};

// Stable, human-readable name for logging and diagnostics.
inline const char* ToString(ErrorCode code) {
    switch (code) {
        case ErrorCode::kOk: return "ok";
        case ErrorCode::kInvalidArgument: return "invalid_argument";
        case ErrorCode::kNotInitialized: return "not_initialized";
        case ErrorCode::kAlreadyInitialized: return "already_initialized";
        case ErrorCode::kSessionError: return "session_error";
        case ErrorCode::kEncodeError: return "encode_error";
        case ErrorCode::kDecodeError: return "decode_error";
        case ErrorCode::kTimeout: return "timeout";
        case ErrorCode::kNoReply: return "no_reply";
        case ErrorCode::kNotConnected: return "not_connected";
        case ErrorCode::kInternal: return "internal";
    }
    return "unknown";
}

} // namespace shidou
