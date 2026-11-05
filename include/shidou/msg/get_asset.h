#pragma once

// robot_description/get_asset service: the robot-side endpoint that returns
// one description asset by the URI the description text carries.
//
// Request: `uri` exactly as it appeared in the description snapshot. The SDK
// does not parse it, join paths, validate it or cache anything; the endpoint
// owns the interpretation of the URI and all safety checks.
//
// Response: `ok` false carries the robot-side reason in `error` and leaves
// `data` empty; `ok` true carries the asset bytes. Failures travel inside the
// reply because the service client treats zenoh error replies as silence.
// The layout mirrors the robot-side GetAsset.srv: one string in, then bool +
// string + uint8 sequence out.

#include <cstdint>
#include <string>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::msg {

struct GetAssetRequest {
    std::string uri; // opaque to the SDK: passed through byte for byte

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const GetAssetRequest& other) const { return uri == other.uri; }
};

struct GetAssetResponse {
    bool ok = false;
    std::string error;          // robot-side reason, empty on success
    std::vector<uint8_t> data;  // asset bytes, empty on failure

    static constexpr size_t kCdrAlignment = 4;

    bool Encode(codec::WriteBuffer& out) const;
    bool Decode(codec::ReadCursor& in);

    bool operator==(const GetAssetResponse& other) const {
        return ok == other.ok && error == other.error && data == other.data;
    }
};

} // namespace shidou::msg
