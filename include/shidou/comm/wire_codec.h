#pragma once

// Wire framing for topic payloads and service requests/replies.
//
// This file is the ONLY place in the SDK that may hold wire-format
// assumptions about the zenoh-plugin-ros2dds bridge. The codec layer
// deals with plain CDR bodies and never sees these envelopes. The
// assumptions below are spike-verified at M3 against real bridge traffic
// and against the plugin source (1.7.2); if the plugin differs, change
// the constants and layouts here only.
//
// Contract:
//   topic payload : 4-byte CDR_LE encapsulation header + CDR body, with
//                   up to 3 zero padding bytes appended so the payload
//                   reaches a 4-byte boundary (rmw_cyclonedds serializes
//                   this way and the bridge forwards the padding)
//   service query : the SAME frame as a topic payload (encapsulation
//                   header + request CDR body). The 16-byte rmw request
//                   id (client guid + sequence number) never crosses the
//                   zenoh wire: the bridge synthesizes it on the ROS side
//                   and strips it from replies, correlating each reply to
//                   its query internally.
//   service reply : 4-byte CDR_LE encapsulation header + response CDR
//                   body (no request id), same padding rule as topics

#include <cstdint>
#include <cstring>
#include <vector>

#include "shidou/codec/codec.h"

namespace shidou::comm {

constexpr size_t kTopicFrameHeaderSize = 4;

// CDR_LE encapsulation: u16 representation identifier 0x0001 + u16
// options 0x0000. The identifier is transmitted in big-endian byte order
// (DDSI-RTPS convention) while the body is little-endian; spike-verified
// against real bridge payloads.
constexpr uint8_t kCdrLeEncapsulation[4] = {0x00, 0x01, 0x00, 0x00};

// Encodes header + CDR body of a topic publication.
template <typename T>
bool EncodeTopicFrame(const T& msg, std::vector<uint8_t>& out) {
    std::vector<uint8_t> body;
    if (!codec::EncodeToVector(msg, body)) {
        return false;
    }
    out.clear();
    out.reserve(kTopicFrameHeaderSize + body.size());
    out.insert(out.end(), kCdrLeEncapsulation, kCdrLeEncapsulation + kTopicFrameHeaderSize);
    out.insert(out.end(), body.begin(), body.end());
    return true;
}

// Decodes a topic payload. Any other encapsulation (e.g. PL_CDR_LE) is
// rejected: the SDK only speaks plain CDR with this bridge. The bridge
// forwards the body exactly as rmw_cyclonedds serialized it, including
// up to 3 zero bytes of padding appended to reach a 4-byte boundary;
// those are accepted and ignored.
template <typename T>
bool DecodeTopicFrame(const uint8_t* data, size_t size, T& msg) {
    if (size < kTopicFrameHeaderSize ||
        std::memcmp(data, kCdrLeEncapsulation, kTopicFrameHeaderSize) != 0) {
        return false;
    }
    codec::ReadCursor in(data + kTopicFrameHeaderSize, size - kTopicFrameHeaderSize);
    if (!msg.Decode(in)) {
        return false;
    }
    const size_t trailing = in.Remaining();
    if (trailing > 3) {
        return false;
    }
    for (size_t i = 0; i < trailing; ++i) {
        if (in.Position()[i] != 0) {
            return false;
        }
    }
    return true;
}

// Encodes a service request: the bridge expects the exact same frame as
// a topic payload (encapsulation header + CDR body).
template <typename TReq>
bool EncodeServiceRequest(const TReq& req, std::vector<uint8_t>& out) {
    return EncodeTopicFrame(req, out);
}

// Decodes a service reply: encapsulation header + response CDR body. The
// zenoh query/reply channel already correlates each reply with the query
// that produced it, so no request id check is needed here.
template <typename TRes>
bool DecodeServiceReply(const uint8_t* data, size_t size, TRes& res) {
    return DecodeTopicFrame(data, size, res);
}

} // namespace shidou::comm
