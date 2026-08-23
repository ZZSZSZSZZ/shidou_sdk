#pragma once

// Message codec contract.
//
// Every message type in shidou::msg provides:
//     bool Encode(WriteBuffer& out) const;   // serializes the CDR body
//     bool Decode(ReadCursor& in);           // parses the CDR body
// Implementations live in msg_codecs.h; field order there is the wire
// contract and must match the robot-side ROS message definitions exactly.
//
// The codec layer deals only with CDR bodies. Framing (topic encapsulation
// header, service request header) belongs to the comm layer
// (comm/wire_codec.h) and is never applied here.
//
// Encode fails only on internal misuse. Decode fails on any malformed or
// truncated input, leaves the message in an unspecified but valid state,
// and must never throw.

#include <cstdint>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::codec {

// Convenience: encode one message into a fresh vector (the CDR body).
template <typename T>
bool EncodeToVector(const T& msg, std::vector<uint8_t>& out) {
    WriteBuffer buf;
    if (!msg.Encode(buf)) {
        return false;
    }
    out = buf.Data();
    return true;
}

// Convenience: decode one message from a CDR body. Strict: trailing bytes
// are rejected, so a mismatched message type fails loudly instead of
// silently decoding a prefix.
template <typename T>
bool DecodeFromVector(const uint8_t* data, size_t size, T& msg) {
    ReadCursor in(data, size);
    return msg.Decode(in) && in.Remaining() == 0;
}

} // namespace shidou::codec
