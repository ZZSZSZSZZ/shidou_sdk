#pragma once

// CDR (XCDR v1, little-endian) compound primitives: sequences and strings.
//
// Wire layout, with all alignment measured from the start of the CDR body
// (a reader and a writer for one message must both start at offset 0):
//   sequence<T>: 4-byte-aligned uint32 length, then the elements, each
//                aligned to its natural CDR alignment; empty = length 0
//   string     : 4-byte-aligned uint32 byte length counting a trailing
//                NUL terminator, then the bytes with a final NUL --
//                the rmw_cyclonedds serdes convention this wire carries
//
// Natural alignments: bool = 1, uint16/int16 = 2, uint32/int32/float32 = 4,
// uint64/float64 = 8.
// Only little-endian hosts are supported; big-endian hosts are out of scope.

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "shidou/codec/buffer.h"

namespace shidou::codec {

// Natural CDR alignment of an element type.
template <typename T>
struct CdrAlignOf
    : std::integral_constant<size_t,
                             sizeof(T) >= 8 ? 8 : (sizeof(T) >= 4 ? 4 : (sizeof(T) >= 2 ? 2 : 1))> {};

// Sequence of fixed-size elements. Decoding rejects lengths whose element
// bytes could not fit into the remaining payload (malformed-length defense);
// no allocation happens before that check.
template <typename T>
bool ReadSeq(ReadCursor& in, std::vector<T>& out) {
    uint32_t len = 0;
    if (!in.Align(4) || !in.ReadU32(len)) {
        return false;
    }
    if (len > in.Remaining() / sizeof(T)) {
        return false;
    }
    out.assign(len, T{});
    if (len == 0) {
        return true;
    }
    if (!in.Align(CdrAlignOf<T>::value)) {
        return false;
    }
    for (T& v : out) {
        if (!in.ReadBytes(&v, sizeof(T))) {
            return false;
        }
    }
    return true;
}

// Sequence of fixed-size elements. Elements are padded only when the
// sequence is non-empty, matching the ROS 2 CDR wire behavior.
template <typename T>
bool WriteSeq(WriteBuffer& out, const std::vector<T>& v) {
    if (!out.Align(4)) {
        return false;
    }
    out.WriteU32(static_cast<uint32_t>(v.size()));
    if (v.empty()) {
        return true;
    }
    if (!out.Align(CdrAlignOf<T>::value)) {
        return false;
    }
    out.WriteBytes(v.data(), v.size() * sizeof(T));
    return true;
}

// Sequence of message structs: uint32 length, then each element aligned to
// its struct alignment (T::kCdrAlignment) and encoded in place with the
// same cursor, so inner alignments stay relative to the CDR body start.
template <typename T>
bool ReadSeqStruct(ReadCursor& in, std::vector<T>& out) {
    uint32_t len = 0;
    if (!in.Align(4) || !in.ReadU32(len)) {
        return false;
    }
    // Every element occupies at least one byte, so this is a safe bound.
    if (len > in.Remaining()) {
        return false;
    }
    out.clear();
    out.reserve(len);
    for (uint32_t i = 0; i < len; ++i) {
        if (!in.Align(T::kCdrAlignment)) {
            return false;
        }
        T e;
        if (!e.Decode(in)) {
            return false;
        }
        out.push_back(std::move(e));
    }
    return true;
}

template <typename T>
bool WriteSeqStruct(WriteBuffer& out, const std::vector<T>& v) {
    if (!out.Align(4)) {
        return false;
    }
    out.WriteU32(static_cast<uint32_t>(v.size()));
    for (const T& e : v) {
        if (!out.Align(T::kCdrAlignment)) {
            return false;
        }
        if (!e.Encode(out)) {
            return false;
        }
    }
    return true;
}

// String: uint32 byte length counting the trailing NUL, then the bytes.
// The length check rejects payloads that cannot hold the claimed bytes.
inline bool ReadString(ReadCursor& in, std::string& out) {
    uint32_t len = 0;
    if (!in.Align(4) || !in.ReadU32(len)) {
        return false;
    }
    if (len > in.Remaining()) {
        return false;
    }
    // rmw_cyclonedds counts the NUL terminator in the length and its
    // deserializer drops it (serdes.hpp validate_str); drop one trailing
    // NUL when present. A length of 0 decodes as the empty string.
    uint32_t content_len = len;
    if (len > 0 && in.Position()[len - 1] == '\0') {
        --content_len;
    }
    out.assign(reinterpret_cast<const char*>(in.Position()), content_len);
    return in.Skip(len);
}

inline bool WriteString(WriteBuffer& out, const std::string& s) {
    if (s.size() >= UINT32_MAX) {
        return false;
    }
    if (!out.Align(4)) {
        return false;
    }
    // The NUL terminator is counted in the length: the rmw_cyclonedds
    // deserializer validates the last byte and reads length - 1 content
    // bytes. An empty string is length 1 with a single NUL.
    out.WriteU32(static_cast<uint32_t>(s.size()) + 1);
    out.WriteBytes(s.data(), s.size());
    out.WriteU8('\0');
    return true;
}

// Aligned primitive helpers; keep message codecs free of padding noise. A
// 16-bit scalar member has to be aligned to 2 the same way; there is no
// helper for it because no message carries one.
inline bool ReadU32Aligned(ReadCursor& in, uint32_t& v) { return in.Align(4) && in.ReadU32(v); }
inline bool ReadF64Aligned(ReadCursor& in, double& v) { return in.Align(8) && in.ReadF64(v); }
inline bool ReadBool(ReadCursor& in, bool& v) {
    uint8_t b = 0;
    if (!in.ReadU8(b)) {
        return false;
    }
    v = (b != 0);
    return true;
}

inline bool WriteU32Aligned(WriteBuffer& out, uint32_t v) {
    if (!out.Align(4)) {
        return false;
    }
    out.WriteU32(v);
    return true;
}

inline bool WriteF64Aligned(WriteBuffer& out, double v) {
    if (!out.Align(8)) {
        return false;
    }
    out.WriteF64(v);
    return true;
}

// CDR bools are one byte; write a normalized 0/1 value.
inline bool WriteBool(WriteBuffer& out, bool v) {
    out.WriteU8(v ? 1 : 0);
    return true;
}

} // namespace shidou::codec
