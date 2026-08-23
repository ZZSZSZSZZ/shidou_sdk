#pragma once

// Bounds-checked cursor over a caller-provided byte span.
//
// Contract: every read checks the remaining length first and returns false
// on overrun; the cursor never reads past the end and never throws. This is
// the front line against malformed network payloads.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace shidou::codec {

class ReadCursor {
public:
    ReadCursor(const uint8_t* data, size_t size) : data_(data), size_(size), offset_(0) {}

    // Copies n bytes into out; false if fewer than n bytes remain.
    bool ReadBytes(void* out, size_t n);

    // Advances without copying; used to skip unknown trailing fields.
    bool Skip(size_t n);

    // Pads to the next multiple of alignment (measured from the start of the
    // buffer); fails on overrun. Alignment must be a power of two.
    bool Align(size_t alignment);

    size_t Offset() const { return offset_; }
    size_t Remaining() const { return size_ - offset_; }

    // Pointer to the next unread byte; valid only while the backing span lives.
    const uint8_t* Position() const { return data_ + offset_; }

    bool ReadU8(uint8_t& v);
    bool ReadU32(uint32_t& v);
    bool ReadI32(int32_t& v);
    bool ReadU64(uint64_t& v);
    bool ReadF32(float& v);
    bool ReadF64(double& v);

private:
    const uint8_t* data_;
    size_t size_;
    size_t offset_;
};

// Growable byte buffer with CDR alignment semantics, used to serialize
// messages. Writes never fail (the buffer grows on demand); Align can
// only fail on an invalid (non-power-of-two) alignment.
class WriteBuffer {
public:
    // Zero-fills to the next multiple of alignment (measured from buffer
    // start). Returns false if alignment is not a power of two.
    bool Align(size_t alignment);

    void WriteBytes(const void* data, size_t n);
    void WriteU8(uint8_t v);
    void WriteU32(uint32_t v);
    void WriteI32(int32_t v);
    void WriteU64(uint64_t v);
    void WriteF32(float v);
    void WriteF64(double v);

    size_t Size() const { return buf_.size(); }
    const std::vector<uint8_t>& Data() const { return buf_; }

private:
    std::vector<uint8_t> buf_;
};

} // namespace shidou::codec
