#pragma once

// A zenoh payload as one contiguous byte span.
//
// Zenoh may deliver a payload as several slices: a message that crosses the
// transport's batch size arrives fragmented, and Bytes::get_contiguous_view()
// fails for it. Decoders want one buffer, so a fragmented payload is copied
// into one here; a contiguous payload keeps its in-place view and pays
// nothing. The view points into the caller's payload, so that payload must
// outlive this object.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <zenoh/api/bytes.hxx>

namespace shidou::comm {

class ContiguousPayload {
public:
    explicit ContiguousPayload(const zenoh::Bytes& payload)
        : view_(payload.get_contiguous_view()) {
        if (!view_) {
            owned_ = payload.as_vector();
        }
    }

    const uint8_t* data() const { return view_ ? view_->data : owned_.data(); }
    size_t size() const { return view_ ? view_->len : owned_.size(); }

private:
    std::optional<zenoh::Slice> view_;
    std::vector<uint8_t> owned_;  // the copy, empty when the view works
};

} // namespace shidou::comm
