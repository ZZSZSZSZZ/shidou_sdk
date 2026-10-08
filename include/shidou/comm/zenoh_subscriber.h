#pragma once

// Type-safe subscriber with a last-value cache (approximating the
// transient_local semantics of the robot-side joint_states publisher:
// the SDK keeps its own latest sample instead of relying on zenoh
// history).
//
// Threading contract:
//   - the user callback runs on a zenoh session thread: it must not
//     block, and must not close the session or destroy the subscriber
//   - LastValue/Seq/AgeMs are safe to call from any thread, including
//     from the callback itself
//   - malformed payloads are dropped, counted, logged rate-limited
//     (1 Hz max) and reported through the error handler

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <zenoh/api/sample.hxx>
#include <zenoh/api/session.hxx>
#include <zenoh/api/subscriber.hxx>

#include "shidou/comm/error.h"
#include "shidou/comm/wire_codec.h"
#include "shidou/logging.h"

namespace shidou::comm {

template <typename T>
class ZenohSubscriber {
public:
    using Callback = std::function<void(const T&)>;

    // Copies the most recently decoded sample. False if none arrived yet.
    bool LastValue(T& out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!last_) {
            return false;
        }
        out = *last_;
        return true;
    }

    // Number of successfully decoded samples received.
    uint64_t Seq() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return seq_;
    }

    // Milliseconds since the last successfully decoded sample;
    // negative if none arrived yet.
    double AgeMs() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (seq_ == 0) {
            return -1.0;
        }
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - last_ts_).count();
    }

    // Number of payloads rejected by decoding.
    uint64_t DecodeErrors() const { return decode_errors_.load(); }

    // Hook for asynchronous decode failures; invoked at most once per
    // second while failures keep arriving. May run on a zenoh thread.
    void SetErrorHandler(std::function<void(ErrorCode)> handler) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_handler_ = std::move(handler);
    }

    const std::string& Topic() const { return topic_; }

private:
    friend class ZenohSession;

    ZenohSubscriber(std::shared_ptr<zenoh::Session> session, std::string topic, Callback callback,
                    zenoh::Subscriber<void> subscriber)
        : session_(std::move(session)),
          topic_(std::move(topic)),
          callback_(std::move(callback)),
          subscriber_(std::move(subscriber)) {}

    // Runs on a zenoh session thread.
    void OnSample(const zenoh::Sample& sample) {
        T msg;
        const zenoh::Bytes& payload = sample.get_payload();
        auto view = payload.get_contiguous_view();
        if (!view || !DecodeTopicFrame(view->data, view->len, msg)) {
            ++decode_errors_;
            ReportError(ErrorCode::kDecodeError, "decode failed on " + topic_);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_ = msg;
            ++seq_;
            last_ts_ = std::chrono::steady_clock::now();
        }
        if (callback_) {
            callback_(msg);  // outside the lock: the callback may query the cache
        }
    }

    void ReportError(ErrorCode code, const std::string& message);

    // Called once by the session after a successful declaration. Must
    // precede any sample delivery for this object.
    void ReplaceSubscriber(zenoh::Subscriber<void> subscriber) { subscriber_ = std::move(subscriber); }

    // Destruction order matters: subscriber_ is declared last, so it is
    // destroyed first. Undeclaring is synchronized with in-flight
    // callbacks by zenoh-c, so no callback runs while the cache below is
    // being torn down.
    std::shared_ptr<zenoh::Session> session_;
    std::string topic_;
    Callback callback_;
    mutable std::mutex mutex_;
    std::optional<T> last_;
    uint64_t seq_ = 0;
    std::chrono::steady_clock::time_point last_ts_;
    std::atomic<uint64_t> decode_errors_{0};
    std::mutex error_mutex_;
    std::function<void(ErrorCode)> error_handler_;
    std::chrono::steady_clock::time_point last_error_ts_;
    zenoh::Subscriber<void> subscriber_;
};

template <typename T>
inline void ZenohSubscriber<T>::ReportError(ErrorCode code, const std::string& message) {
    // Rate-limit both the log line and the user hook to once per second.
    std::function<void(ErrorCode)> handler;
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (now - last_error_ts_ < std::chrono::seconds(1)) {
            return;
        }
        last_error_ts_ = now;
        handler = error_handler_;
    }
    Log("comm")->warn("{} ({} errors total)", message, decode_errors_.load());
    if (handler) {
        handler(code);
    }
}

} // namespace shidou::comm
