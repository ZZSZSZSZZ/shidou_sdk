#pragma once

// Type-safe publisher: encodes a message (CDR body + topic frame) and
// puts it on the session. Never throws; failures are reported through
// LastErrorCode()/LastError().

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <zenoh/api/publisher.hxx>
#include <zenoh/api/session.hxx>

#include "shidou/comm/error.h"
#include "shidou/comm/wire_codec.h"
#include "shidou/logging.h"

namespace shidou::comm {

template <typename T>
class ZenohPublisher {
public:
    // Encodes and publishes one message. Thread-safe: zenoh put is safe
    // for concurrent use; multiple publishing threads are allowed,
    // although the robot side expects one control stream per mode.
    bool Publish(const T& msg) {
        std::vector<uint8_t> frame;
        if (!EncodeTopicFrame(msg, frame)) {
            return Fail(ErrorCode::kEncodeError, "encode failed");
        }
        if (!session_) {
            return Fail(ErrorCode::kNotConnected, "session is gone");
        }
        zenoh::ZResult res = Z_OK;
        publisher_.put(zenoh::Bytes(std::move(frame)), {}, &res);
        if (res != Z_OK) {
            notify_lost_();
            return Fail(ErrorCode::kNotConnected, "zenoh put failed");
        }
        return true;
    }

    const std::string& Topic() const { return topic_; }
    ErrorCode LastErrorCode() const { return last_code_; }
    // Returns a copy: the stored message is not synchronized with
    // concurrent operations.
    std::string LastError() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return last_error_;
    }

private:
    friend class ZenohFactory;

    // notify_lost is injected by the factory (which owns the session-lost
    // callback) so this class has no dependency on the factory type.
    ZenohPublisher(std::shared_ptr<zenoh::Session> session, std::string topic,
                   zenoh::Publisher publisher, std::function<void()> notify_lost)
        : session_(std::move(session)),
          topic_(std::move(topic)),
          publisher_(std::move(publisher)),
          notify_lost_(std::move(notify_lost)) {}

    // Member order matters: publisher_ is destroyed first (undeclaring
    // the publication) before the session it belongs to.
    bool Fail(ErrorCode code, std::string message) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_code_ = code;
        last_error_ = std::move(message);
        return false;
    }

    std::shared_ptr<zenoh::Session> session_;
    std::string topic_;
    zenoh::Publisher publisher_;
    std::function<void()> notify_lost_;
    mutable std::mutex error_mutex_;  // mutable: LastError() is const
    ErrorCode last_code_ = ErrorCode::kOk;
    std::string last_error_;
};

} // namespace shidou::comm
