#pragma once

// Type-safe request/response client over zenoh Queryables. One Call is
// one ROS 2 service invocation on the robot side: the request travels as
// a query payload and the reply carries the response, both in the topic
// frame (CDR encapsulation header + body). The bridge correlates each
// query with its reply internally, so the SDK needs no request id.
//
// Threading contract: Call blocks the calling thread until a reply
// arrives or the timeout elapses. It must NOT be called from a zenoh
// session thread (e.g. from a subscriber callback) -- that would stall
// message processing. Concurrent Calls are safe: zenoh correlates each
// query with its replies independently.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <zenoh/api/bytes.hxx>
#include <zenoh/api/reply.hxx>
#include <zenoh/api/session.hxx>

#include "shidou/comm/error.h"
#include "shidou/comm/options.h"
#include "shidou/comm/wire_codec.h"
#include "shidou/logging.h"

namespace shidou::comm {

template <typename TReq, typename TRes>
class ZenohClient {
public:
    // Blocks until a reply arrives or the configured timeout elapses.
    bool Call(const TReq& req, TRes& res) { return CallImpl(req, res, timeout_); }

    // As Call, but the timeout is overridden for this call only.
    bool Call(const TReq& req, TRes& res, std::chrono::milliseconds timeout) {
        return CallImpl(req, res, timeout);
    }

private:
    friend class ZenohSession;

    bool CallImpl(const TReq& req, TRes& res, std::chrono::milliseconds timeout) {
        if (!session_) {
            return Fail(ErrorCode::kNotConnected, "session is gone");
        }
        std::vector<uint8_t> frame;
        if (!EncodeServiceRequest(req, frame)) {
            return Fail(ErrorCode::kEncodeError, "encode failed");
        }

        // Shared with the reply closures: they may outlive this stack
        // frame (the query finishes when the last reply is processed),
        // so the promise and the decoded payload live on the heap.
        struct CallState {
            std::promise<ErrorCode> promise;
            std::optional<TRes> decoded;
            std::mutex mutex;  // guards decoded and undecodable
            std::atomic<bool> done{false};
            bool undecodable = false;  // an ok reply the decoder rejected
        };
        auto state = std::make_shared<CallState>();
        auto future = state->promise.get_future();

        zenoh::Session::GetOptions opts = zenoh::Session::GetOptions::create_default();
        opts.timeout_ms = static_cast<uint64_t>(timeout.count());
        opts.payload = zenoh::Bytes(std::move(frame));

        try {
            session_->get(
                service_keyexpr_,
                "",
                [state](zenoh::Reply& reply) {
                    if (state->done.load()) {
                        return;
                    }
                    if (!reply.is_ok()) {
                        // An error reply neither ends the query nor counts as
                        // an answer here: it is treated as silence, so a call
                        // whose only replies are errors ends as kNoReply (like
                        // one with no answerer), not with the responder's
                        // error.
                        return;
                    }
                    TRes candidate;
                    const zenoh::Bytes& payload = reply.get_ok().get_payload();
                    auto view = payload.get_contiguous_view();
                    if (!view || !DecodeServiceReply(view->data, view->len, candidate)) {
                        // An ok reply the decoder rejected. Remember it so the
                        // request that saw it can end as a decode failure
                        // instead of as "no reply"; a later decodable reply
                        // still wins.
                        std::lock_guard<std::mutex> lock(state->mutex);
                        state->undecodable = true;
                        return;
                    }
                    {
                        std::lock_guard<std::mutex> lock(state->mutex);
                        state->decoded = std::move(candidate);
                    }
                    state->done.store(true);
                    state->promise.set_value(ErrorCode::kOk);
                },
                [state]() {
                    // All replies are in. A reply that was seen but rejected by
                    // the decoder is this call's decode failure; with no reply
                    // at all the query reached no queryable (the promise may
                    // already be satisfied by a decodable reply).
                    if (!state->done.exchange(true)) {
                        ErrorCode code = ErrorCode::kNoReply;
                        {
                            std::lock_guard<std::mutex> lock(state->mutex);
                            if (state->undecodable) {
                                code = ErrorCode::kDecodeError;
                            }
                        }
                        state->promise.set_value(code);
                    }
                },
                std::move(opts));
        } catch (const zenoh::ZException& e) {
            notify_lost_();
            return Fail(ErrorCode::kNotConnected, std::string("get failed: ") + e.what());
        }

        if (future.wait_for(timeout + std::chrono::milliseconds(100)) != std::future_status::ready) {
            return Fail(ErrorCode::kTimeout, "no reply within timeout");
        }
        const ErrorCode code = future.get();
        if (code != ErrorCode::kOk) {
            return Fail(code, code == ErrorCode::kDecodeError ? "reply decode failed" : "no reply");
        }
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            res = std::move(*state->decoded);
        }
        return true;
    }

public:
    const std::string& Service() const { return service_; }
    // The most recent failure of this client: one record per object, not
    // synchronized with concurrent Call calls (concurrent Calls are safe, but
    // then the record may describe another call). LastError() returns a copy
    // of the stored string.
    ErrorCode LastErrorCode() const { return last_code_; }
    std::string LastError() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return last_error_;
    }

private:
    // notify_lost is injected by the session object that creates the client
    // (which owns the session-lost callback) so this class has no dependency
    // on the session type.
    ZenohClient(std::shared_ptr<zenoh::Session> session, std::string service,
                zenoh::KeyExpr service_keyexpr, std::chrono::milliseconds timeout,
                std::function<void()> notify_lost)
        : session_(std::move(session)),
          service_(std::move(service)),
          service_keyexpr_(std::move(service_keyexpr)),
          timeout_(timeout),
          notify_lost_(std::move(notify_lost)) {}

    bool Fail(ErrorCode code, std::string message) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_code_ = code;
        last_error_ = std::move(message);
        return false;
    }

    std::shared_ptr<zenoh::Session> session_;
    std::string service_;
    zenoh::KeyExpr service_keyexpr_;
    std::chrono::milliseconds timeout_;
    std::function<void()> notify_lost_;
    mutable std::mutex error_mutex_;  // mutable: LastError() is const
    ErrorCode last_code_ = ErrorCode::kOk;
    std::string last_error_;
};

} // namespace shidou::comm
