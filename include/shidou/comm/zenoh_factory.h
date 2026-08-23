#pragma once

// Singleton owning the zenoh session and creating the typed comm objects.
//
// Lifecycle: Init once at process start, Shutdown once at process end.
// Create*/SetNamespace are thread-safe; the created objects hold their
// own reference to the session and outlive Shutdown harmlessly (their
// operations then report kNotConnected).

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <zenoh/api/session.hxx>

#include "shidou/comm/error.h"
#include "shidou/comm/keyexpr.h"
#include "shidou/comm/options.h"
#include "shidou/comm/qos.h"
#include "shidou/comm/zenoh_client.h"
#include "shidou/comm/zenoh_publisher.h"
#include "shidou/comm/zenoh_subscriber.h"
#include "shidou/logging.h"

namespace shidou::comm {

class ZenohFactory {
public:
    static ZenohFactory& Instance() {
        static ZenohFactory instance;
        return instance;
    }

    // Opens the zenoh session from the config. Repeated calls fail with
    // kAlreadyInitialized (call Shutdown first to re-init).
    bool Init(const ZenohConfig& cfg);

    // Closes the session. Safe to call from any thread that is not a
    // zenoh session callback.
    void Shutdown();

    // True while a session exists. A lost transport does not clear this:
    // failures surface through Publish/Call errors and the session-lost
    // callback instead.
    bool Healthy() const;

    // Switches the namespace prefix used by subsequent Create* calls.
    // Does not affect already-created objects. Thread-safe.
    void SetNamespace(const std::string& ns);

    std::string Namespace() const;

    // Creates a publisher for a ROS topic name (leading '/' optional).
    // Returns null on invalid name or session error; see LastError().
    template <typename T>
    std::shared_ptr<ZenohPublisher<T>> CreatePublisher(const std::string& topic,
                                                       const PubOptions& opts = {}) {
        std::string keyexpr;
        if (!BuildKeyexpr(topic, false, keyexpr)) {
            return nullptr;
        }
        const auto session = Session();
        if (!session) {
            SetError(ErrorCode::kNotInitialized, "Init not called");
            return nullptr;
        }
        try {
            zenoh::KeyExpr ke(keyexpr);
            auto pub = session->declare_publisher(ke, MakePublisherOptions(opts.reliability));
            // Direct new (not make_shared): the constructor is private and
            // friendship does not extend into make_shared's internal helper.
            return std::shared_ptr<ZenohPublisher<T>>(new ZenohPublisher<T>(
                session, keyexpr, std::move(pub),
                []() { ZenohFactory::Instance().NotifySessionLost(); }));
        } catch (const zenoh::ZException& e) {
            SetError(ErrorCode::kSessionError, std::string("declare_publisher: ") + e.what());
            return nullptr;
        }
    }

    // Creates a subscriber for a ROS topic name. The callback runs on a
    // zenoh session thread (see ZenohSubscriber). Returns null on invalid
    // name or session error.
    template <typename T>
    std::shared_ptr<ZenohSubscriber<T>> CreateSubscriber(const std::string& topic,
                                                         typename ZenohSubscriber<T>::Callback callback) {
        std::string keyexpr;
        if (!BuildKeyexpr(topic, false, keyexpr)) {
            return nullptr;
        }
        const auto session = Session();
        if (!session) {
            SetError(ErrorCode::kNotInitialized, "Init not called");
            return nullptr;
        }
        // The object owns the declared subscriber; the callback captures
        // its raw pointer. Destruction order inside ZenohSubscriber
        // undeclares (and synchronizes with in-flight callbacks) before
        // any other member dies, so the pointer stays valid.
        auto sub = std::shared_ptr<ZenohSubscriber<T>>(
            new ZenohSubscriber<T>(session, keyexpr, std::move(callback),
                                   zenoh::interop::detail::null<zenoh::Subscriber<void>>()));
        try {
            zenoh::KeyExpr ke(keyexpr);
            auto raw = sub.get();
            zenoh::Subscriber declared = session->declare_subscriber(
                ke,
                [raw](const zenoh::Sample& sample) { raw->OnSample(sample); },
                []() {});
            raw->ReplaceSubscriber(std::move(declared));
        } catch (const zenoh::ZException& e) {
            SetError(ErrorCode::kSessionError, std::string("declare_subscriber: ") + e.what());
            return nullptr;
        }
        return sub;
    }

    // Creates a request/response client for a ROS service name. Returns
    // null on invalid name or session error.
    template <typename TReq, typename TRes>
    std::shared_ptr<ZenohClient<TReq, TRes>> CreateClient(const std::string& service,
                                                          const ClientOptions& opts = {}) {
        std::string keyexpr;
        if (!BuildKeyexpr(service, true, keyexpr)) {
            return nullptr;
        }
        const auto session = Session();
        if (!session) {
            SetError(ErrorCode::kNotInitialized, "Init not called");
            return nullptr;
        }
        try {
            zenoh::KeyExpr ke(keyexpr);
            return std::shared_ptr<ZenohClient<TReq, TRes>>(new ZenohClient<TReq, TRes>(
                session, keyexpr, std::move(ke), opts.timeout,
                []() { ZenohFactory::Instance().NotifySessionLost(); }));
        } catch (const zenoh::ZException& e) {
            SetError(ErrorCode::kSessionError, std::string("keyexpr: ") + e.what());
            return nullptr;
        }
    }

    // Raw session access for tooling (e.g. the wire probe). Null when
    // not initialized.
    std::shared_ptr<zenoh::Session> SessionPtr() const;

    // Callback invoked once per transport-failure transition (a Publish
    // or Call that hit a dead session). Cleared on Shutdown.
    void SetSessionLostCallback(std::function<void()> cb);

    // Reports a transport failure to the session-lost callback (edge
    // triggered). Called by the comm objects on failed operations.
    void NotifySessionLost();

    ErrorCode LastErrorCode() const;
    // Returns a copy: the stored message is not synchronized with
    // concurrent operations.
    std::string LastError() const;

private:
    ZenohFactory() = default;

    // Snapshot of the current session under the lock.
    std::shared_ptr<zenoh::Session> Session() const;

    // Validates a topic or service name against the current namespace;
    // sets last_error_ on failure.
    bool BuildKeyexpr(const std::string& name, bool is_service, std::string& out);

    bool SetError(ErrorCode code, std::string message) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_code_ = code;
        last_error_ = std::move(message);
        return false;
    }

    mutable std::mutex mutex_;
    std::shared_ptr<zenoh::Session> session_;
    std::string ns_;
    std::function<void()> session_lost_cb_;
    std::atomic<bool> lost_notified_{false};
    ErrorCode last_code_ = ErrorCode::kOk;
    std::string last_error_;
};

} // namespace shidou::comm
