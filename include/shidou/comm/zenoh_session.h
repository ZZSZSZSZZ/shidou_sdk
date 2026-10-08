#pragma once

// One zenoh session: opening and closing it, its namespace prefix, and the
// typed comm objects created under it. An ordinary object, not a
// process-wide singleton - whoever opens it owns it.
//
// Ownership: hand it around as a std::shared_ptr<ZenohSession>; the last
// holder's release closes the session, so nothing has to be shut down at
// process end. Closing is synchronous and waits for a silent endpoint to be
// given up on - seconds when the address stopped answering, against
// milliseconds for a local session - so a release belongs where a wait is
// acceptable (scope exit), never on a zenoh session callback or a watchdog
// thread. Comm objects created here hold handles of their own on the zenoh
// session and keep the transport open: Close empties only this object, and
// what it created keeps working until the last of those handles is gone
// (their operations then report kNotConnected). Their session-lost
// notification goes back to the session object that created them, and
// nowhere once that object is gone.
//
// Threading: on an open object, Close/Create*/SetNamespace and the accessors
// are thread-safe, and calls on different ZenohSession objects are
// independent. Open is the exception: it must not run concurrently with Open
// or Close on the same object. Its duplicate check and the assignment of the
// new session are separated by the multi-second zenoh open and the mutex
// covers only those two points, so two concurrent calls would both open a
// session and the second assignment would silently drop the first (both
// callers see true). Calls that arrive once the session is there are refused
// as usual. Close is safe from any thread that is not a zenoh session
// callback; the destructor runs under the same rule.

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

class ZenohSession : public std::enable_shared_from_this<ZenohSession> {
public:
    ZenohSession() = default;
    ~ZenohSession();

    ZenohSession(const ZenohSession&) = delete;
    ZenohSession& operator=(const ZenohSession&) = delete;

    // Opens the zenoh session from the config. Repeated calls fail with
    // kAlreadyInitialized (call Close first to re-open); calls arriving after
    // the session is there are refused, but a call racing this one on the
    // same object is not (see the class comment).
    bool Open(const ZenohConfig& cfg);

    // Releases this object's handle on the session, leaving the object
    // openable again. It does not close the comm objects created from the
    // session: they hold handles of their own and keep working until the
    // last of them is gone (see the class comment). Safe to call from any
    // thread that is not a zenoh session callback.
    void Close();

    // True while a session exists. A lost transport does not clear this:
    // failures surface through Publish/Call errors and the session-lost
    // callback instead.
    bool Healthy() const;

    // Switches the namespace prefix used by subsequent Create* calls.
    // Does not affect already-created objects. Thread-safe. The namespace is
    // this object's, so everyone running on the session shares it: a switch
    // moves the prefix that any later Create* call builds under, whichever
    // holder makes it.
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
            SetError(ErrorCode::kNotInitialized, "session is not open");
            return nullptr;
        }
        try {
            zenoh::KeyExpr ke(keyexpr);
            auto pub = session->declare_publisher(ke, MakePublisherOptions(opts.reliability));
            // Direct new (not make_shared): the constructor is private and
            // friendship does not extend into make_shared's internal helper.
            return std::shared_ptr<ZenohPublisher<T>>(
                new ZenohPublisher<T>(session, keyexpr, std::move(pub), MakeLostNotifier()));
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
            SetError(ErrorCode::kNotInitialized, "session is not open");
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
            SetError(ErrorCode::kNotInitialized, "session is not open");
            return nullptr;
        }
        try {
            zenoh::KeyExpr ke(keyexpr);
            return std::shared_ptr<ZenohClient<TReq, TRes>>(
                new ZenohClient<TReq, TRes>(session, keyexpr, std::move(ke), opts.timeout,
                                            MakeLostNotifier()));
        } catch (const zenoh::ZException& e) {
            SetError(ErrorCode::kSessionError, std::string("keyexpr: ") + e.what());
            return nullptr;
        }
    }

    // Raw session access for tooling (e.g. the wire probe). Null when
    // not open.
    std::shared_ptr<zenoh::Session> SessionPtr() const;

    // Callback invoked once per transport-failure transition (a Publish
    // or Call that hit a dead session). Cleared on Close.
    void SetSessionLostCallback(std::function<void()> cb);

    // Reports a transport failure to the session-lost callback (edge
    // triggered). Called by the comm objects on failed operations.
    void NotifySessionLost();

    ErrorCode LastErrorCode() const;
    // Returns a copy: the stored message is not synchronized with
    // concurrent operations.
    std::string LastError() const;

private:
    // Snapshot of the current session under the lock.
    std::shared_ptr<zenoh::Session> Session() const;

    // Validates a topic or service name against the current namespace;
    // sets last_error_ on failure.
    bool BuildKeyexpr(const std::string& name, bool is_service, std::string& out);

    // Builds the session-lost callback handed to the comm objects created
    // here. It reports to this object while that object is alive and does
    // nothing afterwards: the comm objects may outlive it.
    std::function<void()> MakeLostNotifier();

    bool SetError(ErrorCode code, std::string message) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_code_ = code;
        last_error_ = std::move(message);
        return false;
    }

    // Releases the session and the state tied to it; true when there was
    // one. Log-free, so the destructor can use it.
    bool ReleaseSession();

    mutable std::mutex mutex_;
    std::shared_ptr<zenoh::Session> session_;
    std::string ns_;
    std::function<void()> session_lost_cb_;
    std::atomic<bool> lost_notified_{false};
    ErrorCode last_code_ = ErrorCode::kOk;
    std::string last_error_;
};

} // namespace shidou::comm
