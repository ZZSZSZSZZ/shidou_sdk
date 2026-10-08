#pragma once

// Robot-layer telemetry view: user hooks plus the stale watchdog. The
// latest feedback sample, its sequence and its age are owned by the
// comm-layer subscriber cache (ZenohSubscriber) - the single owner; this
// class keeps no copy of its own. OnFeedback and OnFsmState are fed by the
// subscribers (on zenoh session threads); the watchdog thread polls the
// attached subscriber's age and fires the stale callback. Whether a sample
// arrived since the last report is read from the source's sequence counter,
// so the watchdog needs nothing but AttachFeedbackSource to observe fresh
// samples.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "shidou/codec/msg_codecs.h"
#include "shidou/robot/defaults.h"

namespace shidou::comm {

template <typename T>
class ZenohSubscriber;

} // namespace shidou::comm

namespace shidou::robot {

class Telemetry {
public:
    Telemetry() = default;
    ~Telemetry() { Stop(); }

    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    // Points the watchdog at the sample cache it reports on. Called whenever
    // the comm objects are (re)created, under this object's lock. The weak
    // reference does not extend the subscriber's lifetime: an expired source
    // simply means "nothing to report". Samples already in the cache do not
    // count as fresh: only a sample arriving after this call arms the
    // watchdog.
    void AttachFeedbackSource(std::weak_ptr<comm::ZenohSubscriber<msg::JointFeedback>> sub);

    // Delivery hooks, fed by the subscribers on zenoh session threads.
    void OnFeedback(const msg::JointFeedback& fb);
    void OnFsmState(const std::string& state);

    // User hooks, invoked on zenoh session threads (OnFeedback/OnFsmState
    // callers); they must not block.
    void SetFeedbackCallback(std::function<void(const msg::JointFeedback&)> cb);
    void SetFsmStateCallback(std::function<void(const std::string&)> cb);

    // Stale watchdog: cb(age_ms) fires once when no sample arrives within
    // age_threshold_ms, and again only after a fresh sample arrives. A
    // zero threshold disables the watchdog.
    void SetStaleCallback(std::chrono::milliseconds age_threshold_ms,
                          std::function<void(double age_ms)> cb);

    // Starts the watchdog thread; idempotent. Stop() joins it.
    void Start();
    void Stop();

private:
    void WatchdogLoop();

    mutable std::mutex mutex_;
    std::weak_ptr<comm::ZenohSubscriber<msg::JointFeedback>> feedback_source_;
    std::function<void(const msg::JointFeedback&)> feedback_cb_;
    std::function<void(const std::string&)> fsm_cb_;
    std::chrono::milliseconds stale_threshold_{0};
    std::function<void(double)> stale_cb_;
    // Source sequence the watchdog last reported on; 0 means no sample has
    // been seen yet. Reporting advances it, so a report covers one sample
    // and the next sample arms the watchdog again.
    uint64_t stale_reported_seq_ = 0;
    std::thread watchdog_;
    std::atomic<bool> stop_{false};
};

} // namespace shidou::robot
