#pragma once

// Telemetry cache and stale watchdog for the robot layer. OnFeedback and
// OnFsmState are fed by the subscribers (on zenoh session threads); the
// watchdog thread only reads the cache and fires the stale callback.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "shidou/codec/msg_codecs.h"
#include "shidou/robot/defaults.h"

namespace shidou::robot {

class Telemetry {
public:
    Telemetry() = default;
    ~Telemetry() { Stop(); }

    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    void OnFeedback(const msg::JointFeedback& fb);
    void OnFsmState(const std::string& state);

    bool LastFeedback(msg::JointFeedback& out) const;
    double FeedbackAgeMs() const;  // negative until the first sample
    uint64_t FeedbackSeq() const;
    std::string FsmState() const;

    // User hooks, invoked on zenoh session threads (OnFeedback/OnFsmState
    // callers); they must not block.
    void SetFeedbackCallback(std::function<void(const msg::JointFeedback&)> cb);
    void SetFsmStateCallback(std::function<void(const std::string&)> cb);

    // Stale watchdog: cb(age_ms) fires once when no sample arrives within
    // age_threshold_ms, and again only after a fresh sample arrives. A
    // zero threshold disables the watchdog.
    void SetStaleCallback(std::chrono::milliseconds age_threshold_ms,
                          std::function<void(double age_ms)> cb);

    // Clears the cached samples, counters and fsm state. Called when the
    // robot switches namespace: the previous robot's data must not be
    // served as the new robot's state. Callbacks and watchdog settings
    // survive.
    void Reset();

    // Starts the watchdog thread; idempotent. Stop() joins it.
    void Start();
    void Stop();

private:
    void WatchdogLoop();

    mutable std::mutex mutex_;
    std::optional<msg::JointFeedback> last_;
    uint64_t seq_ = 0;
    std::chrono::steady_clock::time_point last_ts_;
    std::string fsm_state_;
    std::function<void(const msg::JointFeedback&)> feedback_cb_;
    std::function<void(const std::string&)> fsm_cb_;
    std::chrono::milliseconds stale_threshold_{0};
    std::function<void(double)> stale_cb_;
    bool stale_notified_ = true;  // nothing to report until a sample arrives
    std::thread watchdog_;
    std::atomic<bool> stop_{false};
};

} // namespace shidou::robot
