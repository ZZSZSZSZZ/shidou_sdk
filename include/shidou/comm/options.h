#pragma once

// Configuration and per-role options for the comm layer.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace shidou::comm {

// Factory defaults of the reference robot: the address its bridge listens
// on and the keyexpr namespace it publishes under. Both are changeable
// after shipment - the address follows the site network, and the namespace
// is what separates robots that share one network - so code that overrides
// either half must pass it explicitly.
inline constexpr const char* kDefaultRobotAddress = "192.168.168.168:7447";
inline constexpr const char* kDefaultNamespace = "robot168";

// Wire reliability of a publication, mirroring the ROS 2 QoS of the
// matching robot-side topic. The mapping to zenoh properties lives in
// qos.h (single point for QoS tuning).
enum class Reliability {
    kReliable,    // never silently drop (congestion BLOCK)
    kBestEffort,  // target streams: newest wins (congestion DROP, DATA_HIGH)
};

// Session configuration for ZenohSession::Open.
struct ZenohConfig {
    // Robot address to connect to, "<ip>:<port>". The robot-side
    // zenoh-plugin-ros2dds bridge listens on TCP 7447 in router mode; the
    // comm layer connects to it as "tcp/<robot_address>". Required in
    // client mode.
    std::string robot_address = kDefaultRobotAddress;

    // "client" (connect to a router/peer only) or "peer" (participate in
    // routing). Client mode is the normal SDK mode; peer mode is useful
    // for loopback testing without a router.
    std::string mode = "client";

    // Enables multicast scouting as a fallback discovery mechanism.
    // Off by default: the explicit robot address is the reliable path.
    bool enable_scouting = false;

    // Keyexpr prefix for all topics and services. Must match the
    // `namespace` config of the robot-side plugin exactly; empty means no
    // prefix. Defaults to the factory namespace, so a robot that still
    // carries its factory settings answers without any configuration.
    std::string namespace_ = kDefaultNamespace;

    // spdlog level for SDK components ("trace".."off").
    std::string log_level = "info";
};

// Publisher options.
struct PubOptions {
    Reliability reliability = Reliability::kReliable;
};

// Client (request/response) options.
struct ClientOptions {
    // How long Call waits for a reply before reporting kTimeout.
    std::chrono::milliseconds timeout = std::chrono::seconds(5);
};

// Options for the retained-sample subscriber flavour
// (ZenohSession::CreateRetainedSubscriber). They describe the history query
// sent to a publisher that keeps a cache for transient-local topics.
struct RetainedSubscriberOptions {
    // Cached samples to request from each publisher. Description snapshots
    // are complete values, so the latest one is enough.
    size_t max_samples = 1;

    // Oldest sample to accept, relative to its publication time. Zero
    // means no age limit.
    std::chrono::milliseconds max_age{0};

    // How long the history query may take before it is abandoned. Keeps a
    // late subscriber from stalling on an unresponsive publisher.
    std::chrono::milliseconds query_timeout = std::chrono::seconds(3);
};

} // namespace shidou::comm
