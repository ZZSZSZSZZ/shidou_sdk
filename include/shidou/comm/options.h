#pragma once

// Configuration and per-role options for the comm layer.

#include <chrono>
#include <cstdint>
#include <string>

namespace shidou::comm {

// Wire reliability of a publication, mirroring the ROS 2 QoS of the
// matching robot-side topic. The mapping to zenoh properties lives in
// qos.h (single point for QoS tuning).
enum class Reliability {
    kReliable,    // fsm_command: never silently drop (congestion BLOCK)
    kBestEffort,  // target streams: newest wins (congestion DROP, DATA_HIGH)
};

// Session configuration for ZenohSession::Open.
struct ZenohConfig {
    // Robot address to connect to, "<ip>:<port>". The robot-side
    // zenoh-plugin-ros2dds bridge listens on TCP 7447 in router mode; the
    // comm layer connects to it as "tcp/<robot_address>". Required in
    // client mode.
    std::string robot_address = "192.168.168.168:7447";

    // "client" (connect to a router/peer only) or "peer" (participate in
    // routing). Client mode is the normal SDK mode; peer mode is useful
    // for loopback testing without a router.
    std::string mode = "client";

    // Enables multicast scouting as a fallback discovery mechanism.
    // Off by default: the explicit robot address is the reliable path.
    bool enable_scouting = false;

    // Keyexpr prefix for all topics and services. Must match the
    // `namespace` config of the robot-side plugin exactly; empty means no
    // prefix.
    std::string namespace_ = "";

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

} // namespace shidou::comm
