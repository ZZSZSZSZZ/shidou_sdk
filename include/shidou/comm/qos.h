#pragma once

// QoS mapping: SDK reliability -> zenoh publication properties.
//
// This is the single point for QoS tuning. The mapping mirrors the
// robot-side ROS 2 QoS:
//   reliable   (fsm_command): congestion BLOCK, not express -- commands
//               must never be silently dropped, even under backpressure
//   best_effort (target streams at 100 Hz): congestion DROP, express,
//               DATA_HIGH priority -- a stale target must never delay a
//               fresh one
// Transient-local (joint_states) is not represented here: the SDK keeps
// its own last-value cache in ZenohSubscriber instead of using zenoh
// history.

#include <zenoh/api/session.hxx>

#include "shidou/comm/options.h"

namespace shidou::comm {

inline zenoh::Session::PublisherOptions MakePublisherOptions(Reliability reliability) {
    zenoh::Session::PublisherOptions opts = zenoh::Session::PublisherOptions::create_default();
    switch (reliability) {
        case Reliability::kReliable:
            opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
            opts.is_express = false;
            break;
        case Reliability::kBestEffort:
            opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
            opts.is_express = true;
            opts.priority = Z_PRIORITY_DATA_HIGH;
            break;
    }
    return opts;
}

} // namespace shidou::comm
