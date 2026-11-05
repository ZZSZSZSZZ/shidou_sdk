#pragma once

// Default timing for the robot-layer handshakes and calls. The handshakes
// send one run_command request and then wait for the matching fsm_state with
// a bounded budget. Single point to retune per robot.

#include <chrono>

namespace shidou::robot {

constexpr std::chrono::milliseconds kEnableTimeout{5000};
constexpr std::chrono::milliseconds kStopTimeout{5000};
// A mode switch is queued by the robot until the current motion settles, so
// the budget has to cover that wait as well as the state transition itself.
constexpr std::chrono::milliseconds kModeSwitchTimeout{10000};
constexpr std::chrono::milliseconds kGetStateTimeout{5000};
// One asset fetch is one service round trip. The default matches the state
// query; a caller fetching a large asset over a slow link can override it per
// call.
constexpr std::chrono::milliseconds kGetAssetTimeout{5000};

// Enable() resends an `enable` that the robot answered with rejected_moving
// (the arm is still moving in a mode state) at this interval, until its own
// timeout runs out.
constexpr std::chrono::milliseconds kRejectedMovingRetry{50};

// Polling period of the telemetry stale watchdog.
constexpr std::chrono::milliseconds kStalePollPeriod{50};

} // namespace shidou::robot
