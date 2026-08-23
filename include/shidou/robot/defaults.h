#pragma once

// Default timing for the robot-layer handshakes and calls. The values
// follow the robot-side action engine conventions (command -> wait for
// fsm_state with a bounded budget). Single point to retune per robot.

#include <chrono>

namespace shidou::robot {

constexpr std::chrono::milliseconds kEnableTimeout{5000};
constexpr std::chrono::milliseconds kStopTimeout{5000};
constexpr std::chrono::milliseconds kModeSwitchTimeout{2000};
constexpr std::chrono::milliseconds kGetStateTimeout{5000};
constexpr std::chrono::milliseconds kTrajectoryUploadTimeout{10000};

// Polling period of the telemetry stale watchdog.
constexpr std::chrono::milliseconds kStalePollPeriod{50};

} // namespace shidou::robot
