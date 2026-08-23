#pragma once

// Central logging facility.
//
// All SDK code logs through spdlog loggers named after components ("comm",
// "robot", "codec"). Loggers are created lazily on first use; InitLogging()
// applies the configured level to the global registry and to every logger
// that is created afterwards.

#include <memory>
#include <string>

#include <spdlog/spdlog.h>

namespace shidou {

// Applies `level` (trace/debug/info/warn/error/off) to the global registry
// and future loggers. Safe to call multiple times; the last call wins.
void InitLogging(const std::string& level);

// Returns the logger for `component`, creating it on first use.
// Never returns null.
std::shared_ptr<spdlog::logger> Log(const std::string& component);

} // namespace shidou
