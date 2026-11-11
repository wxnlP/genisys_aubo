#pragma once

#ifndef FMT_HEADER_ONLY
  #define FMT_HEADER_ONLY
#endif

#include <fmt/core.h>
#include <fmt/format.h>

#include <atomic>
#include <cstdio>
#include <string_view>

// 加到文件顶部
namespace Ansi {
constexpr auto kReset = "\033[0m";
constexpr auto kRed = "\033[31m";
constexpr auto kGreen = "\033[32m";
constexpr auto kYellow = "\033[33m";
constexpr auto kCyan = "\033[36m";
}  // namespace Ansi

enum class LogLevel {
  DEBUG = 0,
  INFO,
  WARN,
  ERROR,
};

class ArmLogger {
 public:
  static void SetLogLevel(LogLevel level) { log_level_.store(level); }

  static LogLevel GetLogLevel() { return log_level_.load(); }

  static bool ShouldLog(LogLevel level) {
    return static_cast<int>(level) >= static_cast<int>(GetLogLevel());
  }

  static constexpr std::string_view LevelName(LogLevel level) {
    switch (level) {
      case LogLevel::DEBUG:
        return "DEBUG";
      case LogLevel::INFO:
        return "INFO";
      case LogLevel::WARN:
        return "WARN";
      case LogLevel::ERROR:
        return "ERROR";
    }
    return "UNKNOWN";
  }

  template <typename... Args>
  static void Log(LogLevel level, fmt::format_string<Args...> fmt_str,
                  Args&&... args) {
    if (!ShouldLog(level)) {
      return;
    }
    const char* color = Ansi::kReset;
    if (level == LogLevel::DEBUG) { color = Ansi::kCyan; }
    if (level == LogLevel::INFO) { color = Ansi::kGreen; }
    if (level == LogLevel::WARN) { color = Ansi::kYellow; }
    if (level == LogLevel::ERROR) { color = Ansi::kRed; }

    const auto message = fmt::format(fmt_str, std::forward<Args>(args)...);
    FILE* output = (level == LogLevel::WARN || level == LogLevel::ERROR)
                       ? stderr
                       : stdout;
    fmt::print(output, "{}[{}] {} {}\n", color, LevelName(level), message, Ansi::kReset);
  }

 private:
  inline static std::atomic<LogLevel> log_level_{LogLevel::INFO};
};

#define ARM_LOG_DEBUG(fmt, ...) \
  ArmLogger::Log(LogLevel::DEBUG, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
#define ARM_LOG_INFO(fmt, ...) \
  ArmLogger::Log(LogLevel::INFO, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
#define ARM_LOG_WARN(fmt, ...) \
  ArmLogger::Log(LogLevel::WARN, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
#define ARM_LOG_ERROR(fmt, ...) \
  ArmLogger::Log(LogLevel::ERROR, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
