#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <chrono>

namespace phoenix {

/**
 * @brief タイムスタンプを取得する
 * @return タイムスタンプ
 */
builtin_interfaces::msg::Time getTimeStamp(void) {
    builtin_interfaces::msg::Time result;
    auto time = std::chrono::steady_clock::now();
    auto nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch());
    result.sec = nanosecs.count() / 1000000000;
    result.nanosec = nanosecs.count() % 1000000000;
    return result;
}

} // namespace phoenix
