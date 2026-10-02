#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace inspire::command
{

inline constexpr std::size_t kHandDof = 6;
inline constexpr std::size_t kTotalDof = 12;
inline constexpr std::uint8_t kSafetyProtocolMode = 1;
inline constexpr double kTeleopClosingSpeedRaw = 500.0;
inline constexpr double kOpeningSpeedRaw = 1000.0;

struct ActuatorCommand
{
    std::uint8_t mode;
    double position;
    double speed_raw;
    double force_limit_g;
};

using HandTarget = std::array<double, kHandDof>;
using ActualState = std::array<double, kTotalDof>;
using CommandFrame = std::array<ActuatorCommand, kTotalDof>;

inline CommandFrame buildCommandFrame(
    const HandTarget& right_target,
    const HandTarget& left_target,
    const ActualState& actual_state,
    bool has_actual_state,
    double force_limit_g)
{
    CommandFrame frame{};
    for (std::size_t i = 0; i < kHandDof; ++i)
    {
        const bool right_opening =
            has_actual_state && right_target[i] >= actual_state[i];
        const bool left_opening =
            has_actual_state && left_target[i] >= actual_state[i + kHandDof];
        frame[i] = {
            kSafetyProtocolMode,
            right_target[i],
            right_opening ? kOpeningSpeedRaw : kTeleopClosingSpeedRaw,
            force_limit_g};
        frame[i + kHandDof] = {
            kSafetyProtocolMode,
            left_target[i],
            left_opening ? kOpeningSpeedRaw : kTeleopClosingSpeedRaw,
            force_limit_g};
    }
    return frame;
}

}  // namespace inspire::command
