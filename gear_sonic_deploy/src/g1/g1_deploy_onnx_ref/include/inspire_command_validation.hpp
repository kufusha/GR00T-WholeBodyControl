#pragma once

#include <cerrno>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace inspire::validation
{

inline bool hasFiniteBitPattern(double value)
{
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    static_assert(std::numeric_limits<double>::is_iec559);
    std::memcpy(&bits, &value, sizeof(bits));
    constexpr std::uint64_t kExponentMask = 0x7ff0000000000000ULL;
    return (bits & kExponentMask) != kExponentMask;
}

inline bool isValidForceLimitGrams(double value)
{
    // The deploy target uses -ffast-math, which may fold std::isfinite() to
    // true. Inspect the IEEE-754 exponent bits before performing range checks.
    return hasFiniteBitPattern(value) && value >= 1.0 && value <= 1000.0;
}

inline std::optional<double> parseForceLimitGrams(std::string_view text)
{
    if (text.empty())
    {
        return std::nullopt;
    }

    const std::string owned(text);
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(owned.c_str(), &end);
    if (errno == ERANGE || end != owned.c_str() + owned.size() ||
        !isValidForceLimitGrams(value))
    {
        return std::nullopt;
    }
    return value;
}

}  // namespace inspire::validation
