#ifndef INSPIRE_FEEDBACK_PROTOCOL_HPP
#define INSPIRE_FEEDBACK_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace inspire
{
namespace protocol
{

// reserve[0] identifies protocol support; it does not report sensor health.
// reserve[1] reports whether the current force sample is valid for the hand.
inline constexpr std::size_t SAFETY_PROTOCOL_MAGIC_INDEX = 0;
inline constexpr std::size_t FORCE_FEEDBACK_STATUS_INDEX = 1;
inline constexpr std::uint32_t SAFETY_PROTOCOL_MAGIC = 0x494E5350;  // "INSP"
inline constexpr std::uint32_t FORCE_FEEDBACK_VALID = 1;

inline bool supportsSafetyProtocol(std::uint32_t magic)
{
    return magic == SAFETY_PROTOCOL_MAGIC;
}

inline bool isForceFeedbackValid(
    std::uint32_t magic,
    std::uint32_t status)
{
    return supportsSafetyProtocol(magic) &&
        status == FORCE_FEEDBACK_VALID;
}

inline std::array<bool, 2> decodeForceFeedbackValidity(
    std::uint32_t right_magic,
    std::uint32_t right_status,
    std::uint32_t left_magic,
    std::uint32_t left_status)
{
    return {
        isForceFeedbackValid(right_magic, right_status),
        isForceFeedbackValid(left_magic, left_status)
    };
}

}  // namespace protocol
}  // namespace inspire

#endif  // INSPIRE_FEEDBACK_PROTOCOL_HPP
