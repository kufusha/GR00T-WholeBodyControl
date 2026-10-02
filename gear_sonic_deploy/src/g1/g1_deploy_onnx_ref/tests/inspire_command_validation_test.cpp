#include "../include/inspire_command_validation.hpp"
#include "../include/inspire_feedback_protocol.hpp"

#include <cstdint>
#include <cstring>
#include <iostream>

#define CHECK(condition)                                                    \
    do                                                                      \
    {                                                                       \
        if (!(condition))                                                   \
        {                                                                   \
            std::cerr << "CHECK failed: " #condition << '\n';              \
            return 1;                                                       \
        }                                                                   \
    } while (false)

namespace
{

double doubleFromBits(std::uint64_t bits)
{
    double value = 0.0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace

int main()
{
    using inspire::validation::isValidForceLimitGrams;
    using inspire::protocol::FORCE_FEEDBACK_VALID;
    using inspire::protocol::SAFETY_PROTOCOL_MAGIC;
    using inspire::protocol::decodeForceFeedbackValidity;
    using inspire::protocol::isForceFeedbackValid;

    CHECK(isValidForceLimitGrams(1.0));
    CHECK(isValidForceLimitGrams(100.0));
    CHECK(isValidForceLimitGrams(1000.0));
    CHECK(!isValidForceLimitGrams(0.0));
    CHECK(!isValidForceLimitGrams(1001.0));
    CHECK(!isValidForceLimitGrams(doubleFromBits(0x7ff8000000000000ULL)));
    CHECK(!isValidForceLimitGrams(doubleFromBits(0x7ff0000000000000ULL)));
    CHECK(!isValidForceLimitGrams(doubleFromBits(0xfff0000000000000ULL)));

    // Protocol support alone must not be interpreted as healthy feedback.
    CHECK(!isForceFeedbackValid(SAFETY_PROTOCOL_MAGIC, 0));
    CHECK(isForceFeedbackValid(
        SAFETY_PROTOCOL_MAGIC, FORCE_FEEDBACK_VALID));
    CHECK(!isForceFeedbackValid(0, FORCE_FEEDBACK_VALID));

    const auto left_only_valid = decodeForceFeedbackValidity(
        0, FORCE_FEEDBACK_VALID,
        SAFETY_PROTOCOL_MAGIC, FORCE_FEEDBACK_VALID);
    CHECK(!left_only_valid[0]);
    CHECK(left_only_valid[1]);

    const auto right_only_valid = decodeForceFeedbackValidity(
        SAFETY_PROTOCOL_MAGIC, FORCE_FEEDBACK_VALID,
        SAFETY_PROTOCOL_MAGIC, 0);
    CHECK(right_only_valid[0]);
    CHECK(!right_only_valid[1]);
    return 0;
}
