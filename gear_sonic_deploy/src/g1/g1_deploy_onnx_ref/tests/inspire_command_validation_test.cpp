#include "../include/inspire_command_validation.hpp"
#include "../include/inspire_command_frame.hpp"
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
    using inspire::validation::parseForceLimitGrams;
    using inspire::command::buildCommandFrame;
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

    CHECK(parseForceLimitGrams("100").value_or(0.0) == 100.0);
    CHECK(parseForceLimitGrams("1").value_or(0.0) == 1.0);
    CHECK(parseForceLimitGrams("1000").value_or(0.0) == 1000.0);
    CHECK(!parseForceLimitGrams("0"));
    CHECK(!parseForceLimitGrams("1001"));
    CHECK(!parseForceLimitGrams("nan"));
    CHECK(!parseForceLimitGrams("inf"));
    CHECK(!parseForceLimitGrams("100abc"));
    CHECK(!parseForceLimitGrams(""));

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

    const std::array<double, 6> right_target = {
        0.9, 0.4, 0.8, 0.3, 0.7, 0.2};
    const std::array<double, 6> left_target = {
        0.1, 0.6, 0.2, 0.7, 0.3, 0.8};
    const std::array<double, 12> actual_state = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    const auto frame = buildCommandFrame(
        right_target, left_target, actual_state, true, 100.0);

    for (std::size_t i = 0; i < 6; ++i)
    {
        CHECK(frame[i].position == right_target[i]);
        CHECK(frame[i + 6].position == left_target[i]);
        CHECK(frame[i].mode == 1);
        CHECK(frame[i + 6].mode == 1);
        CHECK(frame[i].force_limit_g == 100.0);
        CHECK(frame[i + 6].force_limit_g == 100.0);
        CHECK(frame[i].speed_raw ==
            (right_target[i] >= actual_state[i] ? 1000.0 : 500.0));
        CHECK(frame[i + 6].speed_raw ==
            (left_target[i] >= actual_state[i + 6] ? 1000.0 : 500.0));
    }

    const auto no_feedback_frame = buildCommandFrame(
        right_target, left_target, actual_state, false, 250.0);
    for (const auto& command : no_feedback_frame)
    {
        CHECK(command.speed_raw == 500.0);
        CHECK(command.force_limit_g == 250.0);
    }
    return 0;
}
