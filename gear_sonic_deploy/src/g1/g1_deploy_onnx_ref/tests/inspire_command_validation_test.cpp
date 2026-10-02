#include "../include/inspire_command_validation.hpp"

#include <cassert>
#include <cstdint>
#include <cstring>

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

    assert(isValidForceLimitGrams(1.0));
    assert(isValidForceLimitGrams(100.0));
    assert(isValidForceLimitGrams(1000.0));
    assert(!isValidForceLimitGrams(0.0));
    assert(!isValidForceLimitGrams(1001.0));
    assert(!isValidForceLimitGrams(doubleFromBits(0x7ff8000000000000ULL)));
    assert(!isValidForceLimitGrams(doubleFromBits(0x7ff0000000000000ULL)));
    assert(!isValidForceLimitGrams(doubleFromBits(0xfff0000000000000ULL)));
    return 0;
}
