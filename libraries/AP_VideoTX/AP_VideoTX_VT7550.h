#pragma once

#include <stdint.h>

// Skyzone's supplied CUSTOM table uses MHz and opaque Tramp power codes.
// HIG/MAX are labels, not verified RF wattages.
namespace VT7550 {
static constexpr uint16_t MIN_MHZ = 6240;
static constexpr uint16_t MAX_MHZ = 7500;
static constexpr uint8_t POWER_COUNT = 5;

inline bool valid_frequency(uint16_t mhz, uint32_t device_min = 0, uint32_t device_max = 0)
{
    if (mhz < MIN_MHZ || mhz > MAX_MHZ || (mhz - MIN_MHZ) % 20 != 0) {
        return false;
    }
    return device_min == 0 || device_max == 0 || device_min > device_max ||
           (mhz >= device_min && mhz <= device_max);
}

inline bool valid_level(int8_t level)
{
    return level >= 0 && level < POWER_COUNT;
}

inline uint16_t power_code(int8_t level)
{
    const uint16_t codes[] = {25, 100, 200, 400, 600};
    return valid_level(level) ? codes[level] : 0;
}

inline int8_t power_level(uint16_t code)
{
    for (int8_t i = 0; i < POWER_COUNT; i++) {
        if (power_code(i) == code) {
            return i;
        }
    }
    return -1;
}

inline const char *power_label(int8_t level)
{
    const char *labels[] = {"25mW", "200mW", "500mW", "HIG", "MAX"};
    return valid_level(level) ? labels[level] : "?";
}

inline int8_t switch_level(int8_t position, int8_t maximum)
{
    if (position < 0 || position > 5 || !valid_level(maximum)) {
        return -1;
    }
    // Same nearest-level rounding as the six-position power switch.
    const int8_t level = ((maximum + 1) * (position + 1) + 3) / 6 - 1;
    return level < 0 ? 0 : level;
}
}
