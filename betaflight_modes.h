#ifndef BETAFLIGHT_MODES_H
#define BETAFLIGHT_MODES_H
#include <stdint.h>
/* Compatibility with the local Betaflight MAV_AUTOPILOT_GENERIC custom_mode
 * enumeration. GENERIC alone does not uniquely identify firmware. */
static inline const char *betaflight_mode_name(uint32_t mode)
{
    switch (mode) {
    case 0: return "ACRO";
    case 1: return "ANGLE";
    case 2: return "HORIZON";
    case 3: return "ALT HOLD";
    case 4: return "POS HOLD";
    case 5: return "AUTOPILOT";
    case 6: return "RTL";
    case 7: return "FAILSAFE";
    default: return "UNKNOWN";
    }
}
#endif
