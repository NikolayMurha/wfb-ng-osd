#ifndef OSD_CLI_H
#define OSD_CLI_H
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/* Stable CLI ABI shared with Snipe Player; never reorder these bits. */
enum osd_bit {
    OSD_PING, OSD_COMPASS, OSD_HEADING, OSD_FLIGHT_MODE, OSD_WARNINGS,
    OSD_ALTITUDE, OSD_ATTITUDE, OSD_THROTTLE, OSD_FLIGHT_TIME,
    OSD_BATTERY_PERCENT, OSD_BATTERY_VOLTAGE, OSD_CELL_VOLTAGE,
    OSD_BATTERY_CURRENT, OSD_GPS, OSD_COORDINATES, OSD_HOME_DISTANCE,
    OSD_SPEED, OSD_TRIP, OSD_ATTITUDE_VALUES, OSD_COMPACT_RETICLE
};
#define OSD_KNOWN_MASK UINT32_C(0x000fffff)
#define OSD_GPS_MASK UINT32_C(0x0003e000)
typedef struct {
    bool has_mask;
    bool hide_attitude_values;
    uint32_t mask;
    uint32_t rgba;
    unsigned font_percent;
} osd_cli_t;
extern osd_cli_t osd_cli;

static inline bool osd_bit_enabled(enum osd_bit bit) {
    return (osd_cli.mask & (UINT32_C(1) << bit)) != 0;
}

static inline bool osd_parse_uint(const char *s, bool hex, uint32_t max, uint32_t *out) {
    if (!s || !*s) return false;
    int base = 10;
    if (hex && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) { s += 2; base = 16; }
    if (!*s) return false;
    for (const char *p = s; *p; ++p) {
        if ((*p >= '0' && *p <= '9') ||
            (base == 16 && ((*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F')))) continue;
        return false;
    }
    errno = 0;
    unsigned long value = strtoul(s, NULL, base);
    if (errno || value > max) return false;
    *out = (uint32_t)value;
    return true;
}

static inline bool osd_parse_color(const char *s, uint32_t *rgba) {
    if (!s || strlen(s) != 7 || s[0] != '#') return false;
    char hex[9] = "0x";
    memcpy(hex + 2, s + 1, 7);
    uint32_t rgb;
    if (!osd_parse_uint(hex, true, 0xffffff, &rgb)) return false;
    *rgba = 0xff000000u | ((rgb & 255) << 16) | (rgb & 0xff00) | (rgb >> 16);
    return true;
}
#endif
