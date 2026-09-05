#ifndef BATTERY_CELLS_H
#define BATTERY_CELLS_H
#include <stdint.h>

/* BATTERY_STATUS may contain pack voltage in slot 0, including overflow
 * chunks of 65534 mV. Only multiple contiguous cell measurements identify
 * a cell count unambiguously. A configured count also supports pack-only data.
 */
static inline double battery_cell_average(const uint16_t *cells, const uint16_t *ext,
                                          unsigned configured)
{
    uint32_t sum = 0;
    unsigned count = 0;
    int ended = 0, individual = 1;
    for (unsigned i = 0; i < 14; ++i) {
        uint16_t mv = i < 10 ? cells[i] : ext[i - 10];
        int absent = i < 10 ? mv == UINT16_MAX : mv == 0;
        if (absent) { ended = 1; continue; }
        if (ended || mv == 0 || mv == UINT16_MAX) return 0;
        if (mv > 6000) individual = 0;
        sum += mv;
        ++count;
    }
    if (configured > 0) return sum > 0 ? sum / (1000.0 * configured) : 0;
    return individual && count >= 2 ? sum / (1000.0 * count) : 0;
}
#endif
