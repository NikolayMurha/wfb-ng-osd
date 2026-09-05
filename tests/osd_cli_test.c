#include <assert.h>
#include <stdio.h>
#include <gst/gst.h>
#include "osd_cli.h"
#include "osdrender.h"
#include "osdvar.h"
#include "osdconfig.h"
#include "graphengine.h"

int osd_debug = 0;
bool dvr_recording = false;

static unsigned pixels(void) {
    GstBuffer *buffer = displayGraphics();
    GstMapInfo info;
    assert(gst_buffer_map(buffer, &info, GST_MAP_READ));
    unsigned count = 0;
    for (unsigned i = 0; i < info.size / 4; ++i) count += ((uint32_t *)info.data)[i] != 0;
    gst_buffer_unmap(buffer, &info);
    gst_buffer_unref(buffer);
    return count;
}

int main(int argc, char **argv) {
    uint32_t value;
    assert(osd_parse_uint("0x000FFFFF", true, OSD_KNOWN_MASK, &value) && value == OSD_KNOWN_MASK);
    assert(osd_parse_uint("7679", true, OSD_KNOWN_MASK, &value) && value == 0x1dff);
    assert(osd_parse_uint("0", true, OSD_KNOWN_MASK, &value) && value == 0);
    const char *invalid[] = {"", "0x", "-1", "+1", " 1", "1 ", "1.0", "0x100000", "4294967296", "999999999999999999999999"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(*invalid); ++i)
        assert(!osd_parse_uint(invalid[i], true, OSD_KNOWN_MASK, &value));
    assert(osd_parse_color("#1234Ab", &value) && value == 0xffab3412);
    assert(!osd_parse_color("#12345G", &value));
    assert(!osd_parse_color("red", &value));

    gst_init(&argc, &argv);
    osd_init(0, 0, 1, 1);
    osd_cli.has_mask = true;
    osd_cli.mask = 0;
    clearGraphics(); RenderScreen(); assert(pixels() == 0);
    ping_count = 1; ping_history[0] = 12; ping_received_ms = ping_monotonic_ms();
    osd_got_home = 1; osd_home_lat = 50; osd_home_lon = 30;
    osd_vbat_A = 16; osd_curr_A = 900; osd_battery_remaining_A = 75;
    const osd_params_t baseline = osd_params;
    for (unsigned bit = 0; bit < 18; ++bit) {
        osd_params = baseline;
        osd_cli.mask = 1u << bit;
        clearGraphics(); RenderScreen();
        unsigned count = pixels();
        /* Warnings depend on a current event; every other bit must draw independently. */
        if (bit != OSD_WARNINGS) { if (!count) fprintf(stderr, "Empty bit %u\n", bit); assert(count > 0); }
    }
    osd_cli.mask = 1u << OSD_ATTITUDE;
    clearGraphics(); RenderScreen(); unsigned attitude_without_values = pixels();
    osd_cli.mask |= 1u << OSD_ATTITUDE_VALUES;
    clearGraphics(); RenderScreen(); unsigned attitude_with_values = pixels();
    assert(attitude_with_values > attitude_without_values);
    osd_cli.mask |= 1u << OSD_COMPACT_RETICLE;
    clearGraphics(); RenderScreen(); assert(pixels() == attitude_with_values);
    unsigned counts[3];
    for (unsigned i = 0; i < 3; ++i) {
        osd_cli.font_percent = 50 + i * 50;
        clearGraphics();
        write_string("TEST 123", 320, 180, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, 0);
        counts[i] = pixels();
    }
    assert(counts[0] < counts[1] && counts[1] < counts[2]);
    /* Hidden battery rows must collapse, with the final row pinned at the bottom. */
    osd_cli.font_percent = 100;
    for (unsigned mask = 0; mask < 16; ++mask) {
        osd_cli.mask = mask << OSD_BATTERY_PERCENT;
        clearGraphics(); draw_battery_readout();
        GstBuffer *frame = displayGraphics(); GstMapInfo map;
        assert(gst_buffer_map(frame, &map, GST_MAP_READ));
        unsigned bands = 0; int last = -1; bool previous = false;
        for (int y = 0; y < GRAPHICS_HEIGHT; ++y) {
            bool row = false;
            for (int x = 0; x < GRAPHICS_WIDTH; ++x)
                row |= ((uint32_t *)map.data)[y * GRAPHICS_WIDTH + x] != 0;
            if (row && !previous) {
                ++bands;
                if (last >= 0) assert(y - last <= 5);
            }
            if (row) last = y;
            previous = row;
        }
        assert(bands == (unsigned)__builtin_popcount(mask));
        if (mask) assert(last >= GRAPHICS_BOTTOM - 13 && last <= GRAPHICS_BOTTOM - 10);
        if (((mask >> 1) & 1) == ((mask >> 2) & 1)) {
            /* The legacy voltage toggle controls both voltage rows. */
            osd_cli.has_mask = false;
            osd_params.BattRemaining_en = mask & 1;
            osd_params.BattVolt_en = (mask >> 1) & 1;
            osd_params.BattCurrent_en = (mask >> 3) & 1;
            osd_params.BattRemaining_panel = osd_params.BattVolt_panel = osd_params.BattCurrent_panel = 1;
            current_panel = 1;
            clearGraphics(); draw_battery_readout();
            GstBuffer *legacy = displayGraphics(); GstMapInfo legacy_map;
            assert(gst_buffer_map(legacy, &legacy_map, GST_MAP_READ));
            assert(map.size == legacy_map.size && memcmp(map.data, legacy_map.data, map.size) == 0);
            gst_buffer_unmap(legacy, &legacy_map); gst_buffer_unref(legacy);
            osd_cli.has_mask = true;
        }
        gst_buffer_unmap(frame, &map); gst_buffer_unref(frame);
    }
    osd_cli.rgba = 0xffab3412;
    clearGraphics();
    write_pixel_lm(10, 10, 1, 1); write_pixel_lm(11, 10, 1, 2);
    GstBuffer *buffer = displayGraphics(); GstMapInfo info;
    assert(gst_buffer_map(buffer, &info, GST_MAP_READ));
    assert(((uint32_t *)info.data)[10 * GRAPHICS_WIDTH + 10] == 0xffab3412);
    assert(((uint32_t *)info.data)[10 * GRAPHICS_WIDTH + 11] == 0xff0000ff);
    gst_buffer_unmap(buffer, &info); gst_buffer_unref(buffer);
    puts("OSD CLI and rendering tests passed");
}
