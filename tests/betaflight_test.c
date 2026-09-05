#include "osdmavlink.h"
#include "osdvar.h"
#include "betaflight_modes.h"
#include <assert.h>
#include <math.h>
#include <string.h>
static uint64_t now = 1000;
uint64_t GetSystimeMS(void) { return now; }
static void receive(mavlink_message_t *msg) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    int n = mavlink_msg_to_send_buffer(bytes, msg);
    mavlink_rx_meta_t meta = {0};
    parse_mavlink_packet(bytes, n, &meta);
}
int main(void) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, 1, &msg, 2, 0, 80, 1, 3);
    receive(&msg);
    assert(autopilot == 0 && custom_mode == 1 && !motor_armed);
    assert(strcmp(betaflight_mode_name(custom_mode), "ANGLE") == 0);
    mavlink_msg_vfr_hud_pack(1, 1, &msg, 0, 0, 0, 0, .93f, 0);
    receive(&msg);
    assert(fabs(osd_rel_alt - .93) < .001);
    mavlink_global_position_int_t pos = {0};
    pos.relative_alt = 12340;
    mavlink_msg_global_position_int_encode(1, 1, &msg, &pos);
    receive(&msg);
    mavlink_msg_vfr_hud_pack(1, 1, &msg, 0, 0, 0, 0, 100, 0);
    receive(&msg);
    assert(fabs(osd_rel_alt - 12.34) < .001);
    now += 2001;
    receive(&msg);
    assert(osd_rel_alt == 100);
    assert(strcmp(betaflight_mode_name(7), "FAILSAFE") == 0);
    assert(strcmp(betaflight_mode_name(100), "UNKNOWN") == 0);
}
