#include "osdmavlink.h"
#include "osdvar.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

uint64_t GetSystimeMS(void) { return ping_monotonic_ms(); }

static void sample(int32_t value, const char *name, int corrupt)
{
    mavlink_named_value_int_t ping = {0};
    strncpy(ping.name, name, sizeof(ping.name));
    ping.value = value;
    mavlink_message_t message;
    // Even an FC component must not turn local ping into FC liveness.
    mavlink_msg_named_value_int_encode(42, MAV_COMP_ID_AUTOPILOT1, &message, &ping);
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    int size = mavlink_msg_to_send_buffer(bytes, &message);
    if (corrupt) bytes[size - 1] ^= 1;
    mavlink_rx_meta_t meta = {0};
    parse_mavlink_packet(bytes, size, &meta);
    if (strcmp(name, "PING") == 0)
        assert(!meta.got_non_heartbeat && !meta.got_heartbeat);
}

int main(void)
{
    assert(ping_count == 0);
    sample(0, "PING", 0);
    sample(41, "PING", 0);
    sample(60000, "PING", 0);
    sample(-1, "PING", 0);
    assert(ping_count == 3);
    assert(ping_history[0] == -1 && ping_history[1] == 60000 && ping_history[2] == 41);
    uint64_t received = ping_received_ms;
    sample(-2, "PING", 0);
    sample(60001, "PING", 0);
    sample(12, "PING_LTE", 0);
    sample(12, "PING", 1);
    assert(ping_received_ms == received && ping_history[0] == -1);
    sample(25, "PING", 0);
    assert(ping_history[0] == 25);
    puts("MAVLink ping parsing, history, validation and watchdog isolation passed");
}
