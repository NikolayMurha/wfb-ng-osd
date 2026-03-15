#ifndef __OSD_MAVLINK_H
#define __OSD_MAVLINK_H

#include <stdint.h>
#include <netinet/in.h>
#include "mavlink/common/mavlink.h"

/* ── MAVLink packet metadata (filled by parse_mavlink_packet) ─────────────── */

typedef struct {
    uint8_t got_heartbeat;
    uint8_t got_non_heartbeat;
    uint8_t heartbeat_system;
    uint8_t heartbeat_component;
    uint8_t non_heartbeat_system;
    uint8_t non_heartbeat_component;
} mavlink_rx_meta_t;

void parse_mavlink_packet(uint8_t *buf, int buflen, mavlink_rx_meta_t *meta);

/* ── Telemetry source watchdog ────────────────────────────────────────────── */

#define MAX_TELEMETRY_SOURCES 32

typedef struct {
    uint8_t used;
    uint8_t sysid;
    uint8_t compid;
    struct sockaddr_in addr;
    uint64_t last_heartbeat_ts;
    uint64_t last_telemetry_ts;
} telemetry_source_t;

typedef struct {
    telemetry_source_t sources[MAX_TELEMETRY_SOURCES];
    int target_idx;
    uint64_t last_request_ts;
    uint8_t heartbeat_tx_enabled;
    uint32_t heartbeat_tx_interval_ms;
    uint64_t last_heartbeat_tx_ts;
} telemetry_watchdog_t;

void telemetry_watchdog_init(telemetry_watchdog_t *wd);
void telemetry_watchdog_set_heartbeat_tx(telemetry_watchdog_t *wd, int enabled, uint32_t interval_ms);
void telemetry_watchdog_update(int fd, const struct sockaddr_in *src_addr,
                               const mavlink_rx_meta_t *meta, telemetry_watchdog_t *wd);

#endif  /* __OSD_MAVLINK_H */
