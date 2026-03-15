/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*
 * With Grateful Acknowledgements to the projects:
 * MinimOSD - arducam-osd Controller(https://code.google.com/p/arducam-osd/)
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "osdmavlink.h"
#include "osdvar.h"
#include "osdconfig.h"
#include "osdrender.h"

/* ── Helpers ──────────────────────────────────────────────────────────────── */

float Rad2Deg(float x)
{
    return x * (180.0F / M_PI);
}

/* ── MAVLink packet parser ────────────────────────────────────────────────── */

void parse_mavlink_packet(uint8_t *buf, int buflen, mavlink_rx_meta_t *meta)
{
    mavlink_status_t status;
    mavlink_message_t msg;
    uint8_t mavtype;

    for (int i = 0; i < buflen; i++)
    {
        uint8_t c = buf[i];
        if (!mavlink_parse_char(0, c, &msg, &status))
            continue;

        if (meta != NULL && msg.msgid != MAVLINK_MSG_ID_HEARTBEAT &&
            (msg.compid == 1 || msg.compid == 50))
        {
            meta->got_non_heartbeat = 1;
            meta->non_heartbeat_system = msg.sysid;
            meta->non_heartbeat_component = msg.compid;
        }

        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            if ((msg.compid != 1) && (msg.compid != 50))
                break;

            mavtype = mavlink_msg_heartbeat_get_type(&msg);
            if (mavtype == MAV_TYPE_GCS)
                break;

            mav_system    = msg.sysid;
            mav_component = msg.compid;
            mav_type      = mavtype;

            if (meta != NULL) {
                meta->got_heartbeat = 1;
                meta->heartbeat_system = msg.sysid;
                meta->heartbeat_component = msg.compid;
            }

            autopilot   = mavlink_msg_heartbeat_get_autopilot(&msg);
            base_mode   = mavlink_msg_heartbeat_get_base_mode(&msg);
            custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);

            last_motor_armed = motor_armed;
            motor_armed = base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

            if (!last_motor_armed && motor_armed)
                armed_start_time = GetSystimeMS();

            if (last_motor_armed && !motor_armed) {
                total_armed_time = GetSystimeMS() - armed_start_time + total_armed_time;
                armed_start_time = 0;
            }
            break;
        }

        case MAVLINK_MSG_ID_HOME_POSITION:
            osd_home_lat = mavlink_msg_home_position_get_latitude(&msg) / 1e7;
            osd_home_lon = mavlink_msg_home_position_get_longitude(&msg) / 1e7;
            osd_home_alt = mavlink_msg_home_position_get_altitude(&msg) / 1000;
            osd_got_home = 1;
            break;

        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            vtol_state = mavlink_msg_extended_sys_state_get_vtol_state(&msg);
            break;

        case MAVLINK_MSG_ID_SYS_STATUS:
            osd_vbat_A               = mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f;
            osd_curr_A               = mavlink_msg_sys_status_get_current_battery(&msg);
            osd_battery_remaining_A  = mavlink_msg_sys_status_get_battery_remaining(&msg);
            break;

        case MAVLINK_MSG_ID_BATTERY_STATUS:
            osd_curr_consumed_mah = mavlink_msg_battery_status_get_current_consumed(&msg);
            break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
            osd_lat                 = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0;
            osd_lon                 = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0;
            osd_fix_type            = mavlink_msg_gps_raw_int_get_fix_type(&msg);
            osd_hdop                = mavlink_msg_gps_raw_int_get_eph(&msg);
            osd_satellites_visible  = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
            break;

        case MAVLINK_MSG_ID_GPS2_RAW:
            osd_lat2                = mavlink_msg_gps2_raw_get_lat(&msg) / 10000000.0;
            osd_lon2                = mavlink_msg_gps2_raw_get_lon(&msg) / 10000000.0;
            osd_fix_type2           = mavlink_msg_gps2_raw_get_fix_type(&msg);
            osd_hdop2               = mavlink_msg_gps2_raw_get_eph(&msg);
            osd_satellites_visible2 = mavlink_msg_gps2_raw_get_satellites_visible(&msg);
            break;

        case MAVLINK_MSG_ID_VFR_HUD:
            osd_airspeed    = mavlink_msg_vfr_hud_get_airspeed(&msg);
            osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
            osd_heading     = mavlink_msg_vfr_hud_get_heading(&msg);
            osd_throttle    = mavlink_msg_vfr_hud_get_throttle(&msg);
            osd_alt         = mavlink_msg_vfr_hud_get_alt(&msg);
            osd_climb       = mavlink_msg_vfr_hud_get_climb(&msg);
            break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            mavlink_global_position_int_t p;
            mavlink_msg_global_position_int_decode(&msg, &p);
            osd_alt     = p.alt / 1000.0;
            osd_rel_alt = p.relative_alt / 1000.0;
            break;
        }

        case MAVLINK_MSG_ID_ALTITUDE:
            osd_bottom_clearance = mavlink_msg_altitude_get_bottom_clearance(&msg);
            osd_rel_alt          = mavlink_msg_altitude_get_altitude_relative(&msg);
            break;

        case MAVLINK_MSG_ID_ATTITUDE:
            osd_pitch = Rad2Deg(mavlink_msg_attitude_get_pitch(&msg));
            osd_roll  = Rad2Deg(mavlink_msg_attitude_get_roll(&msg));
            osd_yaw   = Rad2Deg(mavlink_msg_attitude_get_yaw(&msg));
            break;

        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
            nav_roll            = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
            nav_pitch           = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
            nav_bearing         = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
            wp_target_bearing   = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
            wp_dist             = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
            alt_error           = mavlink_msg_nav_controller_output_get_alt_error(&msg);
            aspd_error          = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
            xtrack_error        = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
            break;

        case MAVLINK_MSG_ID_MISSION_CURRENT:
            wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
            break;

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            if (!osd_chan_cnt_above_eight) {
                osd_chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
                osd_chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
                osd_chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(&msg);
                osd_chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(&msg);
                osd_chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
                osd_chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
                osd_chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
                osd_chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
                osd_rssi     = mavlink_msg_rc_channels_raw_get_rssi(&msg);
            }
            break;

        case MAVLINK_MSG_ID_RC_CHANNELS:
            osd_chan_cnt_above_eight = true;
            osd_chan1_raw  = mavlink_msg_rc_channels_get_chan1_raw(&msg);
            osd_chan2_raw  = mavlink_msg_rc_channels_get_chan2_raw(&msg);
            osd_chan3_raw  = mavlink_msg_rc_channels_get_chan3_raw(&msg);
            osd_chan4_raw  = mavlink_msg_rc_channels_get_chan4_raw(&msg);
            osd_chan5_raw  = mavlink_msg_rc_channels_get_chan5_raw(&msg);
            osd_chan6_raw  = mavlink_msg_rc_channels_get_chan6_raw(&msg);
            osd_chan7_raw  = mavlink_msg_rc_channels_get_chan7_raw(&msg);
            osd_chan8_raw  = mavlink_msg_rc_channels_get_chan8_raw(&msg);
            osd_chan9_raw  = mavlink_msg_rc_channels_get_chan9_raw(&msg);
            osd_chan10_raw = mavlink_msg_rc_channels_get_chan10_raw(&msg);
            osd_chan11_raw = mavlink_msg_rc_channels_get_chan11_raw(&msg);
            osd_chan12_raw = mavlink_msg_rc_channels_get_chan12_raw(&msg);
            osd_chan13_raw = mavlink_msg_rc_channels_get_chan13_raw(&msg);
            osd_chan14_raw = mavlink_msg_rc_channels_get_chan14_raw(&msg);
            osd_chan15_raw = mavlink_msg_rc_channels_get_chan15_raw(&msg);
            osd_chan16_raw = mavlink_msg_rc_channels_get_chan16_raw(&msg);
            osd_rssi      = mavlink_msg_rc_channels_get_rssi(&msg);
            break;

        case MAVLINK_MSG_ID_RADIO_STATUS:
            if ((msg.sysid != 3) || (msg.compid != 68))
                break;
            wfb_rssi      = (int8_t)mavlink_msg_radio_status_get_rssi(&msg);
            wfb_errors    = mavlink_msg_radio_status_get_rxerrors(&msg);
            wfb_fec_fixed = mavlink_msg_radio_status_get_fixed(&msg);
            wfb_flags     = mavlink_msg_radio_status_get_remnoise(&msg);
            break;

        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            osd_message_queue_tail = (osd_message_queue_tail + 1) % OSD_MAX_MESSAGES;
            osd_message_t *item = osd_message_queue + osd_message_queue_tail;
            item->severity = mavlink_msg_statustext_get_severity(&msg);
            mavlink_msg_statustext_get_text(&msg, item->message);
            item->message[sizeof(item->message) - 1] = '\0';
            printf("Message: %s\n", item->message);
            break;
        }

        default:
            break;
        }
    }
}

/* ── Telemetry source watchdog ────────────────────────────────────────────── */

#define TELEMETRY_TIMEOUT_MS    3000
#define REQUEST_RETRY_MS        2000
#define HEARTBEAT_MAX_AGE_MS    5000
#define REQUEST_RATE_HZ         4
#define REQUEST_INTERVAL_US     250000
#define HEARTBEAT_TX_INTERVAL_MS_DEFAULT 1000

void telemetry_watchdog_init(telemetry_watchdog_t *wd)
{
    memset(wd, 0, sizeof(*wd));
    wd->target_idx = -1;
    wd->heartbeat_tx_interval_ms = HEARTBEAT_TX_INTERVAL_MS_DEFAULT;
}

void telemetry_watchdog_set_heartbeat_tx(telemetry_watchdog_t *wd, int enabled, uint32_t interval_ms)
{
    wd->heartbeat_tx_enabled = enabled ? 1 : 0;
    wd->heartbeat_tx_interval_ms = interval_ms > 0 ? interval_ms : HEARTBEAT_TX_INTERVAL_MS_DEFAULT;
    wd->last_heartbeat_tx_ts = 0;
}

static int send_mavlink_message(int fd, const struct sockaddr_in *dst, const mavlink_message_t *msg)
{
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t tx_len = mavlink_msg_to_send_buffer(tx_buf, msg);
    return sendto(fd, tx_buf, tx_len, 0, (const struct sockaddr *)dst, sizeof(*dst));
}

static int telemetry_source_less(const telemetry_source_t *a, const telemetry_source_t *b)
{
    if (a->sysid != b->sysid)
        return a->sysid < b->sysid;
    return a->compid < b->compid;
}

static int telemetry_source_find(const telemetry_watchdog_t *wd, uint8_t sysid, uint8_t compid)
{
    for (int i = 0; i < MAX_TELEMETRY_SOURCES; i++) {
        if (wd->sources[i].used &&
            wd->sources[i].sysid == sysid &&
            wd->sources[i].compid == compid)
            return i;
    }
    return -1;
}

static int telemetry_source_alloc(telemetry_watchdog_t *wd)
{
    int oldest_idx = -1;
    uint64_t oldest_ts = UINT64_MAX;

    for (int i = 0; i < MAX_TELEMETRY_SOURCES; i++) {
        if (!wd->sources[i].used)
            return i;

        if (wd->sources[i].last_heartbeat_ts < oldest_ts) {
            oldest_ts = wd->sources[i].last_heartbeat_ts;
            oldest_idx = i;
        }
    }
    return oldest_idx;
}

static int telemetry_source_upsert(telemetry_watchdog_t *wd, uint8_t sysid, uint8_t compid,
                                   const struct sockaddr_in *addr)
{
    int idx = telemetry_source_find(wd, sysid, compid);
    if (idx < 0) {
        idx = telemetry_source_alloc(wd);
        if (idx < 0)
            return -1;
        memset(&wd->sources[idx], 0, sizeof(wd->sources[idx]));
        wd->sources[idx].used   = 1;
        wd->sources[idx].sysid  = sysid;
        wd->sources[idx].compid = compid;
    }
    wd->sources[idx].addr = *addr;
    return idx;
}

static int telemetry_source_is_active(const telemetry_source_t *src, uint64_t now)
{
    return src->used &&
           src->last_heartbeat_ts != 0 &&
           now - src->last_heartbeat_ts <= HEARTBEAT_MAX_AGE_MS;
}

static int telemetry_watchdog_select_target(const telemetry_watchdog_t *wd, uint64_t now)
{
    int prio_idx = -1;
    int fallback_idx = -1;

    for (int i = 0; i < MAX_TELEMETRY_SOURCES; i++) {
        const telemetry_source_t *src = &wd->sources[i];
        if (!telemetry_source_is_active(src, now))
            continue;

        if (src->sysid >= 1 && src->sysid <= 100) {
            if (prio_idx < 0 || telemetry_source_less(src, &wd->sources[prio_idx]))
                prio_idx = i;
        } else {
            if (fallback_idx < 0 || telemetry_source_less(src, &wd->sources[fallback_idx]))
                fallback_idx = i;
        }
    }

    return prio_idx >= 0 ? prio_idx : fallback_idx;
}

static void request_message_interval(int fd, const telemetry_source_t *src,
                                     uint32_t msg_id, int32_t interval_us)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, MAV_COMP_ID_OSD, &msg,
        src->sysid, src->compid,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        (float)msg_id, (float)interval_us,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    send_mavlink_message(fd, &src->addr, &msg);
}

static void send_telemetry_requests(int fd, const telemetry_source_t *src)
{
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(
        255, MAV_COMP_ID_OSD, &msg,
        src->sysid, src->compid,
        MAV_DATA_STREAM_ALL, REQUEST_RATE_HZ, 1);
    send_mavlink_message(fd, &src->addr, &msg);

    static const uint32_t msg_ids[] = {
        MAVLINK_MSG_ID_SYS_STATUS,
        MAVLINK_MSG_ID_BATTERY_STATUS,
        MAVLINK_MSG_ID_ATTITUDE,
        MAVLINK_MSG_ID_VFR_HUD,
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        MAVLINK_MSG_ID_GPS_RAW_INT,
        MAVLINK_MSG_ID_RC_CHANNELS,
        MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
    };

    for (size_t i = 0; i < sizeof(msg_ids) / sizeof(msg_ids[0]); i++)
        request_message_interval(fd, src, msg_ids[i], REQUEST_INTERVAL_US);
}

static void send_osd_heartbeat(int fd, const telemetry_source_t *dst)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        255, MAV_COMP_ID_OSD, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    send_mavlink_message(fd, &dst->addr, &msg);
}

void telemetry_watchdog_update(int fd, const struct sockaddr_in *src_addr,
                               const mavlink_rx_meta_t *meta, telemetry_watchdog_t *wd)
{
    uint64_t now = GetSystimeMS();
    int old_target_idx = wd->target_idx;

    if (meta->got_heartbeat) {
        int idx = telemetry_source_upsert(wd, meta->heartbeat_system, meta->heartbeat_component, src_addr);
        if (idx >= 0)
            wd->sources[idx].last_heartbeat_ts = now;
    }

    if (meta->got_non_heartbeat) {
        int idx = telemetry_source_find(wd, meta->non_heartbeat_system, meta->non_heartbeat_component);
        if (idx >= 0) {
            wd->sources[idx].addr = *src_addr;
            wd->sources[idx].last_telemetry_ts = now;
        }
    }

    wd->target_idx = telemetry_watchdog_select_target(wd, now);

    if (wd->target_idx != old_target_idx) {
        wd->last_request_ts = 0;
        wd->last_heartbeat_tx_ts = 0;

        if (old_target_idx >= 0 && wd->target_idx >= 0) {
            fprintf(stderr, "MAVLink target switch: sys=%u comp=%u -> sys=%u comp=%u\n",
                    wd->sources[old_target_idx].sysid, wd->sources[old_target_idx].compid,
                    wd->sources[wd->target_idx].sysid, wd->sources[wd->target_idx].compid);
        } else if (wd->target_idx >= 0) {
            fprintf(stderr, "MAVLink target selected: sys=%u comp=%u\n",
                    wd->sources[wd->target_idx].sysid, wd->sources[wd->target_idx].compid);
        } else if (old_target_idx >= 0) {
            fprintf(stderr, "MAVLink target lost: sys=%u comp=%u\n",
                    wd->sources[old_target_idx].sysid, wd->sources[old_target_idx].compid);
        }
    }

    if (wd->target_idx < 0)
        return;

    telemetry_source_t *target = &wd->sources[wd->target_idx];
    if (!telemetry_source_is_active(target, now))
        return;

    if (wd->heartbeat_tx_enabled &&
        (wd->last_heartbeat_tx_ts == 0 ||
         (now - wd->last_heartbeat_tx_ts) >= wd->heartbeat_tx_interval_ms)) {
        send_osd_heartbeat(fd, target);
        wd->last_heartbeat_tx_ts = now;
        fprintf(stderr, "Sending OSD heartbeat to sys=%u comp=%u\n",
                target->sysid, target->compid);
    }

    if (target->last_telemetry_ts != 0 &&
        (now - target->last_telemetry_ts) <= TELEMETRY_TIMEOUT_MS)
        return;

    if (wd->last_request_ts != 0 &&
        (now - wd->last_request_ts) <= REQUEST_RETRY_MS)
        return;

    fprintf(stderr, "No telemetry for %d ms, requesting MAVLink streams from sys=%u comp=%u\n",
            TELEMETRY_TIMEOUT_MS, target->sysid, target->compid);
    send_telemetry_requests(fd, target);
    wd->last_request_ts = now;
}
