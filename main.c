/*
  Copyright (C) 2017, 2021 Vasily Evseenko <svpcom@p2ptech.org>
  based on PlayuavOSD https://github.com/TobiasBales/PlayuavOSD.git
*/

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

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "osdrender.h"
#include "osdmavlink.h"
#include "osdvar.h"
#include "osdconfig.h"
#include "UAVObj.h"
#include "graphengine.h"


#ifdef __GST_OPENGL__
int gst_main(int rtp_port, char *codec, int rtp_jitter, int udp_mpegts, osd_render_t osd_render, int screen_width, char *input_url, char *video_sink, char *dvr_path, int dvr_segment_secs, int dvr_bitrate, char *dvr_preset, int plain_player, char *rtp_forward_host, int rtp_forward_port);
#ifdef __APPLE__
#include <gst/gstmacos.h>
#endif

typedef struct {
    int rtp_port;
    char *codec;
    int rtp_jitter;
    int udp_mpegts;
    osd_render_t osd_render;
    int screen_width;
    char *input_url;
    char *video_sink;
    char *dvr_path;
    int dvr_segment_secs;
    int dvr_bitrate;
    char *dvr_preset;
    int plain_player;
    char *rtp_forward_host;
    int rtp_forward_port;
} gst_thread_args_t;

static void *gst_thread_start(void *arg)
{
    gst_thread_args_t *args = (gst_thread_args_t *)arg;
    gst_main(args->rtp_port, args->codec, args->rtp_jitter, args->udp_mpegts, args->osd_render, args->screen_width, args->input_url, args->video_sink, args->dvr_path, args->dvr_segment_secs, args->dvr_bitrate, args->dvr_preset, args->plain_player, args->rtp_forward_host, args->rtp_forward_port);
    fprintf(stderr, "gst thread exited\n");
    exit(1);
}
#endif

static volatile uint8_t finished = 0;
int osd_debug = 0;
int dvr_recording = 0;

void sigterm_handler(int signum)
{
    finished = 1;
}

static int open_udp_socket_for_rx(int port)
{
    struct sockaddr_in saddr;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("Error opening socket");
        exit(1);
    }

    int optval = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(int));

    bzero((char *)&saddr, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons((unsigned short)port);

    if (bind(fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        perror("Bind error");
        exit(1);
    }
    return fd;
}

static char *trim_ws(char *s)
{
    if (s == NULL)
        return NULL;

    while (*s != '\0' && isspace((unsigned char)*s))
        s++;

    if (*s == '\0')
        return s;

    char *end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
    return s;
}

static void disable_group_gps(void)
{
    osd_params.GpsStatus_en = 0;
    osd_params.GpsHDOP_en = 0;
    osd_params.GpsLat_en = 0;
    osd_params.GpsLon_en = 0;
    osd_params.Gps2Status_en = 0;
    osd_params.Gps2HDOP_en = 0;
    osd_params.Gps2Lat_en = 0;
    osd_params.Gps2Lon_en = 0;

    osd_params.HomeDirection_enabled = 0;
    osd_params.HomeLatitude_enabled = 0;
    osd_params.HomeLongitude_enabled = 0;

    osd_params.CWH_home_dist_en = 0;
    osd_params.CWH_wp_dist_en = 0;
    osd_params.CWH_Tmode_en = 0;
    osd_params.CWH_Nmode_en = 0;

    osd_params.Map_en = 0;
    osd_params.Alarm_GPS_status_en = 0;
}

static void disable_group_wfb(void)
{
    osd_params.WFBState_en = 0;
    osd_params.Alarm_wfb_status_en = 0;
}

static int disable_single_item(const char *name)
{
    if (strcasecmp(name, "gps_status") == 0 || strcasecmp(name, "GpsStatus_en") == 0) {
        osd_params.GpsStatus_en = 0;
        osd_params.Alarm_GPS_status_en = 0;
        return 1;
    }
    if (strcasecmp(name, "gps_coords") == 0 || strcasecmp(name, "gps_latlon") == 0) {
        osd_params.GpsLat_en = 0;
        osd_params.GpsLon_en = 0;
        return 1;
    }
    if (strcasecmp(name, "gps2") == 0) {
        osd_params.Gps2Status_en = 0;
        osd_params.Gps2HDOP_en = 0;
        osd_params.Gps2Lat_en = 0;
        osd_params.Gps2Lon_en = 0;
        return 1;
    }
    if (strcasecmp(name, "home") == 0) {
        osd_params.HomeDirection_enabled = 0;
        osd_params.HomeLatitude_enabled = 0;
        osd_params.HomeLongitude_enabled = 0;
        osd_params.CWH_home_dist_en = 0;
        return 1;
    }
    if (strcasecmp(name, "cwh") == 0) {
        osd_params.CWH_home_dist_en = 0;
        osd_params.CWH_wp_dist_en = 0;
        osd_params.CWH_Tmode_en = 0;
        osd_params.CWH_Nmode_en = 0;
        return 1;
    }
    if (strcasecmp(name, "wfb_state") == 0 || strcasecmp(name, "WFBState_en") == 0) {
        osd_params.WFBState_en = 0;
        return 1;
    }
    if (strcasecmp(name, "wfb_alarm") == 0 || strcasecmp(name, "Alarm_wfb_status_en") == 0) {
        osd_params.Alarm_wfb_status_en = 0;
        return 1;
    }
    if (strcasecmp(name, "warnings") == 0) {
        osd_params.Alarm_GPS_status_en = 0;
        osd_params.Alarm_low_batt_en = 0;
        osd_params.Alarm_low_speed_en = 0;
        osd_params.Alarm_over_speed_en = 0;
        osd_params.Alarm_low_alt_en = 0;
        osd_params.Alarm_over_alt_en = 0;
        osd_params.Alarm_rc_status_en = 0;
        osd_params.Alarm_wfb_status_en = 0;
        return 1;
    }
    return 0;
}

static int disable_item(const char *name)
{
    if (strcasecmp(name, "gps") == 0) {
        disable_group_gps();
        return 1;
    }
    if (strcasecmp(name, "wfb") == 0 || strcasecmp(name, "wfb_link") == 0) {
        disable_group_wfb();
        return 1;
    }

    return disable_single_item(name);
}

static void apply_disable_list(const char *disable_list)
{
    if (disable_list == NULL || *disable_list == '\0')
        return;

    char *copy = strdup(disable_list);
    if (copy == NULL) {
        perror("strdup");
        exit(1);
    }

    char *rest = copy;
    while (rest != NULL) {
        char *token = strsep(&rest, ",;");
        token = trim_ws(token);
        if (token == NULL || *token == '\0')
            continue;

        if (!disable_item(token)) {
            fprintf(stderr, "Unknown -D item: %s\n", token);
        } else {
            fprintf(stderr, "OSD disabled: %s\n", token);
        }
    }

    free(copy);
}

#ifdef __GST_OPENGL__
static void parse_rtp_forward(const char *value, char **host, int *port)
{
    char *sep;
    char *end = NULL;
    long parsed_port;

    if (value == NULL || value[0] == '\0') {
        fprintf(stderr, "Invalid RTP forward target: empty value\n");
        exit(1);
    }

    sep = strrchr(value, ':');
    if (sep == NULL || sep == value || sep[1] == '\0') {
        fprintf(stderr, "Invalid RTP forward target: %s (expected host:port)\n", value);
        exit(1);
    }

    for (const char *p = value; p < sep; p++) {
        if (!isalnum((unsigned char)*p) && *p != '.' && *p != '-' && *p != '_') {
            fprintf(stderr, "Invalid RTP forward host: %s\n", value);
            exit(1);
        }
    }

    errno = 0;
    parsed_port = strtol(sep + 1, &end, 10);
    if (errno != 0 || end == sep + 1 || *end != '\0' || parsed_port <= 0 || parsed_port > 65535) {
        fprintf(stderr, "Invalid RTP forward port: %s\n", sep + 1);
        exit(1);
    }

    free(*host);
    *host = strndup(value, sep - value);
    if (*host == NULL) {
        perror("strndup");
        exit(1);
    }
    *port = (int)parsed_port;
}

static int is_safe_x264_preset(const char *preset)
{
    const char *allowed[] = {
        "ultrafast", "superfast", "veryfast", "faster", "fast",
        "medium", "slow", "slower", "veryslow", "placebo",
    };

    if (preset == NULL || preset[0] == '\0')
        return 0;

    for (size_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++) {
        if (strcmp(preset, allowed[i]) == 0)
            return 1;
    }

    return 0;
}
#endif

static int osd_main(int argc, char **argv)
{
    int opt;
    int osd_port = 14551;
    int rtp_port = 5600;
    char *codec = "h264";
    int rtp_jitter = 0;
    int udp_mpegts = 0;
    int heartbeat_tx_enabled = 0;
    int heartbeat_tx_interval_ms = 1000;
#ifdef __APPLE__
    osd_render_t osd_render = OSD_RENDER_AUTO;
#else
    osd_render_t osd_render = OSD_RENDER_GL;
#endif
    int screen_width = 1920;
    char *input_url = NULL;
    char *video_sink = NULL;
    char *dvr_path = NULL;
    int dvr_segment_secs = 60;
    int dvr_bitrate = 12000;
    char *dvr_preset = "veryfast";
    int plain_player = 0;
    char *rtp_forward_host = NULL;
    int rtp_forward_port = 0;

    uint8_t buf[65536];
    int fd;
#ifndef __GST_OPENGL__
    uint64_t render_ts = 0;
    uint64_t cur_ts = 0;
    struct pollfd fds[1];
#endif
    telemetry_watchdog_t telemetry_watchdog;

    telemetry_watchdog_init(&telemetry_watchdog);

    while ((opt = getopt(argc, argv, "hdp:P:R:D:F:45j:THi:xakgs:w:O:S:B:E:n")) != -1) {
        switch (opt) {
        case 'p': osd_port    = atoi(optarg); break;
        case 'P': rtp_port    = atoi(optarg); break;
        case 'R': input_url   = strdup(optarg); break;
        case 'D': apply_disable_list(optarg); break;
#ifdef __GST_OPENGL__
        case 'F': parse_rtp_forward(optarg, &rtp_forward_host, &rtp_forward_port); break;
#endif
        case '4': codec       = "h264"; break;
        case '5': codec       = "h265"; break;
        case 'j': rtp_jitter  = atoi(optarg); break;
        case 'T': udp_mpegts  = 1; break;
        case 'H': heartbeat_tx_enabled = 1; break;
        case 'i':
            heartbeat_tx_interval_ms = atoi(optarg);
            if (heartbeat_tx_interval_ms <= 0) {
                fprintf(stderr, "Invalid heartbeat interval: %s\n", optarg);
                goto show_usage;
            }
            break;
        case 'x': osd_render  = OSD_RENDER_XV; break;
        case 'a': osd_render  = OSD_RENDER_AUTO; break;
        case 'k': osd_render  = OSD_RENDER_KMS; break;
        case 'g': osd_render  = OSD_RENDER_GTK; break;
        case 's': video_sink  = strdup(optarg); break;
        case 'w': screen_width = atoi(optarg); break;
        case 'n': plain_player = 1; break;
        case 'O': dvr_path    = strdup(optarg); break;
        case 'S':
            dvr_segment_secs = atoi(optarg);
            if (dvr_segment_secs <= 0) {
                fprintf(stderr, "Невірна тривалість сегменту DVR: %s\n", optarg);
                goto show_usage;
            }
            break;
        case 'B':
            dvr_bitrate = atoi(optarg);
            if (dvr_bitrate <= 0) {
                fprintf(stderr, "Invalid DVR bitrate: %s\n", optarg);
                goto show_usage;
            }
            break;
        case 'E':
#ifdef __GST_OPENGL__
            if (!is_safe_x264_preset(optarg)) {
                fprintf(stderr, "Invalid DVR x264 preset: %s\n", optarg);
                goto show_usage;
            }
            dvr_preset = strdup(optarg);
#endif
            break;
        case 'd': osd_debug   = 1; break;
        case 'h':
        default:
        show_usage:
#ifdef __GST_OPENGL__
            fprintf(stderr, "%s [-p mavlink_port] [-P video_port] [-T] [-R input_url] [-D disable_items] [-F host:port] [-4] [-5] [-j jitter_ms] [-H] [-i heartbeat_ms] [-x] [-a] [-k] [-g] [-s video_sink] [-w screen_width] [-O dvr_path] [-S dvr_segment_secs] [-B dvr_kbit] [-E x264_preset] [-n]\n", argv[0]);
            fprintf(stderr, "  -T  receive raw MPEG-TS over UDP on -P port (default is RTP)\n");
            fprintf(stderr, "  -n  plain player mode (no OSD overlay)\n");
            fprintf(stderr, "  -F  forward source video as UDP RTP to host:port; generic URI input is re-encoded\n");
            fprintf(stderr, "Default: mavlink_port=%d, rtp_port=%d, input_url=%s, codec=%s, rtp_jitter=%d, heartbeat_tx=%d, heartbeat_ms=%d, screen_width=%d\n",
                    osd_port, rtp_port,
                    input_url != NULL ? input_url : "none",
                    codec, rtp_jitter, heartbeat_tx_enabled, heartbeat_tx_interval_ms, screen_width);
            fprintf(stderr, "DVR: -O /path/segment_%%05d.ts   (шлях до файлів запису; %%05d — номер сегменту)\n");
            fprintf(stderr, "     -S <seconds>               (тривалість сегменту, default=%d)\n", dvr_segment_secs);
            fprintf(stderr, "     -B <kbit/s>                (H264 bitrate, default=%d)\n", dvr_bitrate);
            fprintf(stderr, "     -E <preset>                (x264 preset, default=%s)\n", dvr_preset);
#else
            fprintf(stderr, "%s [-p mavlink_port] [-D disable_items] [-H] [-i heartbeat_ms]\n", argv[0]);
            fprintf(stderr, "Default: mavlink_port=%d, heartbeat_tx=%d, heartbeat_ms=%d\n",
                    osd_port, heartbeat_tx_enabled, heartbeat_tx_interval_ms);
#endif
            fprintf(stderr, "Disable groups/items examples: -D gps,wfb  |  -D wfb_state  |  -D gps_status,gps_coords\n");
            fprintf(stderr, "WFB-ng OSD version " WFB_OSD_VERSION "\n");
            fprintf(stderr, "WFB-ng home page: <http://wfb-ng.org>\n");
            exit(1);
        }
    }

    if (optind > argc)
        goto show_usage;

    telemetry_watchdog_set_heartbeat_tx(&telemetry_watchdog, heartbeat_tx_enabled, (uint32_t)heartbeat_tx_interval_ms);
    dvr_recording = (dvr_path != NULL);

#ifdef __GST_OPENGL__
    printf("Use: mavlink_port=%d, video_port=%d, udp_mpegts=%d, input_url=%s, codec=%s, jitter=%d, heartbeat_tx=%d, heartbeat_ms=%d, osd_render=%d, video_sink=%s, screen_width=%d, dvr_path=%s, dvr_segment_secs=%d, dvr_bitrate=%d, dvr_preset=%s, plain_player=%d, rtp_forward=%s\n",
           osd_port, rtp_port,
           udp_mpegts,
           input_url != NULL ? input_url : "none",
           codec, rtp_jitter, heartbeat_tx_enabled, heartbeat_tx_interval_ms, osd_render,
           video_sink != NULL ? video_sink : "none", screen_width,
           dvr_path != NULL ? dvr_path : "none", dvr_segment_secs, dvr_bitrate, dvr_preset, plain_player,
           rtp_forward_host != NULL ? rtp_forward_host : "none");

    osd_init(0, 0, 1, 1);
    fd = open_udp_socket_for_rx(osd_port);

    gst_thread_args_t gst_args = {
        .rtp_port        = rtp_port,
        .codec           = codec,
        .rtp_jitter      = rtp_jitter,
        .udp_mpegts      = udp_mpegts,
        .osd_render      = osd_render,
        .screen_width    = screen_width,
        .input_url       = input_url,
        .video_sink      = video_sink,
        .dvr_path        = dvr_path,
        .dvr_segment_secs = dvr_segment_secs,
        .dvr_bitrate     = dvr_bitrate,
        .dvr_preset      = dvr_preset,
        .plain_player    = plain_player,
        .rtp_forward_host = rtp_forward_host,
        .rtp_forward_port = rtp_forward_port,
    };

    pthread_t tid;
    pthread_create(&tid, NULL, gst_thread_start, &gst_args);

    while (1) {
        ssize_t rsize;
        struct sockaddr_in src_addr;
        socklen_t src_len = sizeof(src_addr);
        while ((rsize = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&src_addr, &src_len)) >= 0) {
            mavlink_rx_meta_t meta = {0};
            pthread_mutex_lock(&video_mutex);
            parse_mavlink_packet(buf, rsize, &meta);
            pthread_mutex_unlock(&video_mutex);
            telemetry_watchdog_update(fd, &src_addr, &meta, &telemetry_watchdog);
            src_len = sizeof(src_addr);
        }

        if (rsize < 0 && errno != EINTR) {
            perror("Error receiving packet");
            exit(1);
        }
    }

#else
    printf("Use mavlink_port=%d, heartbeat_tx=%d, heartbeat_ms=%d\n",
           osd_port, heartbeat_tx_enabled, heartbeat_tx_interval_ms);

    osd_init(0, 0, 1, 1);
    fd = open_udp_socket_for_rx(osd_port);

    if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
        perror("Unable to set socket into nonblocked mode");
        exit(1);
    }

    memset(fds, '\0', sizeof(fds));
    fds[0].fd = fd;
    fds[0].events = POLLIN;

    signal(SIGTERM, sigterm_handler);
    signal(SIGINT, sigterm_handler);

    fprintf(stderr, "Starting event loop\n");
    while (!finished) {
        cur_ts = GetSystimeMS();
        uint64_t sleep_ts = render_ts > cur_ts ? render_ts - cur_ts : 0;
        int rc = poll(fds, 1, sleep_ts);

        if (rc < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            perror("Poll error");
            exit(1);
        }

        if (fds[0].revents & (POLLERR | POLLNVAL)) {
            fprintf(stderr, "socket error!");
            exit(1);
        }

        if (fds[0].revents & POLLIN) {
            ssize_t rsize;
            struct sockaddr_in src_addr;
            socklen_t src_len = sizeof(src_addr);
            while ((rsize = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&src_addr, &src_len)) >= 0) {
                mavlink_rx_meta_t meta = {0};
                parse_mavlink_packet(buf, rsize, &meta);
                telemetry_watchdog_update(fd, &src_addr, &meta, &telemetry_watchdog);
                src_len = sizeof(src_addr);
            }
            if (rsize < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
                perror("Error receiving packet");
                exit(1);
            }
        }

        cur_ts = GetSystimeMS();
        if (render_ts <= cur_ts) {
            render_ts = cur_ts + 1000 / 30;
            render();
        }
    }
    fprintf(stderr, "Event loop finished\n");
#endif
    return 0;
}

#if defined(__APPLE__) && defined(__GST_OPENGL__)
static int osd_main_macos(int argc, char **argv, gpointer user_data)
{
    (void)user_data;
    return osd_main(argc, argv);
}
#endif

int main(int argc, char **argv)
{
#if defined(__APPLE__) && defined(__GST_OPENGL__)
    return gst_macos_main(osd_main_macos, argc, argv, NULL);
#else
    return osd_main(argc, argv);
#endif
}
