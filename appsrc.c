/*
 * Сopyright (C) 2024 Vasily Evseenko <svpcom@p2ptech.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#define _GNU_SOURCE

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#ifdef __GST_GTK__
#include <gtk/gtk.h>
#endif

#include <stdint.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>

#include "graphengine.h"

// For gstreamer < 1.18
GstClockTime gst_element_get_current_running_time (GstElement * element);


static gboolean
on_message (GstBus * bus, GstMessage * message, gpointer user_data)
{
    GMainLoop *loop = (GMainLoop *) user_data;

    switch (GST_MESSAGE_TYPE (message)) {
    case GST_MESSAGE_ERROR:
    {
        GError *err = NULL;
        gchar *debug;

        gst_message_parse_error (message, &err, &debug);
        g_critical ("Got ERROR: %s (%s)", err->message, GST_STR_NULL (debug));
        g_main_loop_quit (loop);
        break;
    }

    case GST_MESSAGE_WARNING:
    {
        GError *err = NULL;
        gchar *debug;

        gst_message_parse_warning (message, &err, &debug);
        g_warning ("Got WARNING: %s (%s)", err->message, GST_STR_NULL (debug));
        g_error_free (err);
        g_free (debug);
        break;
    }

    case GST_MESSAGE_STATE_CHANGED:
    {
        GstState old_state, new_state;

        gst_message_parse_state_changed (message, &old_state, &new_state, NULL);
        g_print ("gst %s: %s -> %s\n",
                 GST_OBJECT_NAME (message->src),
                 gst_element_state_get_name (old_state),
                 gst_element_state_get_name (new_state));
        break;
    }

    case GST_MESSAGE_EOS:
        g_main_loop_quit (loop);
        break;

    default:
        break;
    }

    return TRUE;
}

/*
 * Callback need-data appsrc'а — викликається GStreamer'ом коли OSD-stream
 * потребує наступного кадру. Рендеримо OSD у RGBA-буфер та пушимо в pipeline.
 * Mixer (glvideomixerelement) композитує цей кадр поверх відео.
 *
 * Це upstream-style архітектура (svpcom/wfb-ng-osd). На відміну від
 * gloverlaycompositor + pad probe, мікшер достовірно показує overlay
 * незалежно від GL/CPU sink chain.
 */
static void cb_need_data (GstElement *appsrc, guint unused_size, gpointer user_data)
{
    GMainLoop *loop = (GMainLoop *) user_data;

    pthread_mutex_lock(&video_mutex);
    GstBuffer *buffer = render();
    pthread_mutex_unlock(&video_mutex);

    GstClockTime pts = gst_element_get_current_running_time(appsrc);
    /* Якщо clock ще не готовий — використовуємо 0, але не пропускаємо буфер.
     * appsrc повинен завжди пушити, інакше mixer почне виводити відео без OSD. */
    GST_BUFFER_PTS (buffer) = GST_CLOCK_TIME_IS_VALID(pts) ? pts : 0;

    /* Min supported fps; низьке значення збільшує latency.
     * Якщо fps нижче ніж selected — CPU usage сильно зростає. */
    GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 30);
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_DROPPABLE);

    GstFlowReturn ret;
    g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref (buffer);

    if (ret != GST_FLOW_OK) {
        g_main_loop_quit (loop);
    }
}


static gboolean is_safe_sink_name(const char *video_sink)
{
    if (video_sink == NULL || video_sink[0] == '\0')
        return FALSE;

    for (const char *p = video_sink; *p != '\0'; p++) {
        if (!g_ascii_isalnum(*p) && *p != '_' && *p != '-')
            return FALSE;
    }

    return TRUE;
}

static gboolean is_safe_dvr_path(const char *path)
{
    if (path == NULL || path[0] == '\0')
        return FALSE;

    for (const char *p = path; *p != '\0'; p++) {
        if (!g_ascii_isalnum(*p) &&
            *p != '/' && *p != '.' && *p != '-' && *p != '_' && *p != '%')
            return FALSE;
    }

    return TRUE;
}

static char* build_dvr_sink_chain(osd_render_t osd_render, const char *video_sink,
                                   const char *dvr_path, int dvr_segment_secs,
                                   int dvr_bitrate, const char *dvr_preset)
{
    /* Визначаємо display sink (CPU-side, без gldownload — він вже є у спільній частині) */
    const char *display_part;
    char display_buf[256];

    if (video_sink != NULL && video_sink[0] != '\0') {
        if (g_strcmp0(video_sink, "glimagesink") == 0) {
            /* glimagesink потребує GL-буфер; після gldownload потрібен glupload.
             * Простіше переключитися на autovideosink для DVR-режиму. */
            fprintf(stderr, "DVR: glimagesink не підтримується з DVR, використовуємо autovideosink\n");
            display_part = "autovideosink sync=false";
        } else if (g_strcmp0(video_sink, "gtksink") == 0) {
            display_part = "gtksink name=gtk_sink sync=false";
        } else {
            snprintf(display_buf, sizeof(display_buf), "%s sync=false", video_sink);
            display_part = display_buf;
        }
    } else {
        switch (osd_render) {
        case OSD_RENDER_XV:  display_part = "xvimagesink sync=false";  break;
        case OSD_RENDER_GL:  display_part = "autovideosink sync=false"; break;
        case OSD_RENDER_KMS: display_part = "kmssink sync=false";       break;
#ifdef __GST_GTK__
        case OSD_RENDER_GTK: display_part = "gtksink name=gtk_sink sync=false"; break;
#endif
        default:             display_part = "autovideosink sync=false"; break;
        }
    }

    guint64 max_time_ns = (guint64)dvr_segment_secs * GST_SECOND;
    char *result = NULL;

    const char *debug_overlay = osd_debug ? "clockoverlay text=DVR valignment=center ! " : "";

    asprintf(&result,
             "glcolorconvert ! gldownload ! "
             "tee name=dvr_tee "
             "dvr_tee. ! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! "
             "%s%s "
             "dvr_tee. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! "
             "videoconvert ! "
             "x264enc tune=zerolatency speed-preset=%s bitrate=%d key-int-max=60 ! "
             "h264parse ! "
             "splitmuxsink name=dvr_sink location=\"%s\" max-size-time=%" G_GUINT64_FORMAT " muxer=mpegtsmux",
             debug_overlay, display_part,
             dvr_preset, dvr_bitrate, dvr_path, max_time_ns);

    return result;
}

/*
 * Будує "sink chain" — частину pipeline після glvideomixer.
 * Mixer виводить GL-буфери, тож для CPU-sinks потрібен glcolorconvert ! gldownload.
 * glimagesink приймає GL напряму.
 */
static char* build_sink_pipeline_for_name(const char *video_sink)
{
    if (!is_safe_sink_name(video_sink)) {
        fprintf(stderr, "Invalid video sink name: %s\n", video_sink != NULL ? video_sink : "(null)");
        exit(1);
    }

    if (g_strcmp0(video_sink, "glimagesink") == 0) {
        return osd_debug ? \
            strdup("clockoverlay text=glimagesink valignment=center ! glimagesink sync=false") : \
            strdup("glimagesink sync=false");
    }

#ifdef __GST_GTK__
    if (g_strcmp0(video_sink, "gtksink") == 0) {
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=gtksink valignment=center ! gtksink name=gtk_sink sync=false") : \
            strdup("glcolorconvert ! gldownload ! gtksink name=gtk_sink sync=false");
    }
#endif

    char *pipeline = NULL;
    if (osd_debug) {
        asprintf(&pipeline,
                 "glcolorconvert ! gldownload ! clockoverlay text=%s valignment=center ! %s sync=false",
                 video_sink, video_sink);
    } else {
        asprintf(&pipeline,
                 "glcolorconvert ! gldownload ! %s sync=false",
                 video_sink);
    }
    return pipeline;
}

static char* select_osd_render(osd_render_t osd_render, const char *video_sink)
{
    if (video_sink != NULL && video_sink[0] != '\0')
        return build_sink_pipeline_for_name(video_sink);

    switch(osd_render)
    {
    case OSD_RENDER_XV:
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=XV valignment=center ! xvimagesink sync=false") : \
            strdup("glcolorconvert ! gldownload ! xvimagesink sync=false");

    case OSD_RENDER_GL:
        return osd_debug ? \
            strdup("clockoverlay text=GL valignment=center ! glimagesink sync=false") : \
            strdup("glimagesink sync=false");

    case OSD_RENDER_KMS:
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=KMS valignment=center ! kmssink sync=false") : \
            strdup("glcolorconvert ! gldownload ! kmssink sync=false");

    case OSD_RENDER_AUTO:
    default:
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=Auto valignment=center ! autovideosink sync=false") : \
            strdup("glcolorconvert ! gldownload ! autovideosink sync=false");

#ifdef __GST_GTK__
    case OSD_RENDER_GTK:
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=GTK valignment=center ! gtksink name=gtk_sink sync=false") : \
            strdup("glcolorconvert ! gldownload ! gtksink name=gtk_sink sync=false");
#endif
    }
}

static char* build_plain_output_chain(const char *plain_sink, const char *dvr_path,
                                      int dvr_segment_secs, int dvr_bitrate,
                                      const char *dvr_preset)
{
    char *result = NULL;

    if (dvr_path == NULL) {
        asprintf(&result, "videoconvert ! %s", plain_sink);
        return result;
    }

    if (!is_safe_dvr_path(dvr_path)) {
        fprintf(stderr, "DVR: недопустимий шлях: %s\n", dvr_path);
        fprintf(stderr, "Дозволені символи: alnum / . - _ %%\n");
        exit(1);
    }

    guint64 max_time_ns = (guint64)dvr_segment_secs * GST_SECOND;

    asprintf(&result,
             "videoconvert ! "
             "tee name=plain_dvr_tee "
             "plain_dvr_tee. ! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! "
             "%s "
             "plain_dvr_tee. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! "
             "x264enc tune=zerolatency speed-preset=%s bitrate=%d key-int-max=60 ! "
             "h264parse ! "
             "splitmuxsink name=dvr_sink location=\"%s\" max-size-time=%" G_GUINT64_FORMAT " muxer=mpegtsmux",
             plain_sink, dvr_preset, dvr_bitrate, dvr_path, max_time_ns);

    return result;
}


static char* build_plain_pipeline(int rtp_port, char *codec, int rtp_jitter,
                                   osd_render_t osd_render, char *input_url, char *video_sink,
                                   char *rtp_forward_host, int rtp_forward_port,
                                   char *dvr_path, int dvr_segment_secs,
                                   int dvr_bitrate, char *dvr_preset)
{
    /* Простий плеєр без OSD: source → decoder → videoconvert → sink */
    char *src_str = NULL;
    char *decoder = NULL;
    char *pipeline_str = NULL;
    char *forward_branch = NULL;
    char *plain_output_chain = NULL;
    gboolean is_srt_source = FALSE;
    gboolean is_generic_uri_source = FALSE;

    /* Вибір sink — CPU-based, glimagesink потребує glupload */
    const char *plain_sink;
    char plain_sink_buf[128];
    if (video_sink != NULL && video_sink[0] != '\0') {
        if (g_strcmp0(video_sink, "glimagesink") == 0) {
            plain_sink = "glupload ! glimagesink sync=false";
        } else if (g_strcmp0(video_sink, "gtksink") == 0) {
            plain_sink = "gtksink name=gtk_sink sync=false";
        } else {
            snprintf(plain_sink_buf, sizeof(plain_sink_buf), "%s sync=false", video_sink);
            plain_sink = plain_sink_buf;
        }
    } else {
        switch (osd_render) {
        case OSD_RENDER_XV:  plain_sink = "xvimagesink sync=false";  break;
        case OSD_RENDER_GL:  plain_sink = "glupload ! glimagesink sync=false"; break;
        case OSD_RENDER_KMS: plain_sink = "kmssink sync=false";      break;
#ifdef __GST_GTK__
        case OSD_RENDER_GTK: plain_sink = "gtksink name=gtk_sink sync=false"; break;
#endif
        default:             plain_sink = "autovideosink sync=false"; break;
        }
    }

    if (input_url != NULL && g_str_has_prefix(input_url, "srt://")) {
        is_srt_source = TRUE;
        asprintf(&src_str, "srtsrc uri=\"%s\" latency=%d", input_url, rtp_jitter);
    } else if (input_url != NULL &&
               (g_str_has_prefix(input_url, "rtsp://") ||
                g_str_has_prefix(input_url, "rtsps://"))) {
        asprintf(&src_str, "rtspsrc latency=%d protocols=tcp location=\"%s\"", rtp_jitter, input_url);
    } else if (input_url != NULL) {
        is_generic_uri_source = TRUE;
        asprintf(&src_str, "uridecodebin uri=\"%s\" name=uri_src", input_url);
    } else {
        asprintf(&src_str,
                 "udpsrc port=%d caps=\"application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H%s\" ! "
                 "rtpjitterbuffer latency=%d",
                 rtp_port, codec + 1, rtp_jitter);
    }

    if (!is_generic_uri_source) {
        char *codecs[]      = {"nv%sdec", "v4l2%sdec", "vaapi%sdec", "avdec_%s"};
        char *codecs_args[] = {NULL, NULL, "low-latency=true", "std-compliance=normal"};
        for (size_t i = 0; i < sizeof(codecs) / sizeof(codecs[0]); i++) {
            char *buf = NULL;
            asprintf(&buf, codecs[i], codec);
            GstElement *tmp = gst_element_factory_make(buf, NULL);
            if (tmp != NULL) {
                gst_object_unref(tmp);
                if (codecs_args[i] != NULL) {
                    asprintf(&decoder, "%s %s", buf, codecs_args[i]);
                    free(buf);
                } else {
                    decoder = buf;
                }
                break;
            }
            free(buf);
        }
        if (decoder == NULL) {
            fprintf(stderr, "No decoder for %s was found\n", codec);
            exit(1);
        }
    }

    if (rtp_forward_host != NULL && rtp_forward_port > 0) {
        if (is_generic_uri_source) {
            const char *encoder = g_strcmp0(codec, "h265") == 0 ?
                "x265enc tune=zerolatency speed-preset=ultrafast" :
                "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30";

            asprintf(&forward_branch,
                     "raw_tee. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! "
                     "videoconvert ! %s ! %sparse config-interval=1 disable-passthrough=true ! "
                     "rtp%spay config-interval=1 pt=96 ! "
                     "udpsink host=\"%s\" port=%d sync=false async=false",
                     encoder, codec, codec, rtp_forward_host, rtp_forward_port);
        } else {
            asprintf(&forward_branch,
                     "encoded_tee. ! queue leaky=downstream max-size-buffers=16 max-size-bytes=0 max-size-time=0 ! "
                     "rtp%spay config-interval=1 pt=96 ! "
                     "udpsink host=\"%s\" port=%d sync=false async=false",
                     codec, rtp_forward_host, rtp_forward_port);
        }
    }

    plain_output_chain = build_plain_output_chain(plain_sink, dvr_path, dvr_segment_secs,
                                                  dvr_bitrate, dvr_preset);

    if (is_srt_source) {
        if (forward_branch != NULL) {
            asprintf(&pipeline_str,
                     "%s ! tsdemux ! %sparse config-interval=1 disable-passthrough=true ! "
                     "tee name=encoded_tee "
                     "encoded_tee. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "%s qos=false ! %s "
                     "%s",
                     src_str, codec, decoder, plain_output_chain, forward_branch);
        } else {
            asprintf(&pipeline_str,
                     "%s ! tsdemux ! %sparse config-interval=1 disable-passthrough=true ! "
                     "%s qos=false ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "%s",
                     src_str, codec, decoder, plain_output_chain);
        }
    } else if (is_generic_uri_source) {
        if (forward_branch != NULL) {
            asprintf(&pipeline_str,
                     "%s uri_src. ! tee name=raw_tee "
                     "raw_tee. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "%s "
                     "%s",
                     src_str, plain_output_chain, forward_branch);
        } else {
            asprintf(&pipeline_str,
                     "%s uri_src. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "%s",
                     src_str, plain_output_chain);
        }
    } else {
        if (forward_branch != NULL) {
            asprintf(&pipeline_str,
                     "%s ! rtp%sdepay ! %sparse config-interval=1 disable-passthrough=true ! "
                     "tee name=encoded_tee "
                     "encoded_tee. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "%s qos=false ! %s "
                     "%s",
                     src_str, codec, codec, decoder, plain_output_chain, forward_branch);
        } else {
            asprintf(&pipeline_str,
                     "%s ! rtp%sdepay ! %sparse config-interval=1 disable-passthrough=true ! "
                     "%s qos=false ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "%s",
                     src_str, codec, codec, decoder, plain_output_chain);
        }
    }

    free(src_str);
    if (decoder) free(decoder);
    if (forward_branch) free(forward_branch);
    if (plain_output_chain) free(plain_output_chain);
    return pipeline_str;
}


int gst_main(int rtp_port, char *codec, int rtp_jitter, osd_render_t osd_render, int screen_width, char *input_url, char *video_sink, char *dvr_path, int dvr_segment_secs, int dvr_bitrate, char *dvr_preset, int plain_player, char *rtp_forward_host, int rtp_forward_port)
{
    int screen_height = screen_width * 9 / 16;

    // EGL is Linux-centric; on macOS let GStreamer select the native GL backend.
#ifdef __linux__
    setenv("GST_GL_PLATFORM", "egl", 0);
#endif

#ifdef __GST_GTK__
    if (osd_render == OSD_RENDER_GTK || g_strcmp0(video_sink, "gtksink") == 0)
        gtk_init(NULL, NULL);
#endif

    /* init GStreamer */
    gst_init (NULL, NULL);

    GMainLoop *loop = g_main_loop_new (NULL, FALSE);
    GstElement *pipeline = NULL;

    if (plain_player) {
        char *pipeline_str = build_plain_pipeline(rtp_port, codec, rtp_jitter,
                                                   osd_render, input_url, video_sink,
                                                   rtp_forward_host, rtp_forward_port,
                                                   dvr_path, dvr_segment_secs,
                                                   dvr_bitrate, dvr_preset);
        printf("GST plain pipeline: %s\n", pipeline_str);
        GError *error = NULL;
        pipeline = gst_parse_launch(pipeline_str, &error);
        free(pipeline_str);
        if (error != NULL) {
            fprintf(stderr, "GST Error: %s\n", error->message);
            g_error_free(error);
            exit(1);
        }
        g_assert(pipeline);
        GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
        gst_bus_add_signal_watch(bus);
        g_signal_connect(G_OBJECT(bus), "message", G_CALLBACK(on_message), loop);
        gst_object_unref(GST_OBJECT(bus));
#ifdef __GST_GTK__
        if (osd_render == OSD_RENDER_GTK || g_strcmp0(video_sink, "gtksink") == 0) {
            GstElement *gtk_sink = gst_bin_get_by_name(GST_BIN(pipeline), "gtk_sink");
            GtkWidget *video_widget;
            g_object_get(gtk_sink, "widget", &video_widget, NULL);
            gst_object_unref(gtk_sink);
            GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
            gtk_window_set_title(GTK_WINDOW(window), "WFB-ng Player");
            gtk_window_set_default_size(GTK_WINDOW(window), screen_width, screen_height);
            g_signal_connect_swapped(window, "destroy", G_CALLBACK(g_main_loop_quit), loop);
            gtk_container_add(GTK_CONTAINER(window), video_widget);
            g_object_unref(video_widget);
            gtk_widget_show_all(window);
        }
#endif
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        g_main_loop_run(loop);
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        return 0;
    }

    /* setup pipeline */
    {
        char *pipeline_str = NULL;
        char *src_str = NULL;
        char *forward_branch = NULL;
        GError *error = NULL;
        gboolean is_srt_source = FALSE;
        gboolean is_generic_uri_source = FALSE;
        char *sink_str = NULL;
        if (dvr_path != NULL) {
            if (!is_safe_dvr_path(dvr_path)) {
                fprintf(stderr, "DVR: недопустимий шлях: %s\n", dvr_path);
                fprintf(stderr, "Дозволені символи: alnum / . - _ %%\n");
                exit(1);
            }
            sink_str = build_dvr_sink_chain(osd_render, video_sink, dvr_path, dvr_segment_secs,
                                            dvr_bitrate, dvr_preset);
        } else {
            sink_str = select_osd_render(osd_render, video_sink);
        }

        if (input_url != NULL && g_str_has_prefix(input_url, "srt://"))
        {
            is_srt_source = TRUE;
            asprintf(&src_str,
                     "srtsrc uri=\"%s\" latency=%d",
                     input_url, rtp_jitter);
        }
        else if (input_url != NULL &&
                 (g_str_has_prefix(input_url, "rtsp://") ||
                  g_str_has_prefix(input_url, "rtsps://")))
        {
            asprintf(&src_str,
                     "rtspsrc latency=%d protocols=tcp location=\"%s\"",
                     rtp_jitter, input_url);
        }
        else if (input_url != NULL)
        {
            is_generic_uri_source = TRUE;
            asprintf(&src_str,
                     "uridecodebin uri=\"%s\" name=uri_src",
                     input_url);
        }
        else
        {
            asprintf(&src_str,
                     "udpsrc port=%d caps=\"application/x-rtp,media=(string)video,  clock-rate=(int)90000, encoding-name=(string)H%s\" ! "
                     "rtpjitterbuffer latency=%d",
                     rtp_port, codec + 1, rtp_jitter);
        }

        char *decoder = NULL;
        if (!is_generic_uri_source)
        {
            char *codecs[] = {"nv%sdec", "v4l2%sdec", "vaapi%sdec", "avdec_%s"};
            char *codecs_args[] = {NULL, NULL, "low-latency=true", "std-compliance=normal"};
            for (size_t i = 0; i < sizeof(codecs) / sizeof(codecs[0]); i++)
            {
                char *buf = NULL;
                asprintf(&buf, codecs[i], codec);
                GstElement *tmp = gst_element_factory_make(buf, "decoder");

                if (tmp != NULL)
                {
                    gst_object_unref(tmp);
                    if (codecs_args[i] != NULL)
                    {
                        asprintf(&decoder, "%s %s", buf, codecs_args[i]);
                        free(buf);
                    }
                    else
                    {
                        decoder = buf;
                    }
                    break;
                }

                free(buf);
            }

            if (decoder == NULL)
            {
                fprintf(stderr, "No decoder for %s was found\n", codec);
                exit(1);
            }
        }

        if (rtp_forward_host != NULL && rtp_forward_port > 0) {
            if (is_generic_uri_source) {
                const char *encoder = g_strcmp0(codec, "h265") == 0 ?
                    "x265enc tune=zerolatency speed-preset=ultrafast" :
                    "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30";

                asprintf(&forward_branch,
                         "raw_tee. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! "
                         "videoconvert ! %s ! %sparse config-interval=1 disable-passthrough=true ! "
                         "rtp%spay config-interval=1 pt=96 ! "
                         "udpsink host=\"%s\" port=%d sync=false async=false",
                         encoder, codec, codec, rtp_forward_host, rtp_forward_port);
            } else {
                asprintf(&forward_branch,
                         "encoded_tee. ! queue leaky=downstream max-size-buffers=16 max-size-bytes=0 max-size-time=0 ! "
                         "rtp%spay config-interval=1 pt=96 ! "
                         "udpsink host=\"%s\" port=%d sync=false async=false",
                         codec, rtp_forward_host, rtp_forward_port);
            }
        }

        /*
         * Upstream-style архітектура з glvideomixer + appsrc:
         *   відео → decoder → queue → glupload → glcolorconvert → mixer.sink_0
         *   appsrc(OSD RGBA) → glupload → glcolorconvert → mixer.sink_1
         *   mixer → sink_chain (glimagesink/gtksink/...)
         *
         * Це надійніше за gloverlaycompositor+pad probe (попередня архітектура у форку),
         * де meta могла губитися при upload'і у GL пам'ять — overlay не показувався.
         *
         * sink_0 (відео) z=-2, sink_1 (OSD) z=0 — OSD поверх відео.
         */
        const char *mixer_chain =
            "glvideomixerelement emit-signals=true start-time-selection=1 name=osd_mixer "
            "sink_0::emit-signals=true sink_0::width=%d sink_0::height=%d sink_0::zorder=-2 "
            "sink_1::emit-signals=true sink_1::width=%d sink_1::height=%d sink_1::zorder=0 "
            "! %s ";

        const char *osd_branch =
            "appsrc name=osd_src stream-type=0 format=time min-latency=0 ! "
            "video/x-raw,format=RGBA,width=%d,height=%d,framerate=0/1 ! "
            "glupload ! glcolorconvert ! osd_mixer.";

        if (is_srt_source)
        {
            if (forward_branch != NULL) {
                asprintf(&pipeline_str,
                         "%s ! "
                         "tsdemux ! "
                         "%sparse config-interval=1 disable-passthrough=true ! "
                         "tee name=encoded_tee "
                         "encoded_tee. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                         "%s qos=false ! "
                         "glupload ! glcolorconvert ! "
                         "%s "
                         "%s "
                         "%s",
                         src_str, codec, decoder,
                         g_strdup_printf(mixer_chain,
                                         screen_width, screen_height,
                                         screen_width, screen_height,
                                         sink_str),
                         g_strdup_printf(osd_branch, GRAPHICS_WIDTH, GRAPHICS_HEIGHT),
                         forward_branch);
            } else {
                asprintf(&pipeline_str,
                         "%s ! "
                         "tsdemux ! "
                         "%sparse config-interval=1 disable-passthrough=true ! "
                         "%s qos=false ! "
                         "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                         "glupload ! glcolorconvert ! "
                         "%s "
                         "%s",
                         src_str, codec, decoder,
                         g_strdup_printf(mixer_chain,
                                         screen_width, screen_height,
                                         screen_width, screen_height,
                                         sink_str),
                         g_strdup_printf(osd_branch, GRAPHICS_WIDTH, GRAPHICS_HEIGHT));
            }
        }
        else if (is_generic_uri_source)
        {
            if (forward_branch != NULL) {
                asprintf(&pipeline_str,
                         "%s "
                         "uri_src. ! tee name=raw_tee "
                         "raw_tee. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                         "glupload ! glcolorconvert ! "
                         "%s "
                         "%s "
                         "%s",
                         src_str,
                         g_strdup_printf(mixer_chain,
                                         screen_width, screen_height,
                                         screen_width, screen_height,
                                         sink_str),
                         g_strdup_printf(osd_branch, GRAPHICS_WIDTH, GRAPHICS_HEIGHT),
                         forward_branch);
            } else {
                asprintf(&pipeline_str,
                         "%s "
                         "uri_src. ! "
                         "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                         "glupload ! glcolorconvert ! "
                         "%s "
                         "%s",
                         src_str,
                         g_strdup_printf(mixer_chain,
                                         screen_width, screen_height,
                                         screen_width, screen_height,
                                         sink_str),
                         g_strdup_printf(osd_branch, GRAPHICS_WIDTH, GRAPHICS_HEIGHT));
            }
        }
        else
        {
            if (forward_branch != NULL) {
                asprintf(&pipeline_str,
                         "%s ! "
                         "rtp%sdepay ! "
                         "%sparse config-interval=1 disable-passthrough=true ! "
                         "tee name=encoded_tee "
                         "encoded_tee. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                         "%s qos=false ! "
                         "glupload ! glcolorconvert ! "
                         "%s "
                         "%s "
                         "%s",
                         src_str, codec, codec, decoder,
                         g_strdup_printf(mixer_chain,
                                         screen_width, screen_height,
                                         screen_width, screen_height,
                                         sink_str),
                         g_strdup_printf(osd_branch, GRAPHICS_WIDTH, GRAPHICS_HEIGHT),
                         forward_branch);
            } else {
                asprintf(&pipeline_str,
                         "%s ! "
                         "rtp%sdepay ! "
                         "%sparse config-interval=1 disable-passthrough=true ! "
                         "%s qos=false ! "
                         "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                         "glupload ! glcolorconvert ! "
                         "%s "
                         "%s",
                         src_str, codec, codec, decoder,
                         g_strdup_printf(mixer_chain,
                                         screen_width, screen_height,
                                         screen_width, screen_height,
                                         sink_str),
                         g_strdup_printf(osd_branch, GRAPHICS_WIDTH, GRAPHICS_HEIGHT));
            }
        }

        free(sink_str);
        free(src_str);
        if (forward_branch != NULL)
            free(forward_branch);
        if (decoder != NULL)
            free(decoder);

        printf("GST pipeline: %s\n", pipeline_str);

        pipeline = gst_parse_launch(pipeline_str, &error);
        free(pipeline_str);

        if(error != NULL)
        {
            fprintf (stderr, "GST Error: %s\n", error->message);
            g_error_free (error);
            exit(1);
        }
    }

    g_assert(pipeline);

    /* Connect appsrc need-data signal — pushes OSD frames into mixer.sink_1 */
    {
        GstElement *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "osd_src");
        g_signal_connect (appsrc, "need-data", G_CALLBACK (cb_need_data), loop);
        gst_object_unref(appsrc);
    }

    // Set message handler
    {
        GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
        gst_bus_add_signal_watch (bus);
        g_signal_connect (G_OBJECT (bus), "message", G_CALLBACK (on_message), loop);
        gst_object_unref (GST_OBJECT (bus));
    }

#ifdef __GST_GTK__
    /*
     * Важливо: GTK window треба налаштувати ПЕРЕД gst_element_set_state(PLAYING).
     * Інакше gtksink при переході в PLAYING створює свій default top-level
     * GtkWindow з title "GTK+ Cairo Renderer", і ми отримуємо ДВА вікна:
     *   - "WFB-ng OSD" (наше, порожнє)
     *   - "GTK+ Cairo Renderer" (gtksink default, з відео)
     * Готуємо widget і вкладаємо в наше вікно до того, як sink стартує.
     */
    if (osd_render == OSD_RENDER_GTK || g_strcmp0(video_sink, "gtksink") == 0) {
        GstElement *gtk_sink = gst_bin_get_by_name(GST_BIN(pipeline), "gtk_sink");
        GtkWidget *video_widget;
        g_object_get(gtk_sink, "widget", &video_widget, NULL);
        gst_object_unref(gtk_sink);

        GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(window), "WFB-ng OSD");
        gtk_window_set_default_size(GTK_WINDOW(window), screen_width, screen_height);
        g_signal_connect_swapped(window, "destroy", G_CALLBACK(g_main_loop_quit), loop);
        gtk_container_add(GTK_CONTAINER(window), video_widget);
        g_object_unref(video_widget);
        gtk_widget_show_all(window);
    }
#endif

    // main loop
    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    g_main_loop_run (loop);
    gst_element_set_state (pipeline, GST_STATE_NULL);

    gst_object_unref (pipeline);

    return 0;
}
