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
            strdup("clockoverlay text=glimagesink valignment=center ! glimagesink") : \
            strdup("glimagesink");
    }

#ifdef __GST_GTK__
    if (g_strcmp0(video_sink, "gtksink") == 0) {
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=gtksink valignment=center ! gtksink name=gtk_sink") : \
            strdup("glcolorconvert ! gldownload ! gtksink name=gtk_sink");
    }
#endif

    char *pipeline = NULL;
    if (osd_debug) {
        asprintf(&pipeline,
                 "glcolorconvert ! gldownload ! clockoverlay text=%s valignment=center ! %s",
                 video_sink, video_sink);
    } else {
        asprintf(&pipeline,
                 "glcolorconvert ! gldownload ! %s",
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
            strdup("glcolorconvert ! gldownload ! clockoverlay text=XV valignment=center ! xvimagesink") : \
            strdup("glcolorconvert ! gldownload ! xvimagesink");

    case OSD_RENDER_GL:
        return osd_debug ? \
            strdup("clockoverlay text=GL valignment=center ! glimagesink") : \
            strdup("glimagesink");

    case OSD_RENDER_KMS:
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=KMS valignment=center ! kmssink") : \
            strdup("glcolorconvert ! gldownload ! kmssink");

    case OSD_RENDER_AUTO:
    default:
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=Auto valignment=center ! autovideosink") : \
            strdup("glcolorconvert ! gldownload ! autovideosink");

#ifdef __GST_GTK__
    case OSD_RENDER_GTK:
        return osd_debug ? \
            strdup("glcolorconvert ! gldownload ! clockoverlay text=GTK valignment=center ! gtksink name=gtk_sink") : \
            strdup("glcolorconvert ! gldownload ! gtksink name=gtk_sink");
#endif
    }
}


int gst_main(int rtp_port, char *codec, int rtp_jitter, osd_render_t osd_render, int screen_width, char *input_url, char *video_sink)
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

    /* setup pipeline */
    {
        char *pipeline_str = NULL;
        char *src_str = NULL;
        GError *error = NULL;
        gboolean is_srt_source = FALSE;
        gboolean is_generic_uri_source = FALSE;
        char *sink_str = select_osd_render(osd_render, video_sink);

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
            "! %s sync=false ";

        const char *osd_branch =
            "appsrc name=osd_src stream-type=0 format=time min-latency=0 ! "
            "video/x-raw,format=RGBA,width=%d,height=%d,framerate=0/1 ! "
            "glupload ! glcolorconvert ! osd_mixer.";

        if (is_srt_source)
        {
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
        else if (is_generic_uri_source)
        {
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
        else
        {
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

        free(sink_str);
        free(src_str);
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
