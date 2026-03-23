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
#include <gst/video/video.h>
#include <gst/video/gstvideometa.h>
#include <gst/video/video-overlay-composition.h>

#include <stdint.h>
#include <pthread.h>
#include <stdio.h>
#include <glib.h>

#include "graphengine.h"

// For gstreamer < 1.18
GstClockTime gst_element_get_current_running_time (GstElement * element);

typedef struct {
    int screen_width;
    int screen_height;
} osd_probe_data_t;


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
 * Pad probe — викликається для кожного відео-кадру.
 * Рендерить OSD у RGBA-буфер та прикріплює його як
 * GstVideoOverlayCompositionMeta. gloverlaycompositor
 * накладає overlay на GPU без жодної синхронізації потоків.
 */
static GstPadProbeReturn
osd_meta_probe (GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    osd_probe_data_t *data = (osd_probe_data_t *)user_data;

    /* Забезпечуємо записуваність буфера для додавання метаданих */
    GstBuffer *buf = gst_buffer_make_writable(GST_PAD_PROBE_INFO_BUFFER(info));
    GST_PAD_PROBE_INFO_DATA(info) = buf;

    /* Рендеримо OSD → RGBA GstBuffer 640x360 */
    pthread_mutex_lock(&video_mutex);
    GstBuffer *osd_buf = render();
    pthread_mutex_unlock(&video_mutex);

    /*
     * gst_video_overlay_rectangle_new_raw вимагає формат BGRA
     * (GST_VIDEO_OVERLAY_COMPOSITION_FORMAT_RGB).
     * graphengine пише пікселі як RGBA, тому переставляємо R↔B.
     */
    {
        GstMapInfo m;
        gst_buffer_map(osd_buf, &m, GST_MAP_READWRITE);
        uint32_t *px = (uint32_t *)m.data;
        int n = GRAPHICS_WIDTH * GRAPHICS_HEIGHT;
        for (int i = 0; i < n; i++) {
            uint32_t p = px[i];
            /* RGBA LE: 0xAABBGGRR → swap R↔B → BGRA LE: 0xAARRGGBB */
            px[i] = (p & 0xFF00FF00u)
                  | ((p & 0x000000FFu) << 16)
                  | ((p & 0x00FF0000u) >> 16);
        }
        gst_buffer_unmap(osd_buf, &m);
    }

    /* Прикріплюємо відео-мета щоб gloverlaycompositor знав формат */
    gst_buffer_add_video_meta(osd_buf,
                              GST_VIDEO_FRAME_FLAG_NONE,
                              GST_VIDEO_FORMAT_BGRA,
                              GRAPHICS_WIDTH, GRAPHICS_HEIGHT);

    /* Беремо реальні розміри відео-кадру з caps паду */
    guint render_w = (guint)data->screen_width;
    guint render_h = (guint)data->screen_height;
    {
        GstCaps *caps = gst_pad_get_current_caps(pad);
        if (caps) {
            GstVideoInfo vinfo;
            if (gst_video_info_from_caps(&vinfo, caps)) {
                render_w = (guint)GST_VIDEO_INFO_WIDTH(&vinfo);
                render_h = (guint)GST_VIDEO_INFO_HEIGHT(&vinfo);
            }
            gst_caps_unref(caps);
        }
    }

    /* Прямокутник overlay масштабується на розмір відео-кадру */
    GstVideoOverlayRectangle *rect = gst_video_overlay_rectangle_new_raw(
        osd_buf,
        0, 0,
        render_w, render_h,
        GST_VIDEO_OVERLAY_FORMAT_FLAG_NONE);
    gst_buffer_unref(osd_buf);

    GstVideoOverlayComposition *comp = gst_video_overlay_composition_new(rect);
    gst_video_overlay_rectangle_unref(rect);

    gst_buffer_add_video_overlay_composition_meta(buf, comp);
    gst_video_overlay_composition_unref(comp);

    return GST_PAD_PROBE_OK;
}


static const char* select_osd_render(osd_render_t osd_render)
{
    switch(osd_render)
    {
    case OSD_RENDER_XV:
        return osd_debug ? \
            "glcolorconvert ! gldownload ! clockoverlay text=XV valignment=center ! xvimagesink" : \
            "glcolorconvert ! gldownload ! xvimagesink";

    case OSD_RENDER_GL:
        return osd_debug ? \
            "clockoverlay text=GL valignment=center ! glimagesink" : \
            "glimagesink";

    case OSD_RENDER_KMS:
        return osd_debug ? \
            "glcolorconvert ! gldownload ! clockoverlay text=Auto valignment=center ! kmssink" : \
            "glcolorconvert ! gldownload ! kmssink";

    case OSD_RENDER_AUTO:
    default:
        return osd_debug ? \
            "glcolorconvert ! gldownload ! clockoverlay text=Auto valignment=center ! autovideosink" : \
            "glcolorconvert ! gldownload ! autovideosink";
    }
}


int gst_main(int rtp_port, char *codec, int rtp_jitter, osd_render_t osd_render, int screen_width, char *input_url)
{
    int screen_height = screen_width * 9 / 16;

    // EGL is Linux-centric; on macOS let GStreamer select the native GL backend.
#ifdef __linux__
    setenv("GST_GL_PLATFORM", "egl", 0);
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
         * Нова архітектура без mixer:
         *   відео → queue → identity(osd_probe) → glupload → glcolorconvert
         *         → gloverlaycompositor → sink
         *
         * Pad probe на identity прикріплює GstVideoOverlayCompositionMeta
         * до кожного відео-кадру. gloverlaycompositor накладає OSD на GPU
         * без синхронізації двох потоків — затримка як у чистого gst.
         */
        if (is_srt_source)
        {
            asprintf(&pipeline_str,
                     "%s ! "
                     "tsdemux ! "
                     "%sparse config-interval=1 disable-passthrough=true ! "
                     "%s qos=false ! "
                     "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "identity name=osd_probe ! "
                     "glupload ! glcolorconvert ! "
                     "gloverlaycompositor ! "
                     "%s sync=false",
                     src_str, codec, decoder,
                     select_osd_render(osd_render));
        }
        else if (is_generic_uri_source)
        {
            asprintf(&pipeline_str,
                     "%s "
                     "uri_src. ! "
                     "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "identity name=osd_probe ! "
                     "glupload ! glcolorconvert ! "
                     "gloverlaycompositor ! "
                     "%s sync=false",
                     src_str,
                     select_osd_render(osd_render));
        }
        else
        {
            asprintf(&pipeline_str,
                     "%s ! "
                     "rtp%sdepay ! "
                     "%sparse config-interval=1 disable-passthrough=true ! "
                     "%s qos=false ! "
                     "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 ! "
                     "identity name=osd_probe ! "
                     "glupload ! glcolorconvert ! "
                     "gloverlaycompositor ! "
                     "%s sync=false",
                     src_str, codec, codec, decoder,
                     select_osd_render(osd_render));
        }

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

    /* Встановлюємо pad probe на identity елемент */
    {
        GstElement *osd_probe_elem = gst_bin_get_by_name(GST_BIN(pipeline), "osd_probe");
        GstPad *src_pad = gst_element_get_static_pad(osd_probe_elem, "src");

        osd_probe_data_t *probe_data = g_new0(osd_probe_data_t, 1);
        probe_data->screen_width  = screen_width;
        probe_data->screen_height = screen_height;

        gst_pad_add_probe(src_pad,
                          GST_PAD_PROBE_TYPE_BUFFER,
                          osd_meta_probe,
                          probe_data,
                          g_free);

        gst_object_unref(src_pad);
        gst_object_unref(osd_probe_elem);
    }

    // Set message handler
    {
        GstBus *bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
        gst_bus_add_signal_watch (bus);
        g_signal_connect (G_OBJECT (bus), "message", G_CALLBACK (on_message), loop);
        gst_object_unref (GST_OBJECT (bus));
    }

    // main loop
    gst_element_set_state (pipeline, GST_STATE_PLAYING);
    g_main_loop_run (loop);
    gst_element_set_state (pipeline, GST_STATE_NULL);

    gst_object_unref (pipeline);

    return 0;
}
