#include "gst_dvr_finalize.h"
#include <gst/check/gsttestclock.h>
#include <glib/gstdio.h>

typedef struct {
    gint frames;
    gint osd_frames;
    GstClockTime last_pts;
    gint white_frames;
} Observed;

static GstPadProbeReturn observe(GstPad *pad, GstPadProbeInfo *info, gpointer data)
{
    (void)pad;
    Observed *seen = (Observed *)data;
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (seen->frames > 0)
        g_assert_cmpuint(GST_BUFFER_PTS(buffer), >, seen->last_pts);
    seen->last_pts = GST_BUFFER_PTS(buffer);
    GstMapInfo map;
    g_assert_true(gst_buffer_map(buffer, &map, GST_MAP_READ));
    if (map.data[0] > 200)
        g_atomic_int_inc(&seen->white_frames);
    gst_buffer_unmap(buffer, &map);
    g_atomic_int_inc(&seen->frames);
    return GST_PAD_PROBE_OK;
}

static void draw_osd(GstElement *overlay, gpointer cairo, guint64 timestamp,
                     guint64 duration, gpointer data)
{
    (void)overlay; (void)cairo; (void)timestamp; (void)duration;
    g_atomic_int_inc(&((Observed *)data)->osd_frames);
}

static void push_frame(GstElement *source, guint8 color, GstClockTime remote_pts)
{
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, 64 * 48 * 4, NULL);
    gst_buffer_memset(buffer, 0, color, gst_buffer_get_size(buffer));
    GST_BUFFER_PTS(buffer) = remote_pts;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / 30;
    GstFlowReturn flow;
    g_signal_emit_by_name(source, "push-buffer", buffer, &flow);
    gst_buffer_unref(buffer);
    g_assert_cmpint(flow, ==, GST_FLOW_OK);
}

static void advance_until(GstTestClock *clock, Observed *seen, gint target)
{
    const gint64 deadline = g_get_monotonic_time() + 60 * G_USEC_PER_SEC;
    while (g_atomic_int_get(&seen->frames) < target && g_get_monotonic_time() < deadline) {
        GstClockID id = NULL;
        if (gst_test_clock_peek_next_pending_id(clock, &id)) {
            GstClockTime now = gst_clock_get_time(GST_CLOCK(clock));
            GstClockTime next = gst_clock_id_get_time(id);
            if (next > now)
                gst_test_clock_set_time(clock, next);
            gst_test_clock_process_id(clock, id);
        } else {
            g_usleep(100);
        }
    }
    g_assert_cmpint(g_atomic_int_get(&seen->frames), >=, target);
}

int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    GError *error = NULL;
    gchar *directory = g_dir_make_tmp("dvr-continuous-XXXXXX", &error);
    g_assert_no_error(error);
    gchar *description = g_strdup_printf(
        "appsrc name=source is-live=true format=time "
        "caps=\"video/x-raw,format=BGRA,width=64,height=48,framerate=30/1\" "
        "! identity name=dvr_input ! compositor name=dvr_clock force-live=true "
        "ignore-inactive-pads=true sink_0::repeat-after-eos=true "
        "! video/x-raw,format=BGRA,framerate=30/1 ! identity name=dvr_output "
        "! cairooverlay name=osd ! videoconvert ! video/x-raw,format=I420 "
        "! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=60 "
        "! h264parse ! splitmuxsink name=dvr_sink muxer=mpegtsmux "
        "location=\"%s/segment_%%05d.ts\" max-size-time=60000000000", directory);
    GstElement *pipeline = gst_parse_launch(description, &error);
    g_assert_no_error(error);
    g_free(description);
    g_assert_true(dvr_timeline_attach(pipeline));
    Observed seen = {0, 0, 0, 0};
    GstElement *output = gst_bin_get_by_name(GST_BIN(pipeline), "dvr_output");
    GstPad *pad = gst_element_get_static_pad(output, "src");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, observe, &seen, NULL);
    gst_object_unref(pad);
    gst_object_unref(output);
    GstElement *osd = gst_bin_get_by_name(GST_BIN(pipeline), "osd");
    g_signal_connect(osd, "draw", G_CALLBACK(draw_osd), &seen);
    gst_object_unref(osd);
    GstClock *clock = gst_test_clock_new_with_start_time(GST_SECOND);
    gst_pipeline_use_clock(GST_PIPELINE(pipeline), clock);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    GstElement *source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
    push_frame(source, 32, 900 * GST_SECOND);
    advance_until(GST_TEST_CLOCK(clock), &seen, 30);
    /* Eight minutes of silence, accelerated by a test clock. */
    advance_until(GST_TEST_CLOCK(clock), &seen, 30 + 8 * 60 * 30);
    g_assert_cmpint(g_atomic_int_get(&seen.white_frames), ==, 0);
    g_assert_cmpint(g_atomic_int_get(&seen.osd_frames), >=, 8 * 60 * 30);
    g_assert_cmpuint(seen.last_pts, >=, 480 * GST_SECOND);
    /* Input EOS must not stop recording; a reconnected source gets a new segment. */
    GstFlowReturn flow;
    g_signal_emit_by_name(source, "end-of-stream", &flow);
    advance_until(GST_TEST_CLOCK(clock), &seen, 30 + 8 * 60 * 30 + 15);
    gst_element_set_state(source, GST_STATE_NULL);
    g_assert_true(gst_element_sync_state_with_parent(source));
    /* Sender restarts from zero instead of continuing its old timebase. */
    push_frame(source, 240, 0);
    advance_until(GST_TEST_CLOCK(clock), &seen, 30 + 8 * 60 * 30 + 30);
    g_assert_cmpint(g_atomic_int_get(&seen.white_frames), >, 0);
    g_assert_true(dvr_finalize(pipeline, 2 * GST_SECOND, NULL, NULL));
    gst_element_set_state(pipeline, GST_STATE_NULL);
    g_print("frames=%d, OSD updates=%d, resumed frames=%d, final PTS=%.3fs; files=%s\n",
            seen.frames, seen.osd_frames, seen.white_frames,
            (double)seen.last_pts / GST_SECOND, directory);
    gst_object_unref(source);
    gst_object_unref(clock);
    gst_object_unref(pipeline);
    GDir *files = g_dir_open(directory, 0, NULL);
    const gchar *name;
    while ((name = g_dir_read_name(files)) != NULL) {
        gchar *path = g_build_filename(directory, name, NULL);
        g_remove(path);
        g_free(path);
    }
    g_dir_close(files);
    g_rmdir(directory);
    g_free(directory);
    return 0;
}
