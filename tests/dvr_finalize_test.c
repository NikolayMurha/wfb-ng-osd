#include "gst_dvr_finalize.h"
#include <glib/gstdio.h>

static void count_frame(GstElement *sink, GstBuffer *buffer, GstPad *pad, gpointer data)
{
    (void)sink;
    (void)buffer;
    (void)pad;
    g_atomic_int_inc((gint *)data);
}

static GstElement *parse(const gchar *description)
{
    GError *error = NULL;
    GstElement *pipeline = gst_parse_launch(description, &error);
    g_assert_no_error(error);
    g_assert_nonnull(pipeline);
    return pipeline;
}

static gint count_recorded_frames(const gchar *path)
{
    gchar *description = g_strdup_printf(
        "filesrc location=\"%s\" ! tsdemux ! h264parse ! "
        "video/x-h264,alignment=au ! fakesink name=counter sync=false signal-handoffs=true", path);
    GstElement *pipeline = parse(description);
    g_free(description);
    GstElement *counter = gst_bin_get_by_name(GST_BIN(pipeline), "counter");
    gint frames = 0;
    g_signal_connect(counter, "handoff", G_CALLBACK(count_frame), &frames);
    gst_object_unref(counter);
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    GstMessage *message = gst_bus_timed_pop_filtered(
        bus, 5 * GST_SECOND, (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    g_assert_nonnull(message);
    g_assert_cmpint(GST_MESSAGE_TYPE(message), ==, GST_MESSAGE_EOS);
    gst_message_unref(message);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);
    return g_atomic_int_get(&frames);
}

static void check_stalled_source(gboolean graceful)
{
    GError *error = NULL;
    gchar *directory = g_dir_make_tmp("dvr-tail-XXXXXX", &error);
    g_assert_no_error(error);
    gchar *description = g_strdup_printf(
        "appsrc name=source is-live=true format=time "
        "caps=\"video/x-raw,format=I420,width=320,height=240,framerate=30/1\" ! tee name=t "
        "t. ! queue ! fakesink name=display sync=false signal-handoffs=true "
        "t. ! queue ! x264enc tune=zerolatency speed-preset=veryfast key-int-max=60 "
        "! h264parse ! splitmuxsink name=dvr_sink muxer=mpegtsmux "
        "location=\"%s/segment_%%05d.ts\" max-size-time=60000000000", directory);
    GstElement *pipeline = parse(description);
    g_free(description);
    GstElement *source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
    GstElement *display = gst_bin_get_by_name(GST_BIN(pipeline), "display");
    gint displayed = 0;
    g_signal_connect(display, "handoff", G_CALLBACK(count_frame), &displayed);
    gst_object_unref(display);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    for (gint index = 0; index < 95; ++index) {
        GstBuffer *buffer = gst_buffer_new_allocate(NULL, 320 * 240 * 3 / 2, NULL);
        gst_buffer_memset(buffer, 0, 128, gst_buffer_get_size(buffer));
        GST_BUFFER_PTS(buffer) = (GstClockTime)index * GST_SECOND / 30;
        GST_BUFFER_DURATION(buffer) = GST_SECOND / 30;
        GstFlowReturn flow;
        g_signal_emit_by_name(source, "push-buffer", buffer, &flow);
        gst_buffer_unref(buffer);
        g_assert_cmpint(flow, ==, GST_FLOW_OK);
    }
    gst_object_unref(source);
    gint64 deadline = g_get_monotonic_time() + 5 * G_USEC_PER_SEC;
    while (g_atomic_int_get(&displayed) != 95 && g_get_monotonic_time() < deadline)
        g_usleep(1000);
    g_assert_cmpint(g_atomic_int_get(&displayed), ==, 95);
    /* Sender is silent without EOS; waiting does not release the last GOP. */
    g_usleep(G_USEC_PER_SEC);
    if (graceful)
        g_assert_true(dvr_finalize(pipeline, 2 * GST_SECOND, NULL, NULL));
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gchar *path = g_build_filename(directory, "segment_00000.ts", NULL);
    gint recorded = count_recorded_frames(path);
    g_print("displayed=95, idle=1s, graceful=%d, recorded=%d\n", graceful, recorded);
    g_assert_cmpint(recorded, ==, graceful ? 95 : 60);
    g_remove(path);
    g_rmdir(directory);
    g_free(path);
    g_free(directory);
}

static GstPadProbeReturn drop_eos(GstPad *pad, GstPadProbeInfo *info, gpointer data)
{
    (void)pad;
    (void)data;
    return GST_EVENT_TYPE(GST_PAD_PROBE_INFO_EVENT(info)) == GST_EVENT_EOS
        ? GST_PAD_PROBE_DROP : GST_PAD_PROBE_OK;
}

static void check_timeout(void)
{
    GstElement *pipeline = parse("fakesrc is-live=true ! fakesink name=dvr_sink sync=false");
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "dvr_sink");
    GstPad *pad = gst_element_get_static_pad(sink, "sink");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM, drop_eos, NULL, NULL);
    gst_object_unref(pad);
    gst_object_unref(sink);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    gst_element_get_state(pipeline, NULL, NULL, GST_SECOND);
    gint64 start = g_get_monotonic_time();
    g_assert_false(dvr_finalize(pipeline, 100 * GST_MSECOND, NULL, NULL));
    g_assert_cmpint(g_get_monotonic_time() - start, <, G_USEC_PER_SEC);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    check_stalled_source(FALSE);
    check_stalled_source(TRUE);
    check_timeout();
    return 0;
}
