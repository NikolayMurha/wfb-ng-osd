#ifndef GST_DVR_TIMELINE_H
#define GST_DVR_TIMELINE_H

#include <gst/gst.h>
#include <string.h>

/* Retimestamp only the recorder input, then let a force-live video aggregator
 * repeat its last raw frame. OSD is composed after the repeater (or on a
 * separate live mixer pad), so it keeps updating during a network outage.
 * Network segments, flushes and EOS must not reset the local recording clock.
 * Call attach before PLAYING; end is called by the bounded EOS finalizer. */
typedef struct {
    gint refs;
    gint seen_frame;
    gint stopping;
    gint valid_output_caps;
    gboolean sent_segment;
    GstClockTime last_pts;
} DvrTimeline;

static inline void dvr_timeline_unref(gpointer data)
{
    DvrTimeline *timeline = (DvrTimeline *)data;
    if (g_atomic_int_dec_and_test(&timeline->refs))
        g_free(timeline);
}

static GstPadProbeReturn dvr_timeline_input(GstPad *pad, GstPadProbeInfo *info, gpointer data)
{
    DvrTimeline *timeline = (DvrTimeline *)data;
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) {
        GstElement *element = gst_pad_get_parent_element(pad);
        GstClock *clock = gst_element_get_clock(element);
        const GstClockTime base = gst_element_get_base_time(element);
        GstClockTime now = clock != NULL ? gst_clock_get_time(clock) : GST_CLOCK_TIME_NONE;
        if (clock != NULL)
            gst_object_unref(clock);
        gst_object_unref(element);
        if (!GST_CLOCK_TIME_IS_VALID(now) || now < base)
            return GST_PAD_PROBE_DROP;
        now -= base;
        if (g_atomic_int_get(&timeline->seen_frame) && now <= timeline->last_pts)
            now = timeline->last_pts + 1;
        GstBuffer *buffer = gst_buffer_make_writable(GST_PAD_PROBE_INFO_BUFFER(info));
        GST_BUFFER_PTS(buffer) = now;
        GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DURATION(buffer) = GST_SECOND / 30;
        GST_BUFFER_FLAG_UNSET(buffer, GST_BUFFER_FLAG_DISCONT);
        GST_PAD_PROBE_INFO_DATA(info) = buffer;
        timeline->last_pts = now;
        g_atomic_int_set(&timeline->seen_frame, 1);
    } else {
        GstEvent *event = GST_PAD_PROBE_INFO_EVENT(info);
        switch (GST_EVENT_TYPE(event)) {
        case GST_EVENT_SEGMENT: {
            GstSegment segment;
            gst_segment_init(&segment, GST_FORMAT_TIME);
            GstEvent *replacement = gst_event_new_segment(&segment);
            gst_event_set_seqnum(replacement, gst_event_get_seqnum(event));
            gst_event_unref(event);
            GST_PAD_PROBE_INFO_DATA(info) = replacement;
            break;
        }
        case GST_EVENT_FLUSH_START:
        case GST_EVENT_FLUSH_STOP:
        case GST_EVENT_GAP:
            return GST_PAD_PROBE_DROP;
        case GST_EVENT_EOS:
            if (!g_atomic_int_get(&timeline->stopping))
                return GST_PAD_PROBE_DROP;
            break;
        default:
            break;
        }
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn dvr_timeline_output(GstPad *pad, GstPadProbeInfo *info, gpointer data)
{
    (void)pad;
    DvrTimeline *timeline = (DvrTimeline *)data;
    if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM) {
        GstEvent *event = GST_PAD_PROBE_INFO_EVENT(info);
        if (GST_EVENT_TYPE(event) == GST_EVENT_CAPS) {
            GstCaps *caps;
            gint width = 0, height = 0;
            gst_event_parse_caps(event, &caps);
            const GstStructure *structure = gst_caps_get_structure(caps, 0);
            gst_structure_get_int(structure, "width", &width);
            gst_structure_get_int(structure, "height", &height);
            const gboolean valid = width > 1 && height > 1;
            g_atomic_int_set(&timeline->valid_output_caps, valid);
            if (!valid)
                return GST_PAD_PROBE_DROP;
        }
        if (GST_EVENT_TYPE(event) == GST_EVENT_SEGMENT) {
            if (!g_atomic_int_get(&timeline->valid_output_caps))
                return GST_PAD_PROBE_DROP;
            timeline->sent_segment = TRUE;
        }
        return GST_PAD_PROBE_OK;
    }
    if (!g_atomic_int_get(&timeline->seen_frame) || !g_atomic_int_get(&timeline->valid_output_caps))
        return GST_PAD_PROBE_DROP;
    if (!timeline->sent_segment) {
        GstSegment segment;
        gst_segment_init(&segment, GST_FORMAT_TIME);
        timeline->sent_segment = TRUE;
        gst_pad_push_event(pad, gst_event_new_segment(&segment));
    }
    return GST_PAD_PROBE_OK;
}

static inline gboolean dvr_timeline_attach(GstElement *pipeline)
{
    GstElement *input = gst_bin_get_by_name(GST_BIN(pipeline), "dvr_input");
    GstElement *output = gst_bin_get_by_name(GST_BIN(pipeline), "dvr_output");
    if (input == NULL || output == NULL) {
        if (input != NULL) gst_object_unref(input);
        if (output != NULL) gst_object_unref(output);
        return FALSE;
    }
    DvrTimeline *timeline = g_new0(DvrTimeline, 1);
    timeline->refs = 3; /* input qdata + the two pad probes */
    g_object_set_data_full(G_OBJECT(input), "dvr-timeline", timeline, dvr_timeline_unref);
    GstPad *pad = gst_element_get_static_pad(input, "src");
    gst_pad_add_probe(pad, (GstPadProbeType)(GST_PAD_PROBE_TYPE_BUFFER |
        GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM | GST_PAD_PROBE_TYPE_EVENT_FLUSH),
        dvr_timeline_input, timeline, dvr_timeline_unref);
    gst_object_unref(pad);
    pad = gst_element_get_static_pad(output, "src");
    gst_pad_add_probe(pad, (GstPadProbeType)(GST_PAD_PROBE_TYPE_BUFFER |
        GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM), dvr_timeline_output,
                      timeline, dvr_timeline_unref);
    gst_object_unref(pad);
    gst_object_unref(input);
    gst_object_unref(output);
    return TRUE;
}

static inline void dvr_timeline_end(GstElement *pipeline)
{
    GstElement *input = gst_bin_get_by_name(GST_BIN(pipeline), "dvr_input");
    if (input != NULL) {
        DvrTimeline *timeline = (DvrTimeline *)g_object_get_data(G_OBJECT(input), "dvr-timeline");
        if (timeline != NULL)
            g_atomic_int_set(&timeline->stopping, 1);
        gst_object_unref(input);
    }
}

/* Restart a failed network source without resetting the recorder's clock. */
static void dvr_restart_source(GstElement *source, gpointer unused)
{
    (void)unused;
    gst_element_set_state(source, GST_STATE_NULL);
    g_usleep(250000);
    GstElement *parent = GST_ELEMENT(gst_element_get_parent(source));
    if (parent != NULL) {
        GstState state;
        gst_element_get_state(parent, &state, NULL, 0);
        if (state >= GST_STATE_PAUSED)
            gst_element_sync_state_with_parent(source);
        gst_object_unref(parent);
    }
    g_object_set_data(G_OBJECT(source), "dvr-reconnecting", NULL);
}

static inline gboolean dvr_timeline_recover_source(GstElement *pipeline, GstMessage *message)
{
    GstElement *input = gst_bin_get_by_name(GST_BIN(pipeline), "dvr_input");
    if (input == NULL)
        return FALSE;
    DvrTimeline *timeline = (DvrTimeline *)g_object_get_data(G_OBJECT(input), "dvr-timeline");
    gboolean active = timeline != NULL && !g_atomic_int_get(&timeline->stopping);
    gst_object_unref(input);
    if (!active || !GST_IS_ELEMENT(GST_MESSAGE_SRC(message)))
        return FALSE;
    GstElement *element = GST_ELEMENT(gst_object_ref(GST_MESSAGE_SRC(message)));
    GstElement *source = NULL;
    while (element != NULL && element != pipeline) {
        GstElementFactory *factory = gst_element_get_factory(element);
        const gchar *klass = factory != NULL
            ? gst_element_factory_get_metadata(factory, GST_ELEMENT_METADATA_KLASS) : NULL;
        if (klass != NULL && strstr(klass, "Source") != NULL && strstr(klass, "Network") != NULL) {
            if (source != NULL) gst_object_unref(source);
            source = GST_ELEMENT(gst_object_ref(element));
        }
        GstElement *parent = GST_ELEMENT(gst_element_get_parent(element));
        gst_object_unref(element);
        element = parent;
    }
    if (element != NULL) gst_object_unref(element);
    if (source == NULL)
        return FALSE;
    if (g_object_get_data(G_OBJECT(source), "dvr-reconnecting") == NULL) {
        g_object_set_data(G_OBJECT(source), "dvr-reconnecting", GINT_TO_POINTER(1));
        gst_element_call_async(source, dvr_restart_source, NULL, NULL);
    }
    gst_object_unref(source);
    return TRUE;
}

#endif
