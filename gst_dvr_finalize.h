#ifndef GST_DVR_FINALIZE_H
#define GST_DVR_FINALIZE_H

#include <gst/gst.h>

/* A silent network source does not end the last GOP in splitmuxsink. Send
 * EOS before NULL, including after an arbitrarily long input outage. Dispatch
 * it off the UI thread: sending EOS can wait for a streaming/render thread. */
static void dvr_send_eos(GstElement *pipeline, gpointer unused)
{
    (void)unused;
    gst_element_send_event(pipeline, gst_event_new_eos());
}

static gboolean dvr_finalize(GstElement *pipeline, GstClockTime timeout,
                             void (*pump_events)(void *), void *context)
{
    GstElement *recorder = gst_bin_get_by_name(GST_BIN(pipeline), "dvr_sink");
    if (recorder == NULL)
        return TRUE;
    gst_object_unref(recorder);

    GstState state;
    gst_element_get_state(pipeline, &state, NULL, 0);
    if (state < GST_STATE_PAUSED)
        return TRUE;

    GstBus *bus = gst_element_get_bus(pipeline);
    const gint64 deadline = g_get_monotonic_time() + (gint64)(timeout / GST_USECOND);
    gst_element_call_async(pipeline, dvr_send_eos, NULL, NULL);
    gboolean complete = FALSE;
    while (g_get_monotonic_time() < deadline) {
        if (pump_events != NULL)
            pump_events(context);
        const gint64 remaining = deadline - g_get_monotonic_time();
        if (remaining <= 0)
            break;
        GstMessage *message = gst_bus_timed_pop_filtered(
            bus, (GstClockTime)MIN(remaining, 10000) * GST_USECOND,
            (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (message != NULL) {
            complete = GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
            gst_message_unref(message);
            break;
        }
    }
    gst_object_unref(bus);
    return complete;
}

#endif
