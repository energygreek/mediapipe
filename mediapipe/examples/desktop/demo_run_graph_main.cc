// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/port/gstreamer_inc.h"

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
constexpr char kWindowName[] = "MediaPipe";

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(int, output_rtp_port, 10018, "output rtp port");

typedef struct {
  GMainLoop *loop;
  GstElement *sink_pipeline;
  GstElement *src_pipeline;
} ProgramData;

/* called when we get a GstMessage from the source pipeline when we get EOS, we
 * notify the appsrc of it. */
static gboolean on_appsink_message(GstBus *bus, GstMessage *message,
                                   ProgramData *data) {
  GstElement *source;

  switch (GST_MESSAGE_TYPE(message)) {
  case GST_MESSAGE_EOS:
    g_print("The source got dry\n");
    source = gst_bin_get_by_name(GST_BIN(data->sink_pipeline), "testsource");
    // gst_app_src_end_of_stream (GST_APP_SRC (source));
    g_signal_emit_by_name(source, "end-of-stream", NULL);
    gst_object_unref(source);
    break;
  case GST_MESSAGE_ERROR:
    g_print("Received error\n");
    g_main_loop_quit(data->loop);
    break;
  default:
    break;
  }
  return TRUE;
}

/* called when we get a GstMessage from the sink pipeline when we get EOS, we
 * exit the mainloop and this testapp. */
static gboolean on_appsrc_message(GstBus *bus, GstMessage *message,
                                  ProgramData *data) {
  /* nil */
  switch (GST_MESSAGE_TYPE(message)) {
  case GST_MESSAGE_EOS:
    g_print("Finished playback\n");
    g_main_loop_quit(data->loop);
    break;
  case GST_MESSAGE_ERROR:
    g_print("Received error\n");
    g_main_loop_quit(data->loop);
    break;
  default:
    break;
  }
  return TRUE;
}

/* Functions below print the Capabilities in a human-friendly format */
static gboolean print_field(GQuark field, const GValue *value, gpointer pfx) {
  gchar *str = gst_value_serialize(value);

  g_print("%s  %15s: %s\n", (gchar *)pfx, g_quark_to_string(field), str);
  g_free(str);
  return TRUE;
}

static void print_caps(const GstCaps *caps, const gchar *pfx) {
  guint i;

  g_return_if_fail(caps != NULL);

  if (gst_caps_is_any(caps)) {
    g_print("%sANY\n", pfx);
    return;
  }
  if (gst_caps_is_empty(caps)) {
    g_print("%sEMPTY\n", pfx);
    return;
  }

  for (i = 0; i < gst_caps_get_size(caps); i++) {
    GstStructure *structure = gst_caps_get_structure(caps, i);

    g_print("%s%s\n", pfx, gst_structure_get_name(structure));
    gst_structure_foreach(structure, print_field, (gpointer)pfx);
  }
}

/* Shows the CURRENT capabilities of the requested pad in the given element */
static void print_pad_capabilities(GstElement *element, gchar *pad_name) {
  GstPad *pad = NULL;
  GstCaps *caps = NULL;

  /* Retrieve pad */
  pad = gst_element_get_static_pad(element, pad_name);
  if (!pad) {
    g_printerr("Could not retrieve pad '%s'\n", pad_name);
    return;
  }

  /* Retrieve negotiated caps (or acceptable caps if negotiation is not finished
   * yet) */
  caps = gst_pad_get_current_caps(pad);
  if (!caps)
    caps = gst_pad_query_caps(pad, NULL);

  /* Print and free */
  g_print("Caps for the %s pad:\n", pad_name);
  print_caps(caps, "      ");
  gst_caps_unref(caps);
  gst_object_unref(pad);
}

int gstreamer_main(int argc, char *argv[], mediapipe::CalculatorGraph &graph,
                   mediapipe::OutputStreamPoller &poller) {
  gchar *filename = NULL;
  ProgramData *data = NULL;
  gchar *string = NULL;
  GstBus *bus = NULL;
  GstElement *testsink = NULL;
  GstElement *testsource = NULL;

  gst_init(&argc, &argv);
  data = g_new0(ProgramData, 1);

  string = g_strdup_printf(
      "udpsrc port=10020 ! "
      "application/"
      "x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)"
      "H264,payload=(int)96 ! "
      "rtph264depay ! avdec_h264 ! videoconvert ! "
      "appsink "
      "caps=\"video/x-raw,format=RGB,width=1280,height=720,framerate=10/1\" "
      "name=testsink sync=false");
  g_free(filename);
  g_print("%s\n", string);
  data->src_pipeline = gst_parse_launch(string, NULL);
  g_free(string);

  if (data->src_pipeline == NULL) {
    g_print("Bad source\n");
    return -1;
  }

  /* to be notified of messages from this pipeline, mostly EOS */
  bus = gst_element_get_bus(data->src_pipeline);
  gst_bus_add_watch(bus, (GstBusFunc)on_appsink_message, data);
  gst_object_unref(bus);

  /* we use appsink in push mode, it sends us a signal when data is available
   * and we pull out the data in the signal callback. We want the appsink to
   * push as fast as it can, hence the sync=false */
  testsink = gst_bin_get_by_name(GST_BIN(data->src_pipeline), "testsink");
  print_pad_capabilities(testsink, "sink");

  /* setting up sink pipeline, we push audio data into this pipeline that will
   * then play it back using the default audio sink. We have no blocking
   * behaviour on the src which means that we will push the entire file into
   * memory. */
  string = g_strdup_printf(
      "appsrc name=testsource ! "
      "video/x-raw,format=RGB,width=1280,height=720,framerate=10/1  ! "
      "videoconvert ! fpsdisplaysink");
  data->sink_pipeline = gst_parse_launch(string, NULL);
  g_free(string);

  if (data->sink_pipeline == NULL) {
    g_print("Bad sink\n");
    return -1;
  }

  testsource = gst_bin_get_by_name(GST_BIN(data->sink_pipeline), "testsource");
  /* configure for time-based format */
  g_object_set(testsource, "format", GST_FORMAT_TIME, NULL);
  /* uncomment the next line to block when appsrc has buffered enough */
  /* g_object_set (testsource, "block", TRUE, NULL); */

  bus = gst_element_get_bus(data->sink_pipeline);
  gst_bus_add_watch(bus, (GstBusFunc)on_appsrc_message, data);
  gst_object_unref(bus);

  /* launching things */
  gst_element_set_state(data->src_pipeline, GST_STATE_PLAYING);
  gst_element_set_state(data->sink_pipeline, GST_STATE_PLAYING);

  GstSample *sample;
  while (true) {
    /* Retrieve the buffer */
    g_signal_emit_by_name(testsink, "pull-sample", &sample);
    if (!sample) {
      g_print("pull-sample fail\n");
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    uint8 pixel_data[info.size];
    // Copy image
    memmove(pixel_data, info.data, info.size);
    gst_buffer_unmap(buffer, &info);

    auto input_frame = absl::make_unique<mediapipe::ImageFrame>();
    input_frame->CopyPixelData(
        mediapipe::ImageFormat::SRGB, 1280, 720, pixel_data,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);

    // Send image packet into the graph.
    size_t frame_timestamp_us = GST_BUFFER_PTS(buffer);
    gst_sample_unref(sample);
    absl::Status status = graph.AddPacketToInputStream(
        kInputStream, mediapipe::Adopt(input_frame.release())
                          .At(mediapipe::Timestamp(frame_timestamp_us)));
    if(!status.ok()){
      LOG(ERROR) << "AddPacketToInputStream";
    }
    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    if (!poller.Next(&packet)) {
      return -1;
    }
    auto &output_frame = packet.Get<mediapipe::ImageFrame>();

    GstMemory *memory;
    gint size, width = 1280, height = 720, bpp = 3;
    size = width * height * bpp;
    buffer = gst_buffer_copy(buffer);
    memory = gst_allocator_alloc (NULL, size, NULL);
    gst_buffer_insert_memory (buffer, -1, memory);

    gst_buffer_map(buffer, &info, GST_MAP_WRITE);
    // Copy image
    memmove(info.data, output_frame.PixelData(), output_frame.PixelDataSize());
    gst_buffer_unmap(buffer, &info);

    GstFlowReturn ret;
    /* Push the buffer into the appsrc */
    g_signal_emit_by_name(testsource, "push-buffer", buffer, &ret);
    /* Free the buffer now that we are done with it */

    if (ret != GST_FLOW_OK) {
      /* We got some error, stop sending data */
      return -1;
    }
  }

  gst_object_unref(testsink);
  gst_object_unref(testsource);
  gst_element_set_state(data->src_pipeline, GST_STATE_NULL);
  gst_element_set_state(data->sink_pipeline, GST_STATE_NULL);

  gst_object_unref(data->src_pipeline);
  gst_object_unref(data->sink_pipeline);
  g_main_loop_unref(data->loop);
  g_free(data);

  return 0;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);

  mediapipe::CalculatorGraph graph;
  std::string calculator_graph_config_contents;
  mediapipe::file::GetContents(
      absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents);
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  absl::Status run_status = graph.Initialize(config);
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to init the graph: " << run_status.message();
    return EXIT_FAILURE;
  }

  mediapipe::StatusOrPoller status_or_poller =
      graph.AddOutputStreamPoller(kOutputStream);
  if (!status_or_poller.ok()) {
    LOG(ERROR) << "AddOutputStreamPoller";
    return EXIT_FAILURE;
  }
  mediapipe::OutputStreamPoller poller = std::move(status_or_poller.value());

  LOG(INFO) << "Start running the calculator graph.";
  run_status = graph.StartRun({});
  if (!run_status.ok()) {
    LOG(ERROR) << "StartRun: " << run_status.message();
    return EXIT_FAILURE;
  }

  int ret = gstreamer_main(argc, argv, graph, poller);
  LOG(INFO) << "Shutting down with gst main ret: " << ret;
  graph.CloseInputStream(kInputStream);
  run_status = graph.WaitUntilDone();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }

  return EXIT_SUCCESS;
}
