
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>
#include <string>
#include <csignal>
#include <atomic>
#include <chrono>
#include <sched.h>
#include <errno.h>
#include <queue>
#include <condition_variable>

#include <iostream>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <map>
#include <iomanip>
#include "NvJpegDecoder.h"
#include "NvBufSurface.h"
#include <NvBuffer.h>
#include <nvbuf_utils.h>
#include<sys/stat.h>
#include <algorithm>
#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <NvJpegDecoder.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <vector>
#include <string>
#include <cstring>

#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <glib.h>
#include <vector>
#include <NvJpegDecoder.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>



#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <gst/allocators/gstdmabuf.h>
#include <glib.h>
#include <vector>
#include <NvJpegDecoder.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>


// 3. GStreamer's DMABuf allocator for wrapping the hardware buffer
#include <iomanip>     // For hex output
#include <sstream>     // For formatted output
// Keep this globally initialized
// Global variables


#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <NvJpegDecoder.h>
#include <nvbufsurface.h>
#include <fstream>
#include <sstream>
#include <iomanip>




GMainLoop *g_loop = nullptr;
GstElement *g_pipeline = nullptr;
std::atomic<bool> g_running(true);
std::atomic<uint64_t> g_next_frame_id(1);
GstAppSrc* g_appsrc = nullptr;
int g_target_framerate = 20;
bool g_use_hardware_encoder = false;

// Signal handler for graceful shutdown
static void signal_handler(int signum) {
if (g_loop && signum == SIGINT) {
ROS_INFO("Received SIGINT, shutting down gracefully...");
g_running = false;
g_main_loop_quit(g_loop);
}
}


static NvJPEGDecoder* g_decoder = NvJPEGDecoder::createJPEGDecoder("jpegdec");
static GstAllocator* g_dmabuf_allocator = nullptr;
#include <fstream>
#include <string>

#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <NvBufSurface.h>

#include "NvUtils.h" 


// Custom function to map GstFlowReturn to string
static const char* get_gst_flow_return_name(GstFlowReturn flow_ret) {
   switch (flow_ret) {
       case GST_FLOW_OK: return "GST_FLOW_OK";
       case GST_FLOW_ERROR: return "GST_FLOW_ERROR";
       case GST_FLOW_NOT_LINKED: return "GST_FLOW_NOT_LINKED";
       case GST_FLOW_FLUSHING: return "GST_FLOW_FLUSHING";
       case GST_FLOW_EOS: return "GST_FLOW_EOS";
       case GST_FLOW_NOT_NEGOTIATED: return "GST_FLOW_NOT_NEGOTIATED";
       case GST_FLOW_NOT_SUPPORTED: return "GST_FLOW_NOT_SUPPORTED";
       default: return "UNKNOWN";
   }
}

static void push_frame_to_appsrc(const sensor_msgs::CompressedImage::ConstPtr& msg) {
 // Stage 1: Check appsrc buffer level
 guint current_level;
 g_object_get(G_OBJECT(g_appsrc), "current-level-bytes", &current_level, NULL);
 if (current_level > 1024 * 512) { // 512KB threshold
     return;
 }

 // Static variables for tracking previous frame
 static unsigned char* prev_frame_data = nullptr;
 static size_t prev_frame_size = 0;

 // Stage 2: Prepare JPEG data
 unsigned char* jpeg_buffer = const_cast<unsigned char*>(msg->data.data());
 size_t jpeg_size = msg->data.size();

 // Stage 3: Decode JPEG
 uint32_t pixfmt = 0, width = 0, height = 0;
 int output_fd = -1;
 int ret = g_decoder->decodeToFd(output_fd, jpeg_buffer, jpeg_size, pixfmt, width, height);
 if (ret < 0 || output_fd < 0) {
     if (output_fd >= 0) close(output_fd);
     return;
 }

 // Stage 4: Create surface from decoded FD
 NvBufSurface *output_surface = nullptr;
 ret = NvBufSurfaceFromFd(output_fd, (void**)&output_surface);
 if (ret != 0 || !output_surface) {
     close(output_fd);
     return;
 }

 // Stage 5: Create downsized surface (640x480, YUV420)
 NvBufSurface *downsized_surface = nullptr;
 NvBufSurfaceCreateParams create_params = {0};
 create_params.gpuId = 0;
 create_params.width = 640;
 create_params.height = 480;
 create_params.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
 create_params.layout = NVBUF_LAYOUT_PITCH;
 create_params.memType = NVBUF_MEM_DEFAULT;
 ret = NvBufSurfaceCreate(&downsized_surface, 1, &create_params);
 if (ret != 0 || !downsized_surface) {
     close(output_fd);
     return;
 }

 // Stage 6: Downsize using NvBufSurfTransform
 NvBufSurfTransformRect src_rect = {0, 0, width, height};
 NvBufSurfTransformRect dst_rect = {0, 0, 640, 480};
 NvBufSurfTransformParams transform_params = {0};
 transform_params.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
 transform_params.src_rect = &src_rect;
 transform_params.dst_rect = &dst_rect;
 transform_params.transform_filter = NvBufSurfTransformInter_Bilinear;
 ret = NvBufSurfTransform(output_surface, downsized_surface, &transform_params);
 if (ret != 0) {
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }

 // Stage 7: Map and sync surface for CPU access
 ret = NvBufSurfaceMap(downsized_surface, 0, -1, NVBUF_MAP_READ);
 if (ret != 0) {
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }
 ret = NvBufSurfaceSyncForCpu(downsized_surface, 0, -1);
 if (ret != 0) {
     NvBufSurfaceUnMap(downsized_surface, 0, -1);
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }

 // Stage 8: Get plane data
 void* y_data = downsized_surface->surfaceList[0].mappedAddr.addr[0];
 void* u_data = downsized_surface->surfaceList[0].planeParams.num_planes >= 2 ? downsized_surface->surfaceList[0].mappedAddr.addr[1] : nullptr;
 void* v_data = downsized_surface->surfaceList[0].planeParams.num_planes >= 3 ? downsized_surface->surfaceList[0].mappedAddr.addr[2] : nullptr;
 if (!y_data || !u_data || !v_data) {
     NvBufSurfaceUnMap(downsized_surface, 0, -1);
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }

 // Stage 9: Calculate plane sizes
 uint32_t final_width = 640, final_height = 480;
 uint32_t expected_y_size = final_width * final_height;
 uint32_t uv_width = final_width / 2, uv_height = final_height / 2;
 uint32_t expected_u_size = uv_width * uv_height;
 uint32_t expected_v_size = uv_width * uv_height;
 uint32_t expected_total = expected_y_size + expected_u_size + expected_v_size;
 uint32_t y_pitch = downsized_surface->surfaceList[0].planeParams.pitch[0];
 uint32_t u_pitch = downsized_surface->surfaceList[0].planeParams.num_planes >= 2 ? downsized_surface->surfaceList[0].planeParams.pitch[1] : 0;
 uint32_t v_pitch = downsized_surface->surfaceList[0].planeParams.num_planes >= 3 ? downsized_surface->surfaceList[0].planeParams.pitch[2] : 0;

 // Stage 10: Allocate and copy to current frame buffer
 unsigned char* curr_frame_data = (unsigned char*)malloc(expected_total);
 if (!curr_frame_data) {
     NvBufSurfaceUnMap(downsized_surface, 0, -1);
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }
 size_t offset = 0;
 for (uint32_t row = 0; row < final_height; row++) {
     char* row_data = (char*)y_data + (row * y_pitch);
     memcpy(curr_frame_data + offset, row_data, final_width);
     offset += final_width;
 }
 for (uint32_t row = 0; row < uv_height; row++) {
     char* row_data = (char*)u_data + (row * u_pitch);
     memcpy(curr_frame_data + offset, row_data, uv_width);
     offset += uv_width;
 }
 for (uint32_t row = 0; row < uv_height; row++) {
     char* row_data = (char*)v_data + (row * v_pitch);
     memcpy(curr_frame_data + offset, row_data, uv_width);
     offset += uv_width;
 }

 // Stage 11: Check frame identicality
 if (prev_frame_data && prev_frame_size == expected_total) {
     bool identical = true;
     for (size_t i = 0; i < expected_total; i++) {
         if (curr_frame_data[i] != prev_frame_data[i]) {
             identical = false;
             break;
         }
     }
     if (identical) {
         free(curr_frame_data);
         NvBufSurfaceUnMap(downsized_surface, 0, -1);
         NvBufSurfaceDestroy(downsized_surface);
         close(output_fd);
         return;
     }
 }

 // Stage 12: Update previous frame data
 if (prev_frame_data) {
     free(prev_frame_data);
 }
 prev_frame_data = curr_frame_data;
 prev_frame_size = expected_total;

 // Stage 13: Allocate GStreamer buffer
 GstBuffer *gst_buffer = gst_buffer_new_allocate(NULL, expected_total, NULL);
 if (!gst_buffer) {
     free(prev_frame_data);
     prev_frame_data = nullptr;
     NvBufSurfaceUnMap(downsized_surface, 0, -1);
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }

 // Stage 14: Map and copy to GStreamer buffer
 GstMapInfo map;
 if (!gst_buffer_map(gst_buffer, &map, GST_MAP_WRITE)) {
     gst_buffer_unref(gst_buffer);
     free(prev_frame_data);
     prev_frame_data = nullptr;
     NvBufSurfaceUnMap(downsized_surface, 0, -1);
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }
 memcpy(map.data, curr_frame_data, expected_total);
 gst_buffer_unmap(gst_buffer, &map);

 // Stage 15: Set timestamp
 if (msg->header.stamp.isValid()) {
     GST_BUFFER_PTS(gst_buffer) = msg->header.stamp.toNSec();
     GST_BUFFER_DTS(gst_buffer) = GST_BUFFER_PTS(gst_buffer);
 }

 // Stage 16: Push buffer to appsrc
 GstFlowReturn flow_ret = gst_app_src_push_buffer(g_appsrc, gst_buffer);
 if (flow_ret != GST_FLOW_OK) {
     gst_buffer_unref(gst_buffer);
     free(prev_frame_data);
     prev_frame_data = nullptr;
     NvBufSurfaceUnMap(downsized_surface, 0, -1);
     NvBufSurfaceDestroy(downsized_surface);
     close(output_fd);
     return;
 }
 printf("Successfully pushed buffer to appsrc\n");

 // Stage 17: Cleanup
 NvBufSurfaceUnMap(downsized_surface, 0, -1);
 NvBufSurfaceDestroy(downsized_surface);
 close(output_fd);
}


// GStreamer bus callback
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
GMainLoop *loop = static_cast<GMainLoop*>(data);
switch (GST_MESSAGE_TYPE(msg)) {
  case GST_MESSAGE_EOS:
      ROS_INFO("End-of-stream reached");
      g_main_loop_quit(loop);
      break;
  case GST_MESSAGE_ERROR: {
      gchar *debug = nullptr;
      GError *error = nullptr;
      gst_message_parse_error(msg, &error, &debug);
      ROS_ERROR("GStreamer Error: %s, debug: %s",
               error ? error->message : "Unknown error",
               debug ? debug : "None");
      if (GST_MESSAGE_SRC(msg)) {
          ROS_ERROR("Error from element: %s", GST_OBJECT_NAME(GST_MESSAGE_SRC(msg)));
      }
      g_error_free(error);
      g_free(debug);
      g_main_loop_quit(loop);
      break;
  }
  case GST_MESSAGE_WARNING: {
      gchar *debug = nullptr;
      GError *warning = nullptr;
      gst_message_parse_warning(msg, &warning, &debug);
      ROS_WARN("GStreamer Warning: %s, debug: %s",
              warning ? warning->message : "Unknown warning",
              debug ? debug : "None");
      g_error_free(warning);
      g_free(debug);
      break;
  }
  default:
      break;
}
return TRUE;
}




// Buffer probe callback to print first 16 bytes of Y-plane
static GstPadProbeReturn buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
   GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
   if (buffer) {
       GstMapInfo map;
       if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {

           printf("\n");
           gst_buffer_unmap(buffer, &map);
       }
   }
   return GST_PAD_PROBE_OK;
}  


// Global variables for framerate control
std::queue<sensor_msgs::CompressedImage::ConstPtr> g_frame_queue;
std::mutex g_frame_queue_mutex;
bool g_frame_ready = false;
std::mutex g_frame_ready_mutex;
std::condition_variable g_frame_ready_cv;

// Modified callback to queue frames instead of processing immediately
void queue_frame_callback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_frame_queue_mutex);
    
    // Keep only the latest frame in queue to avoid buffering delays
    while (!g_frame_queue.empty()) {
        g_frame_queue.pop();
    }
    g_frame_queue.push(msg);
    
    // Signal that a frame is available
    {
        std::lock_guard<std::mutex> ready_lock(g_frame_ready_mutex);
        g_frame_ready = true;
    }
    g_frame_ready_cv.notify_one();
}

// Timer callback to process frames at exact framerate
void process_frame_at_framerate(const ros::TimerEvent& event) {
    if (!g_running) return;
    
    sensor_msgs::CompressedImage::ConstPtr frame_to_process;
    
    // Get the latest frame from queue
    {
        std::lock_guard<std::mutex> lock(g_frame_queue_mutex);
        if (g_frame_queue.empty()) {
            return; // No frame available, skip this cycle
        }
        frame_to_process = g_frame_queue.front();
        g_frame_queue.pop();
    }
    
    // Process the frame using the existing function
    push_frame_to_appsrc(frame_to_process);
}

int main(int argc, char *argv[]) {
   // System optimizations

   // CPU affinity to core 1
   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(1, &cpuset);
   if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
       ROS_WARN("Failed to set CPU affinity to core 1: %s", strerror(errno));
   } else {
       ROS_INFO("Successfully set CPU affinity to core 1 for the video streamer process.");
   }

   // ROS initialization
   ros::init(argc, argv, "video_streamer");
   ros::NodeHandle nh("~");

   // Load parameters
   std::string image_topic, image_format, receiver_ip;
   int output_width, output_height, framerate, bitrate;   
   // Get required parameters - will fail if not found in YAML

  nh.param("image_topic", image_topic, std::string("/test/Station/MDH_01/cam5/Compressed"));
   nh.param("image_format", image_format, std::string("yuv420"));
   nh.param("receiver_ip", receiver_ip, std::string("10.72.99.127"));
   nh.param("output_width", output_width, 640);
   nh.param("output_height", output_height, 480);
   nh.param("framerate", framerate, 10);


 //  nh.param("bitrate", bitrate, 2000); // 2 Mbps
  nh.param("bitrate", bitrate, 1000); // 1 Mbps

   if (framerate <= 0) {
       ROS_WARN("Invalid framerate: %d, setting to default 20", framerate);
       framerate = 10;
   }
   g_target_framerate = framerate;

   ROS_INFO("Starting ULTRA-LOW LATENCY video streamer with parameters:");
   ROS_INFO("  Image topic: %s", image_topic.c_str());
   ROS_INFO("  Image format: %s", image_format.c_str());
   ROS_INFO("  Receiver IP: %s", receiver_ip.c_str());
   ROS_INFO("  Output resolution: %dx%d", output_width, output_height);
   ROS_INFO("  Framerate: %d", framerate);
   ROS_INFO("  Bitrate: %d kbps", bitrate);

   // GStreamer initialization
   gst_init(&argc, &argv);
   g_loop = g_main_loop_new(nullptr, FALSE);
   signal(SIGINT, signal_handler);

   // Create pipeline
   g_pipeline = gst_pipeline_new("video-sender");
   g_appsrc = GST_APP_SRC(gst_element_factory_make("appsrc", "source"));

   GstElement *x264enc = gst_element_factory_make("x264enc", "encoder");
   GstElement *rtph264pay = gst_element_factory_make("rtph264pay", "payloader");
   GstElement *udpsink = gst_element_factory_make("udpsink", "sink");

   // Check if all elements were created
   if (!g_pipeline || !g_appsrc || !x264enc || !rtph264pay || !udpsink) {
       ROS_ERROR("Failed to create one or more GStreamer elements.");
       if (g_pipeline) gst_object_unref(g_pipeline);
       if (g_loop) g_main_loop_unref(g_loop);
       return -1;
   }

   // Configure appsrc
   std::string appsrc_caps_str = "video/x-raw,format=I420,width=" + std::to_string(output_width) +
                                 ",height=" + std::to_string(output_height) +
                                 ",framerate=" + std::to_string(framerate) + "/1";
   GstCaps *appsrc_caps = gst_caps_from_string(appsrc_caps_str.c_str());
   g_object_set(G_OBJECT(g_appsrc),
                "caps", appsrc_caps,
                "format", GST_FORMAT_TIME,
                "stream-type", GST_APP_STREAM_TYPE_STREAM,
                "is-live", TRUE,
                "do-timestamp", TRUE,
                "max-bytes", 1024 * 1024 * 10,
                "block", FALSE,
                "emit-signals", FALSE,
                NULL);
   gst_caps_unref(appsrc_caps);

   // Configure x264enc for low latency
   g_object_set(G_OBJECT(x264enc),
            "tune", "zerolatency",
            "speed-preset", "ultrafast",
            "bitrate", bitrate,
            "threads", 1,
            "bframes", 0,
            "key-int-max", framerate,
            "vbv-buf-capacity", 600,
            "rc-lookahead", 0,
            "sliced-threads", TRUE,
            NULL);

   // Configure rtph264pay
   g_object_set(G_OBJECT(rtph264pay),
                "config-interval", -1,
                "pt", 96,
                "mtu", 1400,
                NULL);

   // Configure udpsink
   g_object_set(G_OBJECT(udpsink),
                "host", receiver_ip.c_str(),
                "port", 5000,
                "sync", FALSE,
                "async", FALSE,
                "qos", FALSE,
                NULL);

   // Add buffer probe to udpsink sink pad
   GstPad *sink_pad = gst_element_get_static_pad(udpsink, "sink");
   if (sink_pad) {
       gst_pad_add_probe(sink_pad, GST_PAD_PROBE_TYPE_BUFFER, buffer_probe, NULL, NULL);
       gst_object_unref(sink_pad);
   } else {
       ROS_WARN("Failed to get udpsink sink pad for buffer probe");
   }

   // Configure pipeline
   g_object_set(G_OBJECT(g_pipeline),
                "latency", 50000000,
                "auto-flush-bus", TRUE,
                NULL);

   // Add elements to pipeline
   gst_bin_add_many(GST_BIN(g_pipeline), GST_ELEMENT(g_appsrc), x264enc, rtph264pay, udpsink, NULL);

   // Link pipeline elements
   if (!gst_element_link_many(GST_ELEMENT(g_appsrc), x264enc, rtph264pay, udpsink, NULL)) {
       ROS_ERROR("Failed to link pipeline elements.");
       GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(g_pipeline));
       GstMessage *msg = gst_bus_timed_pop_filtered(bus, 1 * GST_SECOND, GST_MESSAGE_ERROR);
       if (msg) {
           gchar *debug = nullptr;
           GError *error = nullptr;
           gst_message_parse_error(msg, &error, &debug);
           ROS_ERROR("Linking Error: %s, debug: %s",
                     error ? error->message : "Unknown",
                     debug ? debug : "None");
           g_error_free(error);
           g_free(debug);
           gst_message_unref(msg);
       }
       gst_object_unref(bus);
       gst_object_unref(g_pipeline);
       g_main_loop_unref(g_loop);
       return -1;
   }

   // Set up bus
   GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(g_pipeline));
   guint bus_watch_id = gst_bus_add_watch(bus, bus_call, g_loop);
   gst_object_unref(bus);

   // Start pipeline
   GstStateChangeReturn ret = gst_element_set_state(g_pipeline, GST_STATE_PLAYING);
   if (ret == GST_STATE_CHANGE_FAILURE) {
       ROS_ERROR("Failed to start GStreamer pipeline to PLAYING state.");
       GstBus *error_bus = gst_pipeline_get_bus(GST_PIPELINE(g_pipeline));
       GstMessage *msg = gst_bus_timed_pop_filtered(error_bus, 5 * GST_SECOND, GST_MESSAGE_ERROR);
       if (msg) {
           gchar *debug = nullptr;
           GError *error = nullptr;
           gst_message_parse_error(msg, &error, &debug);
           ROS_ERROR("Pipeline Error: %s, debug: %s",
                     error ? error->message : "Unknown",
                     debug ? debug : "None");
           if (GST_MESSAGE_SRC(msg)) {
               ROS_ERROR("Error from element: %s", GST_OBJECT_NAME(GST_MESSAGE_SRC(msg)));
           }
           g_error_free(error);
           g_free(debug);
           gst_message_unref(msg);
       }
       gst_object_unref(error_bus);
       gst_object_unref(g_pipeline);
       g_main_loop_unref(g_loop);
       return -1;
   }

   // Wait for pipeline to reach PLAYING state
   ret = gst_element_get_state(g_pipeline, NULL, NULL, 10 * GST_SECOND);
   if (ret == GST_STATE_CHANGE_FAILURE) {
       ROS_ERROR("Pipeline failed to reach PLAYING state within timeout.");
       gst_element_set_state(g_pipeline, GST_STATE_NULL);
       gst_object_unref(g_pipeline);
       g_main_loop_unref(g_loop);
       return -1;
   }

   ROS_INFO("Pipeline successfully started and reached PLAYING state");

   // ROS subscriber with queue size of 1 to collect frames (but not process immediately)
   ros::Subscriber sub = nh.subscribe<sensor_msgs::CompressedImage>(
       image_topic, 1, queue_frame_callback);
   if (!sub) {
       ROS_ERROR("Failed to create ROS subscriber");
       gst_element_set_state(g_pipeline, GST_STATE_NULL);
       gst_object_unref(g_pipeline);
       g_main_loop_unref(g_loop);
       return -1;
   }

   ROS_INFO("Subscribed to topic %s. Waiting for images...", image_topic.c_str());
   
   // Create timer to strictly enforce framerate - process exactly 'framerate' frames per second
   ros::Timer frame_timer = nh.createTimer(
       ros::Duration(1.0 / framerate), 
       process_frame_at_framerate
   );

   ros::AsyncSpinner spinner(1);
   spinner.start();

   // Run the main loop
   g_main_loop_run(g_loop);
   g_running = false;
   spinner.stop();

   // Proper cleanup sequence
   ROS_INFO("Shutting down gracefully...");
   if (g_decoder) {
       delete g_decoder;
       g_decoder = nullptr;
   }
   gst_element_set_state(g_pipeline, GST_STATE_NULL);
   gst_object_unref(g_pipeline);
   g_source_remove(bus_watch_id);
   g_main_loop_unref(g_loop);
   ros::shutdown();

   ROS_INFO("Shutdown complete");
   return 0;
}
