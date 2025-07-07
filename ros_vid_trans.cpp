#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>
#include <string>
#include <boost/bind.hpp>
#include <sched.h>
#include <errno.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <cstring>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <sstream>

// Global variables
GMainLoop *g_loop = NULL;
GstElement *g_pipeline = NULL;
std::atomic<bool> g_running(true);
std::atomic<uint64_t> g_frame_count(0);
std::atomic<uint64_t> g_frames_read(0);
std::atomic<uint64_t> g_frames_sent(0);
std::atomic<uint64_t> g_frames_repeated(0);
std::atomic<uint64_t> g_rtp_frames_sent(0);

struct FrameData {
	std::vector<uint8_t> data;
	uint64_t timestamp_ns;
	uint64_t frame_id;
	uint64_t content_hash;
    
	FrameData() : timestamp_ns(0), frame_id(0), content_hash(0) {}
	FrameData(const FrameData& other) : data(other.data), timestamp_ns(other.timestamp_ns),
                                   	frame_id(other.frame_id), content_hash(other.content_hash) {}
	FrameData& operator=(const FrameData& other) {
    	if (this != &other) {
        	data = other.data;
        	timestamp_ns = other.timestamp_ns;
        	frame_id = other.frame_id;
        	content_hash = other.content_hash;
    	}
    	return *this;
	}
};

std::queue<FrameData> frame_queue;
std::mutex frame_mutex;
std::condition_variable frame_cv;
GstAppSrc* g_appsrc = nullptr;
int g_target_framerate = 10;
std::atomic<uint64_t> g_next_frame_id(1);

std::mutex rtp_stats_mutex;
std::vector<uint64_t> rtp_frame_hashes;
uint64_t last_rtp_hash = 0;
uint64_t rtp_identical_count = 0;
uint64_t rtp_different_count = 0;

std::mutex scaler_stats_mutex;
uint64_t last_scaler_hash = 0;
uint64_t scaler_identical_count = 0;
uint64_t scaler_different_count = 0;

static void signal_handler(int signum) {
	if (g_loop && signum == SIGINT) {
    	ROS_INFO("Received SIGINT, shutting down gracefully...");
    	g_running = false;
    	g_main_loop_quit(g_loop);
	}
}

static uint64_t calculate_frame_hash(const std::vector<uint8_t>& data) {
	if (data.empty()) return 0;
	uint64_t hash = data.size();
	if (data.size() > 0) hash ^= (static_cast<uint64_t>(data[0]) << 32);
	if (data.size() > 1) hash ^= (static_cast<uint64_t>(data[data.size()/2]) << 16);
	if (data.size() > 2) hash ^= static_cast<uint64_t>(data[data.size()-1]);
	for (size_t i = 0; i < data.size(); i += 100) {
    	hash ^= static_cast<uint64_t>(data[i]) << (i % 32);
	}
	return hash;
}

static void analyze_rtp_frame(const FrameData& frame_data, bool is_new_frame) {
	std::lock_guard<std::mutex> lock(rtp_stats_mutex);
	uint64_t current_hash = frame_data.content_hash;
	rtp_frame_hashes.push_back(current_hash);
	bool content_changed = (current_hash != last_rtp_hash);
	if (content_changed) {
    	rtp_different_count++;
    	ROS_INFO("🔄 RTP FRAME ANALYSIS: Frame %lu - CONTENT CHANGED (hash: %016lx -> %016lx) [%s]",
             	frame_data.frame_id, last_rtp_hash, current_hash,
             	is_new_frame ? "NEW" : "REPEATED");
	} else {
    	rtp_identical_count++;
    	ROS_WARN("⚠️  RTP FRAME ANALYSIS: Frame %lu - IDENTICAL CONTENT (hash: %016lx) [%s] - POTENTIAL ISSUE!",
             	frame_data.frame_id, current_hash,
             	is_new_frame ? "NEW" : "REPEATED");
	}
	last_rtp_hash = current_hash;
	if ((rtp_different_count + rtp_identical_count) % 10 == 0) {
    	ROS_INFO("📊 RTP TRANSMISSION STATS: Total=%lu, Different=%lu (%.1f%%), Identical=%lu (%.1f%%)",
             	rtp_different_count + rtp_identical_count,
             	rtp_different_count,
             	(100.0 * rtp_different_count) / (rtp_different_count + rtp_identical_count),
             	rtp_identical_count,
             	(100.0 * rtp_identical_count) / (rtp_different_count + rtp_identical_count));
	}
}

static void analyze_scaler_frame(GstBuffer *buffer, uint64_t frame_id) {
	std::lock_guard<std::mutex> lock(scaler_stats_mutex);
	GstMapInfo map;
	if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    	ROS_ERROR("❌ Failed to map buffer for scaler hash");
    	return;
	}
	size_t buffer_size = map.size;
	size_t expected_size = 640 * 480 * 3 / 2; // I420 format: 640x480x1.5 bytes
	if (buffer_size != expected_size) {
    	ROS_ERROR("❌ Scaler buffer size mismatch: got %zu bytes, expected %zu bytes", buffer_size, expected_size);
	} else {
    	ROS_INFO("✅ Scaler buffer size: %zu bytes (matches expected)", buffer_size);
	}
	std::vector<uint8_t> data(map.data, map.data + map.size);
	uint64_t current_hash = calculate_frame_hash(data);
	gst_buffer_unmap(buffer, &map);

	bool content_changed = (current_hash != last_scaler_hash);
	if (content_changed) {
    	scaler_different_count++;
    	ROS_INFO("🔍 SCALER FRAME ANALYSIS: Frame %lu - CONTENT CHANGED (hash: %016lx -> %016lx)",
             	frame_id, last_scaler_hash, current_hash);
	} else {
    	scaler_identical_count++;
    	ROS_WARN("⚠️  SCALER FRAME ANALYSIS: Frame %lu - IDENTICAL CONTENT (hash: %016lx) - POTENTIAL ISSUE!",
             	frame_id, current_hash);
	}
	last_scaler_hash = current_hash;
	if ((scaler_different_count + scaler_identical_count) % 10 == 0) {
    	ROS_INFO("📊 SCALER TRANSMISSION STATS: Total=%lu, Different=%lu (%.1f%%), Identical=%lu (%.1f%%)",
             	scaler_different_count + scaler_identical_count,
             	scaler_different_count,
             	(100.0 * scaler_different_count) / (scaler_different_count + scaler_identical_count),
             	scaler_identical_count,
             	(100.0 * scaler_identical_count) / (scaler_different_count + scaler_identical_count));
	}
}

static GstPadProbeReturn scaler_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
	GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
	if (buffer == NULL) {
    	ROS_WARN("⚠️ Scaler probe: Null buffer received");
    	return GST_PAD_PROBE_OK;
	}
	GstReferenceTimestampMeta *meta = gst_buffer_get_reference_timestamp_meta(buffer, gst_caps_new_empty_simple("application/x-frame-id"));
	uint64_t frame_id = meta ? meta->timestamp : g_frame_count.load();
	analyze_scaler_frame(buffer, frame_id);
	return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn decoder_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
	GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
	if (buffer == NULL) {
    	ROS_WARN("⚠️ Decoder probe: Null buffer received");
    	return GST_PAD_PROBE_OK;
	}
	GstMapInfo map;
	if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    	size_t expected_size = 4208 * 3120 * 3 / 2; // I420 format: 4208x3120x1.5 bytes
    	ROS_INFO("🔍 DECODER OUTPUT: Buffer size=%zu bytes, expected=%zu bytes, PTS=%lu ns",
             	map.size, expected_size, GST_BUFFER_PTS(buffer));
    	if (map.size != expected_size) {
        	ROS_ERROR("❌ Decoder buffer size mismatch: got %zu bytes, expected %zu bytes", map.size, expected_size);
    	}
    	gst_buffer_unmap(buffer, &map);
	} else {
    	ROS_ERROR("❌ Failed to map decoder output buffer");
	}
	return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn jpegparse_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
	GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
	if (buffer == NULL) {
    	ROS_WARN("⚠️ Jpegparse probe: Null buffer received");
    	return GST_PAD_PROBE_OK;
	}
	GstMapInfo map;
	if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    	ROS_INFO("🔍 JPEGPARSE OUTPUT: Buffer size=%zu bytes, PTS=%lu ns", map.size, GST_BUFFER_PTS(buffer));
    	if (map.size < 100000) { // JPEG frames should be ~786,157 bytes
        	ROS_ERROR("❌ Jpegparse buffer size too small: got %zu bytes", map.size);
    	}
    	gst_buffer_unmap(buffer, &map);
	} else {
    	ROS_ERROR("❌ Failed to map jpegparse output buffer");
	}
	return GST_PAD_PROBE_OK;
}

static std::string get_frame_content_summary(const FrameData& frame) {
	if (frame.data.empty()) return "EMPTY";
	std::stringstream ss;
	ss << "Size=" << frame.data.size()
   	<< ", Hash=" << std::hex << frame.content_hash
   	<< ", First4=[" << std::hex;
	for (size_t i = 0; i < std::min(size_t(4), frame.data.size()); ++i) {
    	ss << std::setfill('0') << std::setw(2) << static_cast<int>(frame.data[i]);
    	if (i < 3 && i < frame.data.size() - 1) ss << " ";
	}
	ss << "]";
	return ss.str();
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
	GMainLoop *loop = (GMainLoop *)data;
	switch (GST_MESSAGE_TYPE(msg)) {
    	case GST_MESSAGE_EOS:
        	ROS_INFO("End-of-stream reached");
        	g_main_loop_quit(loop);
        	break;
    	case GST_MESSAGE_ERROR: {
        	gchar *debug = NULL;
        	GError *error = NULL;
        	gst_message_parse_error(msg, &error, &debug);
        	ROS_ERROR("🚨 GStreamer Error: %s", error ? error->message : "Unknown error");
        	ROS_ERROR("Debug info: %s", debug ? debug : "None");
        	g_error_free(error);
        	g_free(debug);
        	g_main_loop_quit(loop);
        	break;
    	}
    	case GST_MESSAGE_WARNING: {
        	gchar *debug = NULL;
        	GError *error = NULL;
        	gst_message_parse_warning(msg, &error, &debug);
        	ROS_WARN("⚠️  GStreamer Warning: %s", error ? error->message : "Unknown warning");
        	g_error_free(error);
        	g_free(debug);
        	break;
    	}
    	case GST_MESSAGE_STATE_CHANGED: {
        	GstState old_state, new_state;
        	gst_message_parse_state_changed(msg, &old_state, &new_state, NULL);
        	if (GST_MESSAGE_SRC(msg) == GST_OBJECT(g_pipeline)) {
            	ROS_INFO("🔄 Pipeline state changed from %s to %s",
                     	gst_element_state_get_name(old_state),
                     	gst_element_state_get_name(new_state));
        	}
        	break;
    	}
    	case GST_MESSAGE_STREAM_STATUS: {
        	GstStreamStatusType type;
        	GstElement *owner;
        	gst_message_parse_stream_status(msg, &type, &owner);
        	ROS_INFO("📡 Stream status: %d from %s", type, GST_ELEMENT_NAME(owner));
        	break;
    	}
    	case GST_MESSAGE_BUFFERING: {
        	gint percent = 0;
        	gst_message_parse_buffering(msg, &percent);
        	ROS_INFO("📡 Buffering: %d%%", percent);
        	break;
    	}
    	default:
        	break;
	}
	return TRUE;
}

static bool frames_are_identical(const FrameData& frame1, const FrameData& frame2) {
	if (frame1.data.size() != frame2.data.size()) {
    	return false;
	}
	if (frame1.data.empty() || frame2.data.empty()) {
    	return frame1.data.empty() && frame2.data.empty();
	}
	if (frame1.content_hash != frame2.content_hash) {
    	return false;
	}
	return memcmp(frame1.data.data(), frame2.data.data(), frame1.data.size()) == 0;
}

static void push_frame_to_appsrc(const FrameData& frame_data, bool is_new_frame, const FrameData* prev_frame) {
	if (!g_appsrc || !g_running) {
    	ROS_DEBUG("Cannot push frame: appsrc=%p, running=%d", g_appsrc, g_running.load());
    	return;
	}
	ROS_INFO("🚀 PUSHING FRAME: ID=%lu, %s, Content=%s",
         	frame_data.frame_id, is_new_frame ? "NEW" : "REPEATED",
         	get_frame_content_summary(frame_data).c_str());
	analyze_rtp_frame(frame_data, is_new_frame);
	GstBuffer *buffer = gst_buffer_new_allocate(NULL, frame_data.data.size(), NULL);
	if (!buffer) {
    	ROS_ERROR("❌ Failed to allocate GStreamer buffer");
    	return;
	}
	GstMapInfo map;
	if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    	ROS_ERROR("❌ Failed to map GStreamer buffer");
    	gst_buffer_unref(buffer);
    	return;
	}
	memcpy(map.data, frame_data.data.data(), frame_data.data.size());
	gst_buffer_unmap(buffer, &map);
	// Attach frame_id as metadata
	GstCaps *meta_caps = gst_caps_new_empty_simple("application/x-frame-id");
	if (meta_caps) {
    	gst_buffer_add_reference_timestamp_meta(buffer, meta_caps, frame_data.frame_id, GST_CLOCK_TIME_NONE);
    	gst_caps_unref(meta_caps);
	} else {
    	ROS_WARN("⚠️ Failed to create metadata caps for frame %lu", frame_data.frame_id);
	}
	static GstClockTime base_time = 0;
	static uint64_t frame_sequence = 0;
	static bool first_frame = true;
	if (base_time == 0) {
    	base_time = gst_util_get_timestamp();
	}
	GstClockTime frame_duration = GST_SECOND / g_target_framerate;
	GstClockTime timestamp = base_time + (frame_sequence * frame_duration);
	GST_BUFFER_PTS(buffer) = timestamp;
	GST_BUFFER_DTS(buffer) = timestamp;
	GST_BUFFER_DURATION(buffer) = frame_duration;
	if (first_frame || (is_new_frame && frame_sequence % (g_target_framerate * 2) == 0)) {
    	GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_DISCONT);
    	if (first_frame) first_frame = false;
	}
	frame_sequence++;
	ROS_INFO("📤 BUFFER PREPARED: PTS=%lu ns, DTS=%lu ns, Duration=%lu ns, Sequence=%lu, Size=%lu bytes",
         	GST_BUFFER_PTS(buffer), GST_BUFFER_DTS(buffer), GST_BUFFER_DURATION(buffer),
         	frame_sequence, frame_data.data.size());
	auto push_start = std::chrono::steady_clock::now();
	GstFlowReturn ret = gst_app_src_push_buffer(g_appsrc, buffer);
	auto push_end = std::chrono::steady_clock::now();
	auto push_duration = std::chrono::duration_cast<std::chrono::microseconds>(push_end - push_start).count();
	if (ret != GST_FLOW_OK) {
    	ROS_ERROR("❌ Failed to push buffer to appsrc: %s (%d)",
              	gst_flow_get_name(ret), ret);
    	return;
	}
	uint64_t rtp_count = g_rtp_frames_sent.fetch_add(1) + 1;
	ROS_INFO("✅ PUSHED TO RTP: Frame=%lu, RTP_Count=%lu, Flow=%s, %s, Push_Duration=%ld us",
         	frame_data.frame_id, rtp_count, gst_flow_get_name(ret),
         	is_new_frame ? "NEW_FRAME" : "REPEATED_FRAME", push_duration);
	if (prev_frame && !prev_frame->data.empty() && !frame_data.data.empty()) {
    	bool is_identical = frames_are_identical(frame_data, *prev_frame);
    	ROS_INFO("🔍 FRAME COMPARISON: Current=%lu vs Previous=%lu - %s (Hash: %016lx vs %016lx)",
             	frame_data.frame_id, prev_frame->frame_id,
             	is_identical ? "IDENTICAL" : "DIFFERENT",
             	frame_data.content_hash, prev_frame->content_hash);
    	if (is_identical && is_new_frame) {
        	ROS_ERROR("🚨 CAMERA ISSUE: NEW frame %lu has IDENTICAL content to previous frame %lu!",
                  	frame_data.frame_id, prev_frame->frame_id);
    	}
	} else {
    	ROS_INFO("🔍 FRAME COMPARISON: No previous frame available for comparison");
	}
	if (is_new_frame) {
    	uint64_t sent_count = g_frames_sent.fetch_add(1) + 1;
    	ROS_INFO("📊 NEW FRAME STATS: Unique=%lu, Repeated=%lu, Total_Pushed=%lu, RTP_Sent=%lu",
             	sent_count, g_frames_repeated.load(), g_frame_count.load() + 1, rtp_count);
	} else {
    	uint64_t repeated_count = g_frames_repeated.fetch_add(1) + 1;
    	ROS_INFO("📊 REPEATED FRAME STATS: Repeated=%lu, Unique=%lu, Total_Pushed=%lu, RTP_Sent=%lu",
             	repeated_count, g_frames_sent.load(), g_frame_count.load() + 1, rtp_count);
	}
	g_frame_count.fetch_add(1);
}

static void frame_pusher_thread() {
	ROS_INFO("🚀 Frame pusher thread started with target framerate: %d fps", g_target_framerate);
	auto frame_interval = std::chrono::milliseconds(1000 / g_target_framerate);
	FrameData current_frame;
	FrameData last_frame;
	bool has_current_frame = false;
	bool has_last_frame = false;
	ROS_INFO("⏰ Frame pusher thread: frame_interval = %ld ms", frame_interval.count());
	while (g_running) {
    	auto loop_start = std::chrono::steady_clock::now();
    	bool got_new_frame = false;
    	{
        	std::unique_lock<std::mutex> lock(frame_mutex);
        	if (frame_cv.wait_for(lock, frame_interval, [&]() {
            	return !frame_queue.empty() || !g_running;
        	})) {
            	if (!g_running) {
                	ROS_INFO("🛑 Frame pusher thread: Received shutdown signal");
                	break;
            	}
            	if (!frame_queue.empty()) {
                	if (has_current_frame) {
                    	last_frame = current_frame;
                    	has_last_frame = true;
                	}
                	current_frame = frame_queue.back();
                	size_t queue_size = frame_queue.size();
                	std::queue<FrameData> empty_queue;
                	frame_queue.swap(empty_queue);
                	has_current_frame = true;
                	got_new_frame = true;
                	ROS_INFO("📥 FRAME QUEUE: Got frame %lu, discarded %zu older frames, Content=%s",
                         	current_frame.frame_id, queue_size - 1,
                         	get_frame_content_summary(current_frame).c_str());
            	}
        	}
    	}
    	if (got_new_frame && has_current_frame) {
        	push_frame_to_appsrc(current_frame, true, has_last_frame ? &last_frame : nullptr);
    	} else if (has_current_frame) {
        	auto now = std::chrono::steady_clock::now();
        	static auto start_time = std::chrono::steady_clock::now();
        	auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_time);
        	current_frame.timestamp_ns = elapsed.count();
        	push_frame_to_appsrc(current_frame, false, has_last_frame ? &last_frame : nullptr);
    	} else {
        	ROS_WARN("⚠️  No frame available to push - waiting for first frame");
    	}
    	auto loop_end = std::chrono::steady_clock::now();
    	auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
    	auto remaining_time = frame_interval - loop_duration;
    	if (remaining_time > std::chrono::milliseconds(0)) {
        	std::this_thread::sleep_for(remaining_time);
    	}
	}
	ROS_INFO("🛑 Frame pusher thread stopped");
}

static void image_callback(const sensor_msgs::CompressedImage::ConstPtr& msg, const std::string& expected_format) {
	if (!g_running) {
    	ROS_DEBUG("Callback skipped: Not running");
    	return;
	}
	std::string msg_format = msg->format;
	std::transform(msg_format.begin(), msg_format.end(), msg_format.begin(), ::tolower);
	std::string expected = expected_format;
	std::transform(expected.begin(), expected.end(), expected.begin(), ::tolower);
	if (msg_format != expected) {
    	ROS_WARN_THROTTLE(1.0, "❌ Format mismatch: received %s, expected %s", msg->format.c_str(), expected_format.c_str());
    	return;
	}
	if (msg->data.empty()) {
    	ROS_WARN("❌ Received empty image data");
    	return;
	}
	FrameData frame_data;
	frame_data.data = msg->data;
	frame_data.frame_id = g_next_frame_id.fetch_add(1);
	frame_data.content_hash = calculate_frame_hash(frame_data.data);
	if (!msg->header.stamp.isZero()) {
    	frame_data.timestamp_ns = msg->header.stamp.toNSec();
	} else {
    	static auto start_time = std::chrono::steady_clock::now();
    	auto current_time = std::chrono::steady_clock::now();
    	auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - start_time);
    	frame_data.timestamp_ns = elapsed.count();
	}
	{
    	std::lock_guard<std::mutex> lock(frame_mutex);
    	size_t queue_size_before = frame_queue.size();
    	frame_queue.push(frame_data);
    	uint64_t read_count = g_frames_read.fetch_add(1) + 1;
    	ROS_INFO("📥 ROS CALLBACK: Frame %lu received, Stamp=%.3f s, Format=%s, Content=%s, Queue=%zu->%zu, Read=%lu",
             	frame_data.frame_id, msg->header.stamp.toSec(), msg->format.c_str(),
             	get_frame_content_summary(frame_data).c_str(),
             	queue_size_before, queue_size_before + 1, read_count);
    	frame_cv.notify_one();
	}
}

static void need_data_callback(GstAppSrc *appsrc, guint length, gpointer user_data) {
	size_t queue_size;
	{
    	std::lock_guard<std::mutex> lock(frame_mutex);
    	queue_size = frame_queue.size();
	}
	ROS_INFO("📡 APPSRC NEEDS DATA: length=%u, queue_size=%zu, read=%lu, sent=%lu, repeated=%lu, rtp_sent=%lu",
         	length, queue_size, g_frames_read.load(), g_frames_sent.load(),
         	g_frames_repeated.load(), g_rtp_frames_sent.load());
}

static void enough_data_callback(GstAppSrc *appsrc, gpointer user_data) {
	ROS_INFO("📡 APPSRC HAS ENOUGH DATA - may indicate buffering or slow processing");
}

int main(int argc, char *argv[]) {
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(2, &cpuset);
	if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
    	ROS_WARN("Failed to set CPU affinity: %s", strerror(errno));
	}
	ros::init(argc, argv, "video_streamer");
	ros::NodeHandle nh("~");
	std::string receiver_ip, image_topic, image_format;
	int input_width, input_height, output_width, output_height, framerate, bitrate;
	nh.param<std::string>("receiver_ip", receiver_ip, "10.72.99.127");
	nh.param<std::string>("image_topic", image_topic, "/autoscan/Station_186/MDH_01/cam1/Compressed");
	nh.param<std::string>("image_format", image_format, "jpeg");
	nh.param<int>("input_width", input_width, 4208);
	nh.param<int>("input_height", input_height, 3120);
	nh.param<int>("output_width", output_width, 640);
	nh.param<int>("output_height", output_height, 480);
	nh.param<int>("framerate", framerate, 10);
	nh.param<int>("bitrate", bitrate, 2000000);
	if (framerate <= 0) {
    	ROS_ERROR("Invalid framerate: %d, must be positive", framerate);
    	return -1;
	}
	g_target_framerate = framerate;
	ROS_INFO("🚀 Starting video streamer with parameters:");
	ROS_INFO("  📡 Receiver IP: %s", receiver_ip.c_str());
	ROS_INFO("  📷 Image topic: %s", image_topic.c_str());
	ROS_INFO("  🎬 Image format: %s", image_format.c_str());
	ROS_INFO("  📏 Input resolution: %dx%d", input_width, input_height);
	ROS_INFO("  📐 Output resolution: %dx%d", output_width, output_height);
	ROS_INFO("  🎯 Framerate: %d", framerate);
	ROS_INFO("  📊 Bitrate: %d", bitrate);
	gst_init(&argc, &argv);
	GMainLoop *loop = g_main_loop_new(NULL, FALSE);
	if (!loop) {
    	ROS_ERROR("Failed to create GMainLoop");
    	return -1;
	}

	g_loop = loop;
	signal(SIGINT, signal_handler);
	GstElement *pipeline = gst_pipeline_new("video-sender");
	GstElement *appsrc = gst_element_factory_make("appsrc", "source");
	GstElement *jpegparse = gst_element_factory_make("jpegparse", "jpegparse");
	GstElement *jpegdec = gst_element_factory_make("jpegdec", "decoder");
	GstElement *queue = gst_element_factory_make("queue", "queue");
	GstElement *videoscale = gst_element_factory_make("videoscale", "scaler");
	GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
	GstElement *x264enc = gst_element_factory_make("x264enc", "encoder");
	GstElement *rtph264pay = gst_element_factory_make("rtph264pay", "payloader");
	GstElement *rtpcapsfilter = gst_element_factory_make("capsfilter", "rtpcapsfilter");
	GstElement *udpsink = gst_element_factory_make("udpsink", "sink");
	if (!pipeline || !appsrc || !jpegparse || !jpegdec || !queue || !videoscale ||
    	!capsfilter || !x264enc || !rtph264pay || !rtpcapsfilter || !udpsink) {
    	ROS_ERROR("❌ Failed to create GStreamer elements");
    	if (pipeline) gst_object_unref(pipeline);
    	if (loop) g_main_loop_unref(loop);
    	return -1;
	}
	g_appsrc = GST_APP_SRC(appsrc);
	g_pipeline = pipeline;
	std::string caps_str = "image/jpeg,width=" + std::to_string(input_width) +
                       	",height=" + std::to_string(input_height) +
                       	",framerate=" + std::to_string(framerate) + "/1";
	GstCaps *caps = gst_caps_from_string(caps_str.c_str());
	g_object_set(G_OBJECT(appsrc),
             	"caps", caps,
             	"format", GST_FORMAT_TIME,
             	"stream-type", GST_APP_STREAM_TYPE_STREAM,
             	"is-live", TRUE,
             	"do-timestamp", FALSE,
             	"min-latency", 0,
             	"max-latency", 1000000000,
             	"block", FALSE,
             	"max-bytes", 0,
             	"max-buffers", 2,
             	"leaky-type", 2,
             	"emit-signals", TRUE,
             	NULL);
	gst_caps_unref(caps);
	g_signal_connect(appsrc, "need-data", G_CALLBACK(need_data_callback), NULL);
	g_signal_connect(appsrc, "enough-data", G_CALLBACK(enough_data_callback), NULL);
	g_object_set(G_OBJECT(queue),
             	"max-size-buffers", 2,
             	"max-size-bytes", 0,
             	"max-size-time", 0,
             	NULL);
	g_object_set(G_OBJECT(videoscale),
             	"add-borders", FALSE,
             	"method", 0, // Nearest-neighbor scaling
             	NULL);
	std::string outcaps_str = "video/x-raw,format=I420,width=" + std::to_string(output_width) +
                          	",height=" + std::to_string(output_height) + ",framerate=" +
                          	std::to_string(framerate) + "/1";
	GstCaps *outcaps = gst_caps_from_string(outcaps_str.c_str());
	g_object_set(G_OBJECT(capsfilter), "caps", outcaps, NULL);
	gst_caps_unref(outcaps);
	g_object_set(G_OBJECT(x264enc),
             	"bitrate", bitrate / 1000, // x264enc uses kbps
             	"speed-preset", 1, // Ultrafast
             	"tune", 4, // Zerolatency
             	"key-int-max", 5,
             	"vbv-buf-capacity", 10,
             	NULL);
	g_object_set(G_OBJECT(rtph264pay),
             	"config-interval", 1,
             	"pt", 96,
             	"mtu", 1400,
             	NULL);
	GstCaps *rtpcaps = gst_caps_from_string("application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96");
	g_object_set(G_OBJECT(rtpcapsfilter), "caps", rtpcaps, NULL);
	gst_caps_unref(rtpcaps);
	g_object_set(G_OBJECT(udpsink),
             	"host", receiver_ip.c_str(),
             	"port", 5000,
             	"sync", FALSE,
             	"async", FALSE,
             	"buffer-size", 65536,
             	NULL);
	gst_bin_add_many(GST_BIN(pipeline), appsrc, jpegparse, jpegdec, queue, videoscale,
                 	capsfilter, x264enc, rtph264pay, rtpcapsfilter, udpsink, NULL);
	if (!gst_element_link_many(appsrc, jpegparse, jpegdec, queue, videoscale, capsfilter,
                           	x264enc, rtph264pay, rtpcapsfilter, udpsink, NULL)) {
    	ROS_ERROR("❌ Failed to link GStreamer elements");
    	if (pipeline) gst_object_unref(pipeline);
    	if (loop) g_main_loop_unref(loop);
    	return -1;
	}
	// Add probe to jpegparse sink pad
	GstPad *jpegparse_sink_pad = gst_element_get_static_pad(jpegparse, "sink");
	gst_pad_add_probe(jpegparse_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, jpegparse_probe_callback, NULL, NULL);
	gst_object_unref(jpegparse_sink_pad);
	// Add probe to jpegdec sink pad
	GstPad *decoder_sink_pad = gst_element_get_static_pad(jpegdec, "sink");
	gst_pad_add_probe(decoder_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, decoder_probe_callback, NULL, NULL);
	gst_object_unref(decoder_sink_pad);
	// Add probe to capsfilter sink pad for post-scaler hashing
	GstPad *sink_pad = gst_element_get_static_pad(capsfilter, "sink");
	gst_pad_add_probe(sink_pad, GST_PAD_PROBE_TYPE_BUFFER, scaler_probe_callback, NULL, NULL);
	gst_object_unref(sink_pad);
	GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
	guint bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
	gst_object_unref(bus);
	ROS_INFO("Starting GStreamer pipeline...");
	GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
	if (ret == GST_STATE_CHANGE_FAILURE) {
    	ROS_ERROR("❌ Failed to start GStreamer pipeline");
    	if (pipeline) gst_object_unref(pipeline);
    	if (loop) g_main_loop_unref(loop);
    	return -1;
	}
	GstState state;
	ret = gst_element_get_state(pipeline, &state, NULL, 5 * GST_SECOND);
	if (ret != GST_STATE_CHANGE_SUCCESS) {
    	ROS_ERROR("❌ Failed to reach PLAYING state within timeout");
    	if (pipeline) gst_object_unref(pipeline);
    	if (loop) g_main_loop_unref(loop);
    	return -1;
	}
	ROS_INFO("✅ Pipeline started successfully");
	ros::Subscriber sub = nh.subscribe<sensor_msgs::CompressedImage>(
    	image_topic, 10, boost::bind(image_callback, _1, image_format));
	ros::AsyncSpinner spinner(1);
	spinner.start();
	std::thread pusher_thread(frame_pusher_thread);
	ROS_INFO("🎥 Video streamer is running. Press Ctrl+C to stop.");
	g_main_loop_run(loop);
	ROS_INFO("🛑 Shutting down...");
	g_running = false;
	if (pusher_thread.joinable()) {
    	pusher_thread.join();
	}
	gst_element_set_state(pipeline, GST_STATE_NULL);
	gst_object_unref(pipeline);
	g_source_remove(bus_watch_id);
	g_main_loop_unref(loop);
	spinner.stop();
	ros::shutdown();
	ROS_INFO("✅ Video streamer stopped");
	return 0;
}



