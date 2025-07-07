ROS Video Streaming with GStreamer
This project provides two implementations for streaming compressed video frames from a ROS topic to a remote receiver using GStreamer. The first version (ros_vid_trans.cpp) is a CPU-only implementation, while the second is a CPU-GPU hybrid leveraging NVIDIA's hardware encoding/decoding with the NVCodec SDK for improved performance. Both versions process JPEG frames from a ROS topic, scale them, encode to H.264, and stream via RTP/UDP. The CPU-only version achieves ~10 FPS on a tested Ubuntu system, while the CPU-GPU hybrid significantly improves performance with hardware acceleration.
Overview
The pipeline subscribes to a ROS topic (sensor_msgs/CompressedImage), processes JPEG frames, scales them to 640x480, encodes to H.264, and streams to a remote IP via RTP/UDP. It includes frame hashing for content analysis, queue management to handle frame drops, and GStreamer probes for debugging. The CPU-GPU hybrid version uses NVIDIA's nvv4l2decoder and nvv4l2h264enc for hardware-accelerated decoding and encoding, reducing CPU load.
Key Features

Video Streaming: Streams JPEG frames from a ROS topic to a remote receiver.
Frame Processing: Scales frames to 640x480 and encodes to H.264.
Performance:
CPU-Only: ~10 FPS on Ubuntu, suitable for lightweight systems.
CPU-GPU Hybrid: Higher FPS with NVCodec SDK, leveraging NVIDIA GPU hardware.


Frame Analysis: Computes content hashes to detect identical frames and logs stats.
GStreamer Pipeline: Uses appsrc, jpegparse, jpegdec, videoscale, x264enc (or nvv4l2h264enc in hybrid), and udpsink.
ROS Integration: Subscribes to sensor_msgs/CompressedImage topic.

Prerequisites

ROS: Tested on Ubuntu with ROS Noetic or later.
GStreamer: Version 1.0 or later with plugins (core, base, good, bad, ugly, libav).
NVCodec SDK (Hybrid only): For NVIDIA hardware encoding/decoding.
NVIDIA GPU (Hybrid only): With compatible drivers and CUDA.
Dependencies: libgstreamer1.0-dev, libgstreamer-plugins-base1.0-dev, ros-<distro>-sensor-msgs.
Operating System: Tested on Ubuntu (e.g., Ubuntu 20.04).

Installation

Install ROS:
Follow instructions for your ROS distribution (e.g., ROS Noetic).


Install GStreamer:sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav


Install NVCodec SDK (Hybrid only):
Install NVIDIA Video Codec SDK and GStreamer nvcodec plugin:

sudo apt-get install gstreamer1.0-nvcodec


Set Up Project:
Copy ros_vid_trans.cpp
For the hybrid version, modify the pipeline to use nvv4l2decoder and nvv4l2h264enc.


Compile:catkin_make --source <your_ros_workspace>

Ensure CMakeLists.txt includes GStreamer and ROS dependencies.

Usage

Set Parameters:
Edit ROS parameters in main():nh.param<std::string>("receiver_ip", receiver_ip, "10.72.99.127");
nh.param<std::string>("image_topic", image_topic, "/autoscan/Station_186/MDH_01/cam1/Compressed");
nh.param<int>("framerate", framerate, 10);
nh.param<int>("bitrate", bitrate, 2000000);


For the hybrid version, modify the pipeline:GstElement *jpegdec = gst_element_factory_make("nvv4l2decoder", "decoder");
GstElement *x264enc = gst_element_factory_make("nvv4l2h264enc", "encoder");




Run:
Source your ROS workspace:source <your_ros_workspace>/devel/setup.bash


Launch the node:rosrun <your_package> video_streamer


Press Ctrl+C to stop.


Output:
Streams H.264 video to the specified receiver_ip (port 5000).
Logs frame stats, content hashes, and GStreamer pipeline status.



Performance

CPU-Only: Achieves ~10 FPS on Ubuntu, limited by software JPEG decoding and H.264 encoding.
CPU-GPU Hybrid: Leverages NVIDIA hardware for decoding (nvv4l2decoder) and encoding (nvv4l2h264enc), achieving requried FPS with minimal CPU usage.

Code Structure

FrameData Struct: Stores frame data, timestamp, ID, and content hash.
Frame Pusher Thread: Manages frame queue and pushes frames to appsrc at target framerate.
GStreamer Pipeline: Chains appsrc ! jpegparse ! jpegdec (or nvv4l2decoder) ! queue ! videoscale ! capsfilter ! x264enc (or nvv4l2h264enc) ! rtph264pay ! udpsink.
Probes: Monitor buffers at jpegparse, jpegdec, and capsfilter for debugging.
ROS Callback: Subscribes to sensor_msgs/CompressedImage and queues frames.

Limitations

CPU-Only: High CPU usage due to software decoding/encoding.
Hybrid: Requires NVIDIA GPU and NVCodec SDK, not portable to non-NVIDIA systems.
Fixed Resolution: Hardcoded input (4208x3120) and output (640x480) resolutions.
No Output Saving: Streams to UDP but does not save locally.

Future Improvements

Dynamic Resolutions: Support configurable input/output resolutions.
Local Recording: Add tee and filesink to save the stream.
Error Handling: Improve robustness for format mismatches or network issues.
Hybrid Optimization: Tune NVCodec parameters for maximum FPS.

Troubleshooting

Pipeline Failure: Verify GStreamer plugins and ROS topic.
Low FPS: Check CPU/GPU load; for hybrid, ensure nvv4l2* elements are used.
Format Mismatch: Confirm image_format matches the ROS topic.

License
Provided as-is for educational purposes. Ensure compliance with ROS, GStreamer, and NVCodec licenses.
