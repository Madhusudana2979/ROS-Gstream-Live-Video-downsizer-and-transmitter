# ROS GStreamer Live Video Downsizer and Transmitter

A high-performance, ultra-low latency ROS package for capturing, processing, and streaming compressed video over UDP on NVIDIA Jetson Xavier NX. This package leverages hardware-accelerated JPEG decoding, intelligent frame downsampling, and optimized H.264 encoding for real-time video transmission.

## Features

- **Hardware-Accelerated Processing**: Utilizes NVIDIA's hardware JPEG decoder and buffer surface transformations
- **Intelligent Frame Filtering**: Drops identical consecutive frames to reduce bandwidth
- **Ultra-Low Latency**: Optimized pipeline with zero-latency encoding settings
- **Adaptive Frame Rate Control**: Strict framerate enforcement with timer-based processing
- **CPU Affinity**: Pins process to dedicated CPU core for consistent performance
- **Resolution Downsampling**: Converts input images to 640x480 for efficient streaming

## System Requirements

### Hardware
- NVIDIA Jetson Xavier NX
- Network connection for UDP streaming

### Software Dependencies
- Ubuntu 18.04/20.04 (JetPack)
- ROS (Noetic/Melodic)
- GStreamer 1.14.5 or newer
- NVIDIA Multimedia API
- Required libraries:
  - `libnvjpeg`
  - `libnvbufsurface`
  - `libnvbufsurftransform`
  - `gstreamer1.0-plugins-base`
  - `gstreamer1.0-plugins-good`
  - `gstreamer1.0-plugins-bad`
  - `gstreamer1.0-x264`

## Installation

### 1. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-sensor-msgs \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-x264 \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```



### 2. Link NVIDIA Multimedia Libraries

Ensure the following libraries are in your library path:
```bash
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH
```

## Configuration

The package uses ROS parameters for configuration. Create a launch file or set parameters via command line.

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_topic` | string | `/test/Station/MDH_01/cam5/Compressed` | Input ROS topic for compressed images |
| `image_format` | string | `yuv420` | Output image format (currently supports yuv420) |
| `receiver_ip` | string | `10.72.99.127` | IP address of the receiving machine |
| `output_width` | int | `640` | Output video width in pixels |
| `output_height` | int | `480` | Output video height in pixels |
| `framerate` | int | `10` | Target streaming framerate (fps) |
| `bitrate` | int | `1000` | H.264 encoding bitrate in kbps |


## How It Works

### Pipeline Architecture

1. **Frame Reception**: Subscribes to ROS compressed image topic
2. **Queue Management**: Maintains only the latest frame to minimize latency
3. **Timer-Based Processing**: Processes frames at exact intervals (1/framerate seconds)
4. **Hardware Decoding**: Decodes JPEG using NVIDIA hardware decoder
5. **Surface Transformation**: Downsamples to target resolution using bilinear interpolation
6. **Duplicate Detection**: Compares with previous frame pixel-by-pixel
7. **GStreamer Pipeline**: Encodes and streams via:
   - `appsrc` → `x264enc` → `rtph264pay` → `udpsink`

### Key Optimizations

- **CPU Affinity**: Process pinned to CPU core 1 for deterministic performance
- **Zero-Latency Encoding**: x264enc configured with `tune=zerolatency`, `speed-preset=ultrafast`
- **Frame Dropping**: Skips identical frames to save bandwidth
- **Non-Blocking Pipeline**: Asynchronous buffer handling prevents backpressure
- **Buffer Management**: Monitors appsrc buffer level to prevent overflow
## Troubleshooting

### Issue: "Failed to create GStreamer elements"
- Ensure all GStreamer plugins are installed
- Check that x264enc plugin is available: `gst-inspect-1.0 x264enc`

### Issue: "Failed to set CPU affinity"
- This is a warning and won't prevent operation
- Requires elevated privileges: run with `sudo` if needed

### Issue: High latency or frame drops
- Reduce `bitrate` parameter (try 500-800 kbps)
- Lower `framerate` parameter
- Check network bandwidth and quality
- Verify receiver is keeping up with stream

### Issue: NVIDIA decoder errors
- Ensure JetPack is properly installed
- Check that `/usr/lib/aarch64-linux-gnu/tegra` libraries are accessible
- Verify CUDA and multimedia APIs are configured

### Issue: No frames received
- Confirm the input topic is publishing: `rostopic hz <topic_name>`
- Check image format is JPEG compressed
- Verify network connectivity to receiver IP

## Performance Tips


- NVIDIA Multimedia API documentation
- GStreamer community
- ROS community
