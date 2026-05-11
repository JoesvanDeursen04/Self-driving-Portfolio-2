# Duckiebot Hardware Integration Guide

This guide explains how to integrate the localization and mapping system with actual Duckiebot hardware.

## Hardware Requirements

- **Duckiebot:** DB-19 or later model
- **Processor:** Jetson Nano or TX2 (4GB+ RAM)
- **Sensors:**
  - Monocular camera (1280×720 @ 30fps)
  - Left/Right wheel encoders
- **Network:** Ethernet or WiFi for rosmaster communication

## Encoder Integration

### Wheel Encoder Topics

The odometry node expects encoder data on:
- `/duckiebot_name/left_wheel_encoder` - Int32 message (tick count)
- `/duckiebot_name/right_wheel_encoder` - Int32 message (tick count)

### Publishing Encoder Data

Create a driver node to publish encoder ticks from wheel motors:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

class EncoderPublisher:
    def __init__(self):
        self.left_pub = rospy.Publisher(
            f'/{ROBOT_NAME}/left_wheel_encoder',
            Int32,
            queue_size=10
        )
        self.right_pub = rospy.Publisher(
            f'/{ROBOT_NAME}/right_wheel_encoder',
            Int32,
            queue_size=10
        )
        
    def publish_encoders(self, left_ticks, right_ticks):
        self.left_pub.publish(Int32(left_ticks))
        self.right_pub.publish(Int32(right_ticks))
```

### Duckiebot Motor Hardware

Encoders are typically available from:
- Motor driver firmware (e.g., HEV, Teensy-based controllers)
- DuckiePower board accumulator
- Custom encoder reader via GPIO

### Calibration

Determine your robot's exact parameters:

```bash
# 1. Measure wheel radius
wheel_diameter = (measured circumference) / π

# 2. Measure wheelbase
wheelbase = (distance between wheel centers)

# 3. Measure encoder resolution
# Drive wheel one full revolution, count encoder ticks
encoder_ticks_per_rev = (ticks for one revolution)

# Update in odometry.launch:
<param name="wheel_radius" value="0.0207"/>
<param name="wheelbase" value="0.1015"/>
<param name="encoder_ticks_per_rev" value="135"/>
```

## Camera Integration

### Camera Topic Setup

The SLAM node subscribes to:
- `/duckiebot_name/camera/image_raw` - sensor_msgs/Image (BGR8)

### Duckiebot Camera Driver

Use the official Duckietown camera driver:

```bash
# Install camera calibration package
pip install -e /path/to/dt-camera-calibration

# Launch camera node
roslaunch duckiebot_description camera.launch veh:=$ROBOT_NAME
```

### Camera Calibration

Obtain camera intrinsic matrix from calibration:

```bash
# 1. Run calibration procedure
rosrun camera_calibration cameracalibrator.py \
    --size=8x6 --square=0.108 \
    camera:=/duckiebot_name/camera

# 2. Extract calibration parameters
# Save to ~/.ros/camera_calibration/

# 3. Update slam.launch:
<param name="focal_length_x" value="183.456"/>
<param name="focal_length_y" value="182.123"/>
<param name="principal_point_x" value="159.789"/>
<param name="principal_point_y" value="119.456"/>
```

## Launch Procedure

### 1. Start ROS Master on Robot

```bash
# On Duckiebot (via SSH)
export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311
roscore
```

### 2. Start Hardware Drivers

```bash
# Terminal 1: Motor and encoder publishers
docker exec -it duckiebot-container \
    roslaunch duckiebot_description motor_driver.launch \
    veh:=duckiebot00

# Terminal 2: Camera driver
docker exec -it duckiebot-container \
    roslaunch duckiebot_description camera.launch \
    veh:=duckiebot00
```

### 3. Launch Localization System

```bash
# Terminal 3: Full system
docker exec -it duckiebot-container \
    roslaunch dbot_odometry all.launch \
    robot_name:=duckiebot00 \
    launch_rviz:=true
```

## Testing on Hardware

### Test 1: Encoder Verification

```bash
# In separate terminal
rostopic echo /odometry

# Expected behavior:
# - x, y increase as robot moves
# - theta changes when rotating
# - No values should be NaN
```

### Test 2: Camera Verification

```bash
# Check image stream
rostopic hz /camera/image_raw  # Should show ~30 Hz
rostopic echo -n 1 /camera/image_raw  # Should show valid image data
```

### Test 3: SLAM Feature Detection

Monitor feature tracking:

```bash
# In RViz or separate process
rqt_image_view /slam/feature_visualization

# Expected behavior:
# - Green points mark detected features
# - Yellow keypoints on image corners
# - Matches shown as connecting lines
```

### Test 4: Pose Consistency

Compare odometry, SLAM, and fused estimates:

```bash
# Compare outputs
rostopic echo /odometry
rostopic echo /slam/camera_motion
rostopic echo /fused_pose

# Expected behavior:
# - Odometry drifts over time
# - SLAM may be noisy but unbiased
# - Fused pose should be smooth and accurate
```

### Test 5: Mapping Accuracy

Drive in known patterns and verify convergence:

```bash
# Record rosbag
rosbag record /odometry /fused_odometry /odometry /slam/features

# Drive robot in square pattern (1m x 1m)
# Expected: Final position within ±10cm of start

# Analyze error
python3 scripts/evaluate_mapping.py recorded.bag
```

## ROS Parameter Tuning

### Odometry Parameters

```bash
# Reduce update rate if CPU usage is high
rosparam set /odometry_node/update_rate 15

# Adjust wheelbase if measurements show systematic error
rosparam set /odometry_node/wheelbase 0.102
```

### SLAM Parameters

```bash
# Adjust feature detection sensitivity
rosparam set /slam_node/feature_quality 0.01

# Limit features for faster processing
rosparam set /slam_node/max_features 150
```

### Filter Parameters

```bash
# Change measurement noise if sensor behavior differs
# Lower values = more trust in measurement
rosparam set /sensor_fusion_node/R_odom_x 0.03
rosparam set /sensor_fusion_node/R_vision_x 0.15
```

## Advanced Configuration

### IMU Integration (Optional)

For better heading estimation, add IMU:

1. Publish `/imu/data` (sensor_msgs/Imu)
2. Modify `sensor_fusion_node.py`:

```python
# Add IMU subscriber
rospy.Subscriber('/imu/data', Imu, self.imu_callback)

# Update heading update in filter
self.ekf.x[2] = imu_heading  # Replace SLAM heading estimate
```

### Loop Closure (Optional)

For longer-range mapping, add loop closure detection:

```python
# In slam_node.py, add image hashing
import imagehash
current_hash = imagehash.phash(current_frame)
if hamming_distance(current_hash, old_hash) < threshold:
    # Detected loop closure, trigger pose correction
    self.correct_pose(old_pose)
```

### Occupancy Grid (Optional)

Accumulate SLAM features into 2D occupancy grid:

```bash
# Install gmapping or octomap
sudo apt-get install ros-noetic-gmapping

# Convert sparse features to occupancy map
rosrun gmapping slam_gmapping \
    scan:=/slam/features \
    map:=/occupancy_grid
```

## Docker Deployment

### Build Image for Duckiebot

```bash
# On development machine
docker build -t dbot-localization:latest .

# Push to Docker Hub (optional)
docker tag dbot-localization:latest username/dbot-localization:latest
docker push username/dbot-localization:latest
```

### Run on Hardware

```bash
# On Duckiebot
docker run -it --network=host \
    -v /data/calibrations:/calibrations \
    dbot-localization:latest
```

### Volume Mounts for Data

```bash
# Mount for rosbag recording
docker run -it --network=host \
    -v /tmp/rosbags:/rosbags \
    dbot-localization:latest
```

## Performance Monitoring

### CPU and Memory Usage

```bash
# Monitor resource usage
rqt_top

# Check specific node
top | grep odometry_node
```

### Latency Measurement

```bash
# Check publication frequency
rostopic hz /odometry
rostopic hz /fused_pose

# Expected: 30 Hz for both

# Check end-to-end latency
roswtf
```

### Network Bandwidth

```bash
# Monitor ROS network traffic
rostopic bw /odometry

# Expected: ~2-5 kB/s for odometry
# Expected: ~50-100 kB/s for camera images
```

## Troubleshooting on Hardware

### Encoder Data Missing

```bash
# Check connection
rostopic list | grep encoder

# Verify motor driver publishing
# Contact motor driver firmware team if needed
```

### Camera Not Streaming

```bash
# Check USB connection
lsusb | grep camera

# Restart camera driver
rosnode kill /camera_node
roslaunch duckiebot_description camera.launch
```

### High Odometry Drift

**Symptoms:** Position error > 20% of distance

**Solutions:**
- Verify wheel radius calibration
- Check for wheel slippage (test on non-slippery surface)
- Increase encoder sampling rate
- Add friction measurement to model

### SLAM Features Not Tracking

**Symptoms:** `/slam/feature_visualization` shows few/no points

**Solutions:**
- Improve lighting conditions
- Reduce motion speed (motion blur damages features)
- Pasted AprilTags or QR codes for better tracking
- Increase `feature_quality` parameter

### Sensor Fusion Divergence

**Symptoms:** `/fused_pose` deviates from expected trajectory

**Solutions:**
- Tune measurement noise covariances
- Verify both odometry and SLAM are publishing
- Check for timestamp synchronization issues
- Increase process noise if environment has unexpected forces

## Data Collection and Analysis

### Record System Performance

```bash
# Record all topics
rosbag record -a -O system_performance.bag

# Record specific topics only
rosbag record /odometry /fused_pose /slam/features -O mapping_data.bag
```

### Replay and Analysis

```bash
# Replay rosbag
rosbag play mapping_data.bag

# Analyze in Python
import rosbag
bag = rosbag.Bag('mapping_data.bag')
for topic, msg, t in bag.read_messages(topics=['/odometry']):
    print(f"Position: {msg.pose.pose.position}")
```

### Generate Reports

```bash
# Create evaluation report
python3 scripts/generate_report.py system_performance.bag

# Export to CSV for analysis
rosbag2csv mapping_data.bag -o odometry_data.csv
```

---

**For additional support, refer to:**
- [TECHNICAL_REPORT.md](../TECHNICAL_REPORT.md) - System architecture details
- [SYSTEM_README.md](../SYSTEM_README.md) - Software setup and usage
- Duckietown documentation: https://docs.duckietown.org/
