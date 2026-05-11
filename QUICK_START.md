# Quick Start Guide - Duckiebot Localization & Mapping

## What's Included

This repository contains a complete autonomous localization and mapping system for Duckiebot composed of three ROS packages:

1. **dbot_odometry** - Wheel encoder-based kinematic odometry
2. **dbot_slam** - Vision-based monocular SLAM with feature tracking
3. **dbot_sensor_fusion** - Extended Kalman Filter fusion node

## Files Overview

### Source Code
- `packages/dbot_odometry/src/odometry_node.py` - Differential drive kinematics
- `packages/dbot_slam/src/slam_node.py` - ORB feature-based SLAM
- `packages/dbot_sensor_fusion/src/sensor_fusion_node.py` - EKF sensor fusion

### Launch Files
- `packages/dbot_odometry/launch/all.launch` - Master launch (all components)
- `packages/dbot_odometry/launch/odometry.launch` - Odometry only
- `packages/dbot_slam/launch/slam.launch` - SLAM only
- `packages/dbot_sensor_fusion/launch/fusion.launch` - Fusion only

### Documentation
- **TECHNICAL_REPORT.md** (2-3 pages) - Complete system architecture and design
- **SYSTEM_README.md** - Detailed usage, installation, and configuration guide
- **HARDWARE_INTEGRATION.md** - Hardware setup and Duckiebot integration
- **QUICK_START.md** - This file

### ROS Configuration
- `Dockerfile` - Updated with project parameters
- `launchers/default.sh` - Launcher script for Docker deployment
- `dependencies-apt.txt` - APT package dependencies
- `dependencies-py3.txt` - Python package dependencies

## Installation (5 minutes)

```bash
# 1. Navigate to catkin workspace
cd ~/catkin_ws

# 2. Build packages
catkin_make

# 3. Source setup
source devel/setup.bash
```

## Quick Test (10 minutes)

### Option A: Simulate with Dummy Data

```bash
# Terminal 1: Start ROS master
roscore

# Terminal 2: Publish dummy encoder data
python3 -c "
import rospy
from std_msgs.msg import Int32
rospy.init_node('dummy_encoder')
left_pub = rospy.Publisher('/duckiebot/left_wheel_encoder', Int32, queue_size=10)
right_pub = rospy.Publisher('/duckiebot/right_wheel_encoder', Int32, queue_size=10)
rate = rospy.Rate(30)
tick = 0
while not rospy.is_shutdown():
    tick += 1
    left_pub.publish(Int32(tick))
    right_pub.publish(Int32(tick))
    rate.sleep()
"

# Terminal 3: Publish dummy camera image
python3 -c "
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

rospy.init_node('dummy_camera')
pub = rospy.Publisher('/duckiebot/camera/image_raw', Image, queue_size=10)
bridge = CvBridge()
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    # Create random image (simulating camera)
    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    # Add checkboard pattern
    img[100:400, 150:550] = cv2.checkershadow(200, 200, img[100:400, 150:550])
    msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    pub.publish(msg)
    rate.sleep()
"

# Terminal 4: Launch system
roslaunch dbot_odometry all.launch robot_name:=duckiebot launch_rviz:=true
```

### Option B: Real Hardware

See [HARDWARE_INTEGRATION.md](HARDWARE_INTEGRATION.md) for complete setup instructions.

```bash
# Single command to start everything
roslaunch dbot_odometry all.launch robot_name:=duckiebot robot_id:=00
```

## Understanding the System

### Data Flow

```
Wheel Encoders          Camera
     ↓                    ↓
  Odometry Node       SLAM Node
     ↓                    ↓
     └────→ Sensor Fusion ←────
            (EKF Filter)
                 ↓
           Fused Pose Estimate
                 ↓
         RViz Visualization
         Navigation Stack
```

### Key ROS Topics

| Topic | Type | Publishes | Frequency |
|-------|------|-----------|-----------|
| `/odometry` | Odometry | Odometry Node | 30 Hz |
| `/slam/camera_motion` | PoseStamped | SLAM Node | ~15 Hz |
| `/fused_pose` | PoseStamped | Fusion Node | 30 Hz |
| `/slam/feature_visualization` | Image | SLAM Node | ~15 Hz |

## Configuration

### Change Update Rate

```xml
<!-- In launch file: -->
<param name="update_rate" value="20"/>  <!-- 20 Hz instead of 30 Hz -->
```

### Calibrate for Your Robot

```xml
<!-- In dbot_odometry/launch/odometry.launch: -->
<param name="wheel_radius" value="0.021"/>      <!-- Your wheel radius in meters -->
<param name="wheelbase" value="0.101"/>         <!-- Distance between wheels -->
<param name="encoder_ticks_per_rev" value="135"/>
```

### Adjust Filter Tuning

```python
# In dbot_sensor_fusion/src/sensor_fusion_node.py:
self.R_odom = np.array([[0.05, 0, 0], ...])   # Odometry trust
self.R_vision = np.array([[0.2, 0, 0], ...])  # Vision trust
self.Q = np.array([[0.01, 0, 0], ...])        # Process noise
```

## Visualization

### RViz (Recommended)

```bash
# Automatically launched with:
roslaunch dbot_odometry all.launch launch_rviz:=true

# Or start manually:
rviz -d $(rospack find dbot_odometry)/rviz/localization.rviz
```

### Command Line Monitoring

```bash
# Watch odometry updates
rostopic echo /odometry

# Monitor fusion results
rostopic echo /fused_pose

# Check SLAM feature map
rostopic echo /slam/features

# View feature visualization
rqt_image_view /slam/feature_visualization
```

## Evaluation

### Manual Verification

1. **Drive in square pattern** (1m × 1m)
   - Expected: Final position within ±10cm of start
   - Check in RViz or echo `/fused_pose`

2. **Check feature tracking**
   - Monitor `/slam/feature_visualization`
   - Should see green points for matched features

3. **Compare measurements**
   ```bash
   rostopic echo -n 100 /odometry > odometry.txt
   rostopic echo -n 100 /fused_pose > fused_pose.txt
   # Fused pose should be smoother than odometry
   ```

### Automated Testing

```bash
# Record system performance
rosbag record -a -O performance.bag

# Run analysis script
python3 scripts/evaluate_mapping.py performance.bag
```

## Troubleshooting

### "No module named 'cv_bridge'"
```bash
sudo apt-get install ros-noetic-cv-bridge
```

### Topics not publishing
```bash
# Check all active nodes
rosnode list

# Check specific topic
rostopic list | grep odometry
rostopic hz /odometry  # Should show 30 Hz
```

### RViz shows no TF frames
```bash
# Verify TF broadcasting
rostopic echo /tf
# Should see odom → base_link transform
```

### High CPU usage
- Reduce update rate: `update_rate: 15`
- Reduce max features: `max_features: 100`
- Limit image size in camera driver

## Next Steps

1. **Test on Hardware:** Follow [HARDWARE_INTEGRATION.md](HARDWARE_INTEGRATION.md)
2. **Understand Design:** Read [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md)
3. **Tune Parameters:** See [SYSTEM_README.md](SYSTEM_README.md)
4. **Extend System:** Add IMU fusion, loop closure, or occupancy grid

## Key Concepts

### Odometry
- Integrates wheel encoder ticks using differential drive kinematics
- Drifts over time due to wheel slip and quantization
- Fast (~1ms latency) but unreliable long-term

### SLAM
- Detects and tracks visual features (ORB keypoints)
- Estimates camera motion from feature matches
- Can be noisy but handles long-term drift
- Limited by monocular scale ambiguity

### Sensor Fusion (EKF)
- Combines strengths of both modalities
- Uses Kalman filter to optimally weight measurements
- Reduces drift while maintaining responsiveness
- Sweet spot between accuracy and latency

## System Specifications

| Component | Property | Value |
|-----------|----------|-------|
| Odometry | Update Rate | 30 Hz |
| Odometry | Latency | ~1 ms |
| SLAM | Update Rate | ~15 Hz |
| SLAM | Feature Detector | ORB (200 max) |
| SLAM | Matcher | BFMatcher, Lowe's ratio |
| Fusion | Filter Type | Extended Kalman Filter |
| Fusion | State Dimension | 3 (x, y, θ) |
| Fusion | Update Rate | 30 Hz |

## References

For more information:
- See `TECHNICAL_REPORT.md` for complete algorithm descriptions
- See `SYSTEM_README.md` for detailed configuration and usage
- See `HARDWARE_INTEGRATION.md` for Duckiebot-specific setup

## Support

**Common Questions:**

Q: How do I integrate my IMU?  
A: See "IMU Integration" in HARDWARE_INTEGRATION.md

Q: How do I improve mapping accuracy?  
A: Improve lighting, add textured patterns, increase camera resolution

Q: Can the system detect when it revisits a location?  
A: Not yet - loop closure is listed as future improvement

Q: What's the estimated cost of implementation?  
A: Uses only standard ROS packages, free and open source

---

**Version:** 1.0  
**Last Updated:** May 2026  
**Files:** 23 total (3 packages, 7 docs, 12+ support files)
