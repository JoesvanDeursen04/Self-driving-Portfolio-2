# Duckiebot Localization and Mapping System

A complete ROS-based autonomous localization and mapping system for Duckiebot that combines wheel odometry, vision-based SLAM, and sensor fusion using an Extended Kalman Filter.

## Overview

This system provides real-time pose estimation and environmental mapping for autonomous Duckiebot navigation. It integrates three independent ROS nodes:

1. **Odometry Node** - Estimates robot position from wheel encoder data
2. **SLAM Node** - Detects visual features and estimates camera motion
3. **Sensor Fusion Node** - Fuses odometry and SLAM measurements using EKF

## System Architecture

```
Duckiebot Hardware
├── Left/Right Wheel Encoders
├── Monocular Camera (1280×720)
└── Motor Drivers

ROS Nodes
├── dbot_odometry/odometry_node.py
│   └── Subscribes: /duckiebot/left_wheel_encoder, /duckiebot/right_wheel_encoder
│   └── Publishes: /odometry, /pose
│
├── dbot_slam/slam_node.py
│   └── Subscribes: /duckiebot/camera/image_raw
│   └── Publishes: /slam/camera_motion, /slam/features, /slam/feature_visualization
│
└── dbot_sensor_fusion/sensor_fusion_node.py
    └── Subscribes: /odometry, /slam/camera_motion
    └── Publishes: /fused_pose, /fused_odometry
```

## Packages

### dbot_odometry
Implements differential drive kinematics for Duckiebot.

**Key Features:**
- Encoder tick to distance conversion
- Differential drive kinematic model
- TF transform broadcasting
- 30 Hz pose update rate

**Topics:**
- **Subscribes:** `/duckiebot/left_wheel_encoder`, `/duckiebot/right_wheel_encoder` (Int32)
- **Publishes:** `/odometry` (Odometry), `/pose` (PoseStamped)

### dbot_slam
Vision-based monocular SLAM using ORB features.

**Key Features:**
- ORB feature detection and descriptor computation
- Feature tracking across frames
- Essential matrix computation via RANSAC
- Camera motion estimation
- 3D feature map building

**Topics:**
- **Subscribes:** `/duckiebot/camera/image_raw` (Image)
- **Publishes:** 
  - `/slam/camera_motion` (PoseStamped)
  - `/slam/features` (PointCloud2)
  - `/slam/feature_visualization` (Image)

### dbot_sensor_fusion
Extended Kalman Filter for sensor fusion.

**Key Features:**
- 3-state EKF (x, y, θ)
- Asymmetric measurement noise modeling
- Process noise integration
- Covariance tracking

**Topics:**
- **Subscribes:** `/odometry` (Odometry), `/slam/camera_motion` (PoseStamped)
- **Publishes:** `/fused_pose` (PoseStamped), `/fused_odometry` (Odometry)

## Installation

### Prerequisites

- ROS Melodic or later
- Python 3
- OpenCV 4.x
- NumPy, SciPy
- Duckietown SDK (optional, for actual hardware)

### Build

```bash
# Clone repository
git clone <repo-url> ~/catkin_ws/src/localization

# Build packages
cd ~/catkin_ws
catkin_make

# Source setup
source devel/setup.bash
```

### Dependencies

Python packages (see `dependencies-py3.txt`):
```
numpy>=1.19.0
scipy>=1.5.0
opencv-contrib-python>=4.4.0
rospy>=1.14.0
```

## Usage

### Launch All Components

Launch the complete localization and mapping system:

```bash
roslaunch dbot_odometry all.launch robot_name:=duckiebot robot_id:=00
```

### Launch Individual Components

```bash
# Terminal 1: Odometry
roslaunch dbot_odometry odometry.launch

# Terminal 2: SLAM
roslaunch dbot_slam slam.launch

# Terminal 3: Sensor Fusion
roslaunch dbot_sensor_fusion fusion.launch
```

### Visualization

Visualize odometry, pose estimates, and feature maps in RViz:

```bash
rviz -d $(rospack find dbot_odometry)/rviz/localization.rviz
```

Or launch with RViz enabled:

```bash
roslaunch dbot_odometry all.launch launch_rviz:=true
```

## Configuration

### Robot Parameters

Duckiebot physical parameters are defined in launch files:

```xml
<!-- dbot_odometry/launch/odometry.launch -->
<param name="wheel_radius" value="0.02"/>          <!-- 2 cm -->
<param name="wheelbase" value="0.1"/>              <!-- 10 cm -->
<param name="encoder_ticks_per_rev" value="135"/>
<param name="update_rate" value="30"/>             <!-- Hz -->
```

### Camera Calibration

Camera intrinsic matrix parameters in SLAM launch file:

```xml
<!-- dbot_slam/launch/slam.launch -->
<param name="focal_length_x" value="183"/>    <!-- fx -->
<param name="focal_length_y" value="183"/>    <!-- fy -->
<param name="principal_point_x" value="160"/> <!-- cx -->
<param name="principal_point_y" value="120"/> <!-- cy -->
```

### Filter Parameters

Edit EKF parameters in `sensor_fusion_node.py`:

```python
# Measurement noise - odometry (lower = more trust)
self.R_odom = np.array([[0.05, 0, 0], ...])

# Measurement noise - vision (higher = less trust)
self.R_vision = np.array([[0.2, 0, 0], ...])

# Process noise
self.Q = np.array([[0.01, 0, 0], ...])
```

## Algorithm Details

### Odometry (Differential Drive Kinematics)

$$v = \frac{v_L + v_R}{2}, \quad \omega = \frac{v_R - v_L}{L}$$

$$\Delta x = v \cos(\theta) \Delta t, \quad \Delta y = v \sin(\theta) \Delta t, \quad \Delta \theta = \omega \Delta t$$

### SLAM (Feature-based Monocular Vision)

1. **Detection:** ORB keypoints (200 max)
2. **Matching:** BFMatcher with Lowe's ratio test (0.7)
3. **Motion Estimation:** Fundamental matrix → Essential matrix → Camera pose
4. **Depth:** Simple triangulation with assumed 0.5m baseline

### Sensor Fusion (Extended Kalman Filter)

**State:** $\mathbf{x} = [x, y, \theta]^T$

**Update:** $\mathbf{K} = \mathbf{P}(\mathbf{P} + \mathbf{R})^{-1}$
$$\mathbf{x} \leftarrow \mathbf{x} + \mathbf{K}(\mathbf{z} - \mathbf{x})$$
$$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K})\mathbf{P}$$

## Testing and Validation

### Manual Testing

1. **Odometry Only**
   ```bash
   # Record rosbag
   rosbag record /odometry
   
   # Drive robot in known pattern (e.g., square)
   # Verify final pose matches expected location
   ```

2. **SLAM Visualization**
   - Monitor `/slam/feature_visualization` in RViz
   - Verify features are tracked across frames
   - Check feature map grows with exploration

3. **Sensor Fusion**
   - Compare `/odometry` vs `/fused_pose` vs `/fused_odometry`
   - Monitor covariance growth over time
   - Verify fusion reduces cumulative drift

### Expected Performance

- **Odometry drift:** ~5-10% of distance traveled (typical encoder)
- **SLAM accuracy:** ±0.1-0.2m in small environments
- **Fused estimate:** Better than odometry alone, handles sensor failure
- **Update rate:** 30 Hz real-time on Duckiebot Jetson TX2

## Limitations

1. **Wheel Slip:** Encoders assume no slip
2. **Scale Ambiguity:** Monocular vision cannot determine absolute scale
3. **Texture Dependency:** Low-texture environments yield sparse features
4. **Convergence Time:** EKF requires time to converge
5. **No Loop Closure:** Cannot detect revisited locations

See [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md) for detailed limitations and failure scenarios.

## Debugging

### Common Issues

**Odometry node not receiving encoder data:**
```bash
# Check encoder topics
rostopic list | grep encoder
rostopic echo /duckiebot/left_wheel_encoder
```

**SLAM node not detecting features:**
```bash
# Check camera topic and image quality
rostopic echo /duckiebot/camera/image_raw
# Verify well-lit, textured environment
```

**Sensor fusion not running:**
```bash
# Check both odometry and SLAM are publishing
rostopic echo /odometry
rostopic echo /slam/camera_motion
```

### Debug Logging

Enable debug output:
```bash
rosparam set rospy_log_level rosgraph_msgs/Log DEBUG
```

Monitor system:
```bash
rqt_graph    # Node and topic graph
rqt_console  # Log messages
```

## File Structure

```
dbot_localization/
├── dbot_odometry/
│   ├── src/odometry_node.py         # Main odometry implementation
│   ├── launch/odometry.launch       # Launch configuration
│   ├── launch/all.launch            # Master launch file
│   ├── rviz/localization.rviz       # RViz configuration
│   ├── CMakeLists.txt
│   └── package.xml
│
├── dbot_slam/
│   ├── src/slam_node.py             # SLAM implementation
│   ├── launch/slam.launch           # Launch configuration
│   ├── CMakeLists.txt
│   └── package.xml
│
├── dbot_sensor_fusion/
│   ├── src/sensor_fusion_node.py    # EKF implementation
│   ├── launch/fusion.launch         # Launch configuration
│   ├── CMakeLists.txt
│   └── package.xml
│
├── TECHNICAL_REPORT.md              # Detailed technical documentation
└── README.md                        # This file
```

## Contributing

To extend this system:

1. **Add IMU fusion:** Modify sensor_fusion_node.py to include gyroscope
2. **Implement loop closure:** Add image hashing for place recognition
3. **Stereo SLAM:** Replace monocular with stereo camera
4. **Occupancy grid:** Accumulate sensor data into probabilistic map

## References

- Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics
- Klein, G., & Murray, D. (2007). Parallel Tracking and Mapping for Small AR Workspaces
- Rublee, E., et al. (2011). ORB: An Efficient Alternative to SIFT or SURF

## License

See LICENSE.pdf in root directory.

## Contact

For questions or issues:
- Check [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md) for detailed architecture
- Review ROS node source code comments
- Consult `rqt_graph` and `rostopic` tools for system debugging

---

**Last Updated:** May 2026  
**System Version:** 1.0  
**ROS Version:** Melodic/Noetic compatible
