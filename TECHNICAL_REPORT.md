# Duckiebot Localization and Mapping System
## Technical Report

### Executive Summary

This report documents the design and implementation of an autonomous localization and mapping system for a Duckiebot robot platform. The system combines four independent ROS nodes that collaboratively estimate the robot's pose while building a map of environmental features. The solution integrates odometry estimation from wheel encoders with vision-based SLAM using a monocular camera, fused through an Extended Kalman Filter for robust position tracking, and a semantic perception layer that maps AprilTags and duckie detections.

### 1. System Architecture

#### 1.1 Overview

The system consists of four main ROS packages communicating through standardized ROS topics:

```
┌─────────────────────────────────────────────────────────────┐
│                    Duckiebot Platform                        │
│  ┌──────────────┐          ┌──────────────┐                │
│  │ Motor Drivers│          │   Camera     │                │
│  │  (Encoders)  │          │   (1280x720) │                │
│  └──────┬───────┘          └──────┬───────┘                │
│         │                         │                         │
│    /encoder_data             /camera_image                  │
└──────┬──────────────────────────┬──────────────────────────┘
       │                          │
       ▼                          ▼
┌────────────────────────────────────────────────────────────┐
│                     ROS Node Layer                          │
│                                                             │
│  ┌─────────────────┐    ┌──────────────────┐              │
│  │ Odometry Node   │    │   SLAM Node      │              │
│  │ (Kinematic      │    │  (Visual Feature │              │
│  │  Estimation)    │    │   Tracking)      │              │
│  └────────┬────────┘    └────────┬─────────┘              │
│           │                      │                         │
│           └──────────────┬───────┘                         │
│                          ▼                                  │
│           ┌──────────────────────────┐                     │
│           │  Sensor Fusion Node      │                     │
│           │  (Extended Kalman Filter)│                     │
│           └──────────────┬───────────┘                     │
│                          │                                  │
│                   /fused_pose                              │
└──────────────────────┬─────────────────────────────────────┘
                       │
                       ▼
              ┌──────────────────┐
              │  RViz Visualization
              │  & Navigation    │
              └──────────────────┘
```

The semantic perception node runs alongside this geometric stack and publishes AprilTag and duckie landmarks into the same odom-referenced world model.

#### 1.2 Component Responsibilities

**Odometry Node (dbot_odometry)**
- Integrates wheel encoder data from left and right motors
- Implements differential drive kinematic model
- Maintains continuous pose estimation (x, y, θ)
- Publishes at 30 Hz on `/odometry` topic
- Broadcast TF transforms for visualization

**SLAM Node (dbot_slam)**
- Captures images from monocular camera
- Detects local features using ORB (Oriented FAST and Rotated BRIEF)
- Tracks features across consecutive frames
- Estimates camera motion via essential matrix decomposition
- Builds sparse 3D feature map
- Publishes feature visualization and camera pose

**Sensor Fusion Node (dbot_sensor_fusion)**
- Subscribes to odometry and vision measurements
- Implements Extended Kalman Filter for pose fusion
- Manages measurement and process noise covariances
- Publishes fused pose estimate and uncertainty bounds
- Provides ground truth for navigation subsystems

**Semantic Perception Node (dbot_semantics)**
- Detects AprilTags for fixed infrastructure such as signs and lights
- Runs an ONNX-based duckie detector for semantic object landmarks
- Projects detections into the odom frame using the fused pose estimate
- Publishes landmark markers and a debug image stream for live visualization

### 2. Implementation Details

#### 2.1 Odometry Implementation

The odometry model employs differential drive kinematics for a two-wheeled robot:

$$v = \frac{v_L + v_R}{2}$$

$$\omega = \frac{v_R - v_L}{L}$$

Where:
- $v$ = linear velocity
- $\omega$ = angular velocity
- $v_L$, $v_R$ = left and right wheel velocities
- $L$ = wheelbase (distance between wheels)

Pose integration uses:

$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t$$

$$y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t$$

$$\theta_{k+1} = \theta_k + \omega_k \Delta t$$

**Key Features:**
- Encoder tick conversion to linear distance via wheel circumference
- Radius of curvature handling for curved trajectories
- Angle normalization to [-π, π] range
- 30 Hz update rate for responsive tracking

**Design Choices:**
- Encoder-based differentiation avoids velocity estimation complexity
- Kinematic model (not dynamic) sufficient for low-speed Duckiebot operation
- Real-time performance achievable with simple linear algebra

#### 2.2 Vision-based SLAM

The SLAM node employs monocular feature-based SLAM:

**Feature Detection:**
- ORB detector: Fast, scale and rotation-invariant
- Non-maximum suppression for well-distributed features
- 200 feature maximum for computational efficiency

**Feature Matching:**
- BFMatcher (Brute Force) with Hamming distance
- Lowe's ratio test (0.7 threshold) for outlier rejection
- Cross-frame feature correlation tracking

**Motion Estimation:**
- Fundamental matrix via RANSAC (robust to outliers)
- Essential matrix decomposition: $E = K^T F K$
- Camera pose recovery via `cv2.recoverPose()`
- 4-point triangulation for depth estimation

**Map Building:**
- Persistent feature tracking across frames
- Triangulation to simple 3D points using assumed depth
- Feature ID association for temporal consistency
- PointCloud2 publication for RViz visualization

**Design Choices:**
- ORB chosen for low computational cost vs SIFT/SURF
- Monocular vision limits absolute scale (scale ambiguity)
- Assumed depth = 0.5m (can be refined with better triangulation)
- Frame-to-frame tracking avoids loop closure complexity

#### 2.3 Extended Kalman Filter (EKF)

The sensor fusion uses a 3-state EKF combining two measurement sources:

**State Vector:** $\mathbf{x} = [x, y, \theta]^T$

**Prediction Step (No Motion Model):**
$$\mathbf{P}_{k|k-1} = \mathbf{P}_{k-1} + \mathbf{Q}$$

Uncertainty increases due to process noise $\mathbf{Q}$ representing model uncertainty.

**Update Steps:**

For measurement $\mathbf{z}$:

**Innovation:** $\mathbf{y} = \mathbf{z} - \mathbf{x}$

**Innovation Covariance:** $\mathbf{S} = \mathbf{P} + \mathbf{R}$

**Kalman Gain:** $\mathbf{K} = \mathbf{P} \mathbf{S}^{-1}$

**State Update:** $\mathbf{x} \leftarrow \mathbf{x} + \mathbf{K} \mathbf{y}$

**Covariance Update:** $\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}) \mathbf{P}$

**Noise Covariances:**

Odometry measurement noise (higher trust):
$$\mathbf{R}_{\text{odom}} = \text{diag}(0.05, 0.05, 0.05)$$

Vision measurement noise (lower trust):
$$\mathbf{R}_{\text{vision}} = \text{diag}(0.2, 0.2, 0.2)$$

Process noise:
$$\mathbf{Q} = \text{diag}(0.01, 0.01, 0.01)$$

**Design Choices:**
- Linear EKF sufficient for 2D localization
- Asymmetric measurement noise reflects relative reliability
- No explicit motion command input (passive fusion only)
- Scalable to higher-dimensional state if extended

#### 2.4 Semantic Perception

The semantic perception node adds an object-aware mapping layer on top of the geometric localization stack.

**AprilTag Detection:**
- Uses OpenCV's AprilTag support to detect tags placed on signs and lights
- Estimates pose from tag corners using `cv2.solvePnP()`
- Converts each tag into a stable semantic landmark in the odom frame

**Duckie Detection:**
- Loads the provided `best.onnx` model with OpenCV DNN
- Detects duckies as semantic object landmarks
- Estimates range from the detection box height using a pinhole camera approximation

**Map Integration:**
- Landmarks are tracked in a persistent dictionary keyed by type and label
- Detections are published as `MarkerArray` messages for RViz
- A debug image topic overlays tags, boxes, and labels for live verification

### 3. ROS Topic Structure

| Topic | Type | Node Source | Description |
|-------|------|-------------|-------------|
| `/duckiebot/left_wheel_encoder` | Int32 | Motor Driver | Left wheel encoder ticks |
| `/duckiebot/right_wheel_encoder` | Int32 | Motor Driver | Right wheel encoder ticks |
| `/duckiebot/camera/image_raw` | Image | Camera | Raw camera frame (1280×720) |
| `/odometry` | Odometry | Odometry Node | Pose from encoder integration |
| `/pose` | PoseStamped | Odometry Node | Position estimate |
| `/slam/camera_motion` | PoseStamped | SLAM Node | Camera pose from vision |
| `/slam/features` | PointCloud2 | SLAM Node | 3D feature map |
| `/slam/feature_visualization` | Image | SLAM Node | Feature tracking visualization |
| `/fused_pose` | PoseStamped | Fusion Node | Final pose estimate |
| `/fused_odometry` | Odometry | Fusion Node | Fused pose with covariance |
| `/semantic_perception/markers` | MarkerArray | Semantic Perception Node | Semantic landmarks in RViz |
| `/semantic_perception/debug_image` | Image | Semantic Perception Node | Duckie and AprilTag overlay image |

### 4. Design Choices and Trade-offs

| Choice | Rationale | Trade-offs |
|--------|-----------|-----------|
| ORB Features | Low CPU, scale-invariant | Lower accuracy than SIFT/SURF |
| Monocular SLAM | Simplicity, low sensor cost | Scale ambiguity, limited depth |
| Linear EKF | Computational efficiency, stability | Non-linear effects ignored |
| 30 Hz Update | Responsive but stable | May miss fast motions |
| Separate Nodes | Modularity, reusability | Additional IPC overhead |
| Simple Triangulation | Speed | Assumes constant depth |

### 5. Limitations and Failure Scenarios

#### 5.1 Odometry Limitations

1. **Wheel Slip:** Encoders assume no slip; on low-friction surfaces causes drift
2. **Heading Accumulation:** Long traversals accumulate angular error
3. **Unmodeled Dynamics:** Doesn't account for motor acceleration/deceleration
4. **Encoder Noise:** Quantization and sensor noise propagate to pose

#### 5.2 SLAM Limitations

1. **Large Baseline:** Insufficient parallax for accurate triangulation
2. **Low Texture:** Featureless environments (white walls) yield sparse maps
3. **Fast Motion:** Motion blur reduces feature tracking quality
4. **Scale Ambiguity:** Monocular vision cannot determine absolute scale
5. **Initialization:** Requires sufficient motion to initialize depth

#### 5.3 Fusion Limitations

1. **Noise Tuning:** Covariance values require manual tuning for specific robots
2. **Outliers:** Doesn't employ outlier rejection beyond SLAM's RANSAC
3. **Convergence:** Moving average window may propagate errors
4. **No Loop Closure:** Can't detect and correct revisited locations

### 6. Future Improvements

1. **Stereo Vision:** Replace monocular with stereo for metric scale
2. **Loop Closure Detection:** Add image hashing (e.g., ORB-SLAM2 approach)
3. **IMU Integration:** Fuse accelerometer/gyroscope for drift reduction
4. **Adaptive Filtering:** Learn noise covariances from data
5. **Occupancy Grid:** Build probabilistic 2D/3D occupancy map
6. **Multi-hypothesis Tracking:** Handle ambiguous feature matches

### 7. Building and Running

**Build:**
```bash
cd ~/catkin_ws
catkin_make dbot_odometry dbot_slam dbot_sensor_fusion
```

**Launch All Components:**
```bash
roslaunch dbot_odometry all.launch robot_name:=duckiebot robot_id:=00
```

**Individual Launch:**
```bash
roslaunch dbot_odometry odometry.launch
roslaunch dbot_slam slam.launch
roslaunch dbot_sensor_fusion fusion.launch
```

**Visualization:**
```bash
rviz -d $(rospack find dbot_odometry)/rviz/localization.rviz
```

### 8. Conclusion

This system provides a functional framework for autonomous Duckiebot localization and mapping through sensor fusion of complementary modalities. The modular ROS architecture enables independent development and testing of odometry, SLAM, and fusion components. While simplified compared to production systems like ORBSLAM2, the implementation demonstrates core robotics concepts including kinematics, visual feature tracking, and probabilistic filtering. Future work should focus on robust outlier handling, loop closure detection, and validation on diverse Duckietown environments.

---

**Document Version:** 1.0  
**Last Updated:** May 2026  
**Authors:** Student Team
