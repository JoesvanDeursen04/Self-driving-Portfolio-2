#!/usr/bin/env python3

"""
Sensor Fusion Node for Duckiebot
Combines odometry and SLAM using Extended Kalman Filter.

Topics:
    Subscribes:
        - /odometry (nav_msgs/Odometry)
        - /slam/camera_motion (geometry_msgs/PoseStamped)
    Publishes:
        - /fused_pose (geometry_msgs/PoseStamped)
        - /fused_odometry (nav_msgs/Odometry)
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from std_msgs.msg import Header
import numpy as np
import math
import tf.transformations as tf_trans
from scipy.spatial.transform import Rotation as R
import threading
from duckietown.dtros import DTROS, NodeType


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for pose estimation.
    
    State vector: [x, y, theta]
    Measurement 1: odometry [x, y, theta]
    Measurement 2: vision [x, y, theta]
    """
    
    def __init__(self):
        """Initialize EKF with default parameters."""
        # State dimension
        self.dim_state = 3
        self.dim_meas_odom = 3
        self.dim_meas_vision = 3
        
        # State: [x, y, theta]
        self.x = np.array([0.0, 0.0, 0.0])
        
        # State covariance (uncertainty)
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance (model uncertainty)
        self.Q = np.array([
            [0.01, 0, 0],
            [0, 0.01, 0],
            [0, 0, 0.01]
        ])
        
        # Measurement noise covariance for odometry (higher trust in odometry)
        self.R_odom = np.array([
            [0.05, 0, 0],
            [0, 0.05, 0],
            [0, 0, 0.05]
        ])
        
        # Measurement noise covariance for vision (lower trust, more uncertainty)
        self.R_vision = np.array([
            [0.2, 0, 0],
            [0, 0.2, 0],
            [0, 0, 0.2]
        ])
        
    def predict(self, dt, control_v=0.0, control_w=0.0):
        """
        Prediction step with a simple unicycle motion model.
        
        Args:
            dt: Time step
            control_v: Forward speed in m/s
            control_w: Angular speed in rad/s
        """
        theta = float(self.x[2])

        # Predict state using kinematic model.
        self.x[0] = self.x[0] + (control_v * dt * math.cos(theta))
        self.x[1] = self.x[1] + (control_v * dt * math.sin(theta))
        self.x[2] = self._normalize_angle(self.x[2] + (control_w * dt))

        # Linearized Jacobian of the motion model.
        F = np.array([
            [1.0, 0.0, -control_v * dt * math.sin(theta)],
            [0.0, 1.0, control_v * dt * math.cos(theta)],
            [0.0, 0.0, 1.0],
        ])

        # Propagate covariance with process noise.
        self.P = F @ self.P @ F.T + (self.Q * dt)
        
    def update_odometry(self, z_odom):
        """
        Update step using odometry measurement.
        
        Args:
            z_odom: Odometry measurement [x, y, theta]
        """
        # Innovation (measurement residual)
        y = z_odom - self.x
        
        # Normalize angle
        y[2] = self._normalize_angle(y[2])
        
        # Innovation covariance
        S = self.P + self.R_odom
        
        # Kalman gain
        K = self.P @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Update covariance
        self.P = (np.eye(3) - K) @ self.P
        
    def update_vision(self, z_vision):
        """
        Update step using vision/SLAM measurement.
        
        Args:
            z_vision: Vision measurement [x, y, theta]
        """
        # Innovation (measurement residual)
        y = z_vision - self.x
        
        # Normalize angle
        y[2] = self._normalize_angle(y[2])
        
        # Innovation covariance
        S = self.P + self.R_vision
        
        # Kalman gain
        K = self.P @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Update covariance
        self.P = (np.eye(3) - K) @ self.P

    def update_vision_delta(self, delta_vision):
        """
        Update state with relative vision increment instead of absolute pose.

        Args:
            delta_vision: Increment [dx, dy, dtheta]
        """
        # Turn relative motion into a proposal around current state.
        z = self.x + delta_vision
        z[2] = self._normalize_angle(z[2])

        y = z - self.x
        y[2] = self._normalize_angle(y[2])

        S = self.P + self.R_vision
        K = self.P @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = self._normalize_angle(self.x[2])
        self.P = (np.eye(3) - K) @ self.P
        
    def get_pose(self):
        """
        Get current pose state.
        
        Returns:
            State vector [x, y, theta]
        """
        return self.x.copy()
    
    def get_covariance(self):
        """Get state covariance matrix."""
        return self.P.copy()
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class SensorFusionNode(DTROS):
    """
    Sensor Fusion node combining odometry and SLAM.
    Uses Extended Kalman Filter for robust pose estimation.
    """
    
    def __init__(self):
        """Initialize sensor fusion node with EKF."""
        super(SensorFusionNode, self).__init__(node_name='sensor_fusion_node', node_type=NodeType.LOCALIZATION)
        
        # Robot parameters
        self.robot_name = rospy.get_param('~robot_name', 'duckiebot')
        self.odom_frame = 'odom'
        self.base_frame = f'{self.robot_name}/base_link'
        
        # Initialize Extended Kalman Filter
        self.ekf = ExtendedKalmanFilter()
        
        # Measurement buffers
        self.odom_buffer = None
        self.vision_delta_buffer = None
        self.last_odom_time = None
        self.last_vision_time = None
        self.prev_vision_pose = None
        self.last_odom_for_velocity = None

        # Motion model controls estimated from odometry
        self.control_v = 0.0
        self.control_w = 0.0
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 6.0)

        # Vision reset/outlier guards
        self.vision_reset_near_zero_m = rospy.get_param('~vision_reset_near_zero_m', 0.1)
        self.vision_reset_previous_far_m = rospy.get_param('~vision_reset_previous_far_m', 0.5)
        self.max_vision_delta_m = rospy.get_param('~max_vision_delta_m', 0.5)
        
        # Thread lock for thread-safe access
        self.lock = threading.Lock()
        
        # Update rate
        self.update_rate = rospy.get_param('~update_rate', 30)
        self.nominal_dt = 1.0 / self.update_rate
        self.last_predict_time = None
        self.max_predict_dt = rospy.get_param('~max_predict_dt', 0.2)
        
        # Publishers
        self.fused_pose_pub = rospy.Publisher(
            '/fused_pose',
            PoseStamped,
            queue_size=10
        )
        
        self.fused_odom_pub = rospy.Publisher(
            '/fused_odometry',
            Odometry,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber('/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/slam/camera_motion', PoseStamped, self.vision_callback)
        
        rospy.loginfo(f"Sensor Fusion node initialized for {self.robot_name}")
        rospy.loginfo("Extended Kalman Filter ready for sensor fusion")
        
    def odometry_callback(self, msg):
        """
        Callback for odometry measurement.
        
        Args:
            msg: nav_msgs/Odometry
        """
        with self.lock:
            # Extract pose
            odom_pose = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self._quaternion_to_angle(msg.pose.pose.orientation)
            ])
            self.odom_buffer = odom_pose
            self.last_odom_time = msg.header.stamp.to_sec()

            # Estimate controls from odometry increments for EKF prediction.
            if self.last_odom_for_velocity is not None:
                prev_pose, prev_t = self.last_odom_for_velocity
                dt = self.last_odom_time - prev_t
                if dt > 1e-4:
                    dx = odom_pose[0] - prev_pose[0]
                    dy = odom_pose[1] - prev_pose[1]
                    dtheta = self.ekf._normalize_angle(odom_pose[2] - prev_pose[2])
                    forward = dx * math.cos(prev_pose[2]) + dy * math.sin(prev_pose[2])
                    v = forward / dt
                    w = dtheta / dt
                    self.control_v = float(np.clip(v, -self.max_linear_speed, self.max_linear_speed))
                    self.control_w = float(np.clip(w, -self.max_angular_speed, self.max_angular_speed))

            self.last_odom_for_velocity = (odom_pose, self.last_odom_time)
            
    def vision_callback(self, msg):
        """
        Callback for vision/SLAM measurement.
        
        Args:
            msg: geometry_msgs/PoseStamped
        """
        with self.lock:
            # Extract pose and convert to relative increment.
            vision_pose = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                self._quaternion_to_angle(msg.pose.orientation)
            ])
            self.last_vision_time = msg.header.stamp.to_sec()

            if self.prev_vision_pose is None:
                self.prev_vision_pose = vision_pose
                return

            prev_norm = np.linalg.norm(self.prev_vision_pose[:2])
            curr_norm = np.linalg.norm(vision_pose[:2])

            # Detect SLAM restart/relocalization reset and re-anchor safely.
            if curr_norm < self.vision_reset_near_zero_m and prev_norm > self.vision_reset_previous_far_m:
                self.prev_vision_pose = vision_pose
                return

            delta = vision_pose - self.prev_vision_pose
            delta[2] = self.ekf._normalize_angle(delta[2])
            self.prev_vision_pose = vision_pose

            if np.linalg.norm(delta[:2]) > self.max_vision_delta_m:
                return

            self.vision_delta_buffer = delta
            
    def _quaternion_to_angle(self, quaternion):
        """
        Convert quaternion to yaw angle.
        
        Args:
            quaternion: geometry_msgs/Quaternion
            
        Returns:
            Yaw angle in radians
        """
        q = np.array([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ])
        
        # Compute yaw from quaternion
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
        
    def _angle_to_quaternion(self, angle):
        """
        Convert yaw angle to quaternion.
        
        Args:
            angle: Yaw angle in radians
            
        Returns:
            geometry_msgs/Quaternion
        """
        q = tf_trans.quaternion_from_euler(0, 0, angle)
        return Quaternion(q[0], q[1], q[2], q[3])
        
    def publish_fused_pose(self, timestamp):
        """
        Publish fused pose estimate.
        
        Args:
            timestamp: ROS timestamp
        """
        pose = self.ekf.get_pose()
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.odom_frame
        
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = self._angle_to_quaternion(pose[2])
        
        self.fused_pose_pub.publish(pose_msg)
        
    def publish_fused_odometry(self, timestamp):
        """
        Publish fused odometry.
        
        Args:
            timestamp: ROS timestamp
        """
        pose = self.ekf.get_pose()
        covariance = self.ekf.get_covariance()
        
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Set position
        odom_msg.pose.pose.position.x = pose[0]
        odom_msg.pose.pose.position.y = pose[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self._angle_to_quaternion(pose[2])
        
        # Set covariance (position variance)
        odom_msg.pose.covariance = [0] * 36
        odom_msg.pose.covariance[0] = covariance[0, 0]  # x
        odom_msg.pose.covariance[7] = covariance[1, 1]  # y
        odom_msg.pose.covariance[35] = covariance[2, 2]  # theta
        
        self.fused_odom_pub.publish(odom_msg)
        
    def run(self):
        """Main loop - perform EKF prediction and update."""
        rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo("Sensor Fusion running...")
        
        while not rospy.is_shutdown():
            with self.lock:
                now_sec = rospy.Time.now().to_sec()
                if self.last_predict_time is None:
                    dt = self.nominal_dt
                else:
                    dt = now_sec - self.last_predict_time
                    dt = max(1e-4, min(dt, self.max_predict_dt))
                self.last_predict_time = now_sec

                # Prediction step
                self.ekf.predict(dt, self.control_v, self.control_w)
                
                # Update with odometry if available
                if self.odom_buffer is not None:
                    self.ekf.update_odometry(self.odom_buffer)
                    self.odom_buffer = None
                    rospy.logdebug("Odometry update applied")
                
                # Update with vision if available
                if self.vision_delta_buffer is not None:
                    self.ekf.update_vision_delta(self.vision_delta_buffer)
                    self.vision_delta_buffer = None
                    rospy.logdebug("Vision update applied")
                
                # Publish fused estimates
                timestamp = rospy.Time.now()
                self.publish_fused_pose(timestamp)
                self.publish_fused_odometry(timestamp)
                
            rate.sleep()


if __name__ == '__main__':
    try:
        node = SensorFusionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
