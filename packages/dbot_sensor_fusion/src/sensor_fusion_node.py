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
        
    def predict(self, dt):
        """
        Prediction step - no motion model update in this simplified version.
        In a more advanced implementation, you would integrate motion commands.
        
        Args:
            dt: Time step
        """
        # Increase uncertainty over time due to process noise
        self.P = self.P + self.Q * dt
        
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


class SensorFusionNode:
    """
    Sensor Fusion node combining odometry and SLAM.
    Uses Extended Kalman Filter for robust pose estimation.
    """
    
    def __init__(self):
        """Initialize sensor fusion node with EKF."""
        rospy.init_node('sensor_fusion_node', anonymous=False)
        
        # Robot parameters
        self.robot_name = rospy.get_param('~robot_name', 'duckiebot')
        self.odom_frame = 'odom'
        self.base_frame = f'{self.robot_name}/base_link'
        
        # Initialize Extended Kalman Filter
        self.ekf = ExtendedKalmanFilter()
        
        # Measurement buffers
        self.odom_buffer = None
        self.vision_buffer = None
        self.last_odom_time = None
        self.last_vision_time = None
        
        # Thread lock for thread-safe access
        self.lock = threading.Lock()
        
        # Update rate
        self.update_rate = rospy.get_param('~update_rate', 30)
        self.dt = 1.0 / self.update_rate
        
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
            self.odom_buffer = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self._quaternion_to_angle(msg.pose.pose.orientation)
            ])
            self.last_odom_time = msg.header.stamp.to_sec()
            
    def vision_callback(self, msg):
        """
        Callback for vision/SLAM measurement.
        
        Args:
            msg: geometry_msgs/PoseStamped
        """
        with self.lock:
            # Extract pose
            self.vision_buffer = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                self._quaternion_to_angle(msg.pose.orientation)
            ])
            self.last_vision_time = msg.header.stamp.to_sec()
            
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
                # Prediction step
                self.ekf.predict(self.dt)
                
                # Update with odometry if available
                if self.odom_buffer is not None:
                    self.ekf.update_odometry(self.odom_buffer)
                    rospy.logdebug("Odometry update applied")
                
                # Update with vision if available
                if self.vision_buffer is not None:
                    self.ekf.update_vision(self.vision_buffer)
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
