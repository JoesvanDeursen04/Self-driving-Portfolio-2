#!/usr/bin/env python3

"""
Odometry Node for Duckiebot
Estimates robot position and orientation using wheel encoder data.
Implements differential drive kinematic model.

Topics:
    Subscribes:
        - /{robot_name}/left_wheel_encoder_node/tick (WheelEncoderStamped)
        - /{robot_name}/right_wheel_encoder_node/tick (WheelEncoderStamped)
    Publishes:
        - /odometry (Odometry)
        - /pose (PoseStamped)
"""

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from duckietown_msgs.msg import WheelEncoderStamped
import tf
import tf2_ros as tf2
import math
import numpy as np
from duckietown.dtros import DTROS, NodeType


class OdometryNode(DTROS):
    """
    Odometry estimation node for differential drive robot.
    
    Estimates robot pose (x, y, theta) by integrating encoder data
    using differential drive kinematics.
    """
    
    def __init__(self):
        """Initialize odometry node with parameters and subscribers."""
        super(OdometryNode, self).__init__(node_name='odometry_node', node_type=NodeType.LOCALIZATION)
        
        # Get robot name
        self.robot_name = rospy.get_param('~robot_name', 'duckiebot')
        
        # Duckiebot parameters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.02)  # 2 cm wheel radius
        self.wheelbase = rospy.get_param('~wheelbase', 0.1)  # Distance between wheels
        self.encoder_ticks_per_revolution = rospy.get_param('~encoder_ticks_per_rev', 135)
        
        # Update rate
        self.update_rate = rospy.get_param('~update_rate', 30)  # Hz
        self.dt = 1.0 / self.update_rate
        
        # Robot state: [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Wheel states
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_encoder_initialized = False
        self.right_encoder_initialized = False
        
        # Frame IDs
        self.odom_frame = 'odom'
        self.base_frame = f'{self.robot_name}/base_link'

        # Odometry covariance (pose uncertainty)
        self.odom_cov_x = rospy.get_param('~odom_cov_x', 0.02)
        self.odom_cov_y = rospy.get_param('~odom_cov_y', 0.02)
        self.odom_cov_yaw = rospy.get_param('~odom_cov_yaw', 0.02)
        
        # Create transform broadcaster
        self.br = tf.TransformBroadcaster()
        
        # Publishers
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        
        # Subscribers
        # Real Duckiebot: dt-duckiebot-interface launches the encoder nodes as
        # left_wheel_encoder_node and right_wheel_encoder_node, publishing on ~tick
        rospy.Subscriber(
            f'/{self.robot_name}/left_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.left_encoder_callback
        )
        rospy.Subscriber(
            f'/{self.robot_name}/right_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.right_encoder_callback
        )
        
        rospy.loginfo(f"Odometry node initialized for {self.robot_name}")
        rospy.loginfo(f"Wheel radius: {self.wheel_radius} m")
        rospy.loginfo(f"Wheelbase: {self.wheelbase} m")
        
    def left_encoder_callback(self, msg):
        """
        Callback for left wheel encoder data.
        
        Args:
            msg: WheelEncoderStamped message containing cumulative tick count
        """
        self.left_encoder_count = msg.data
        if not self.left_encoder_initialized:
            # Use first tick sample as baseline to avoid a startup jump.
            self.left_encoder_prev = msg.data
            self.left_encoder_initialized = True
        if msg.resolution > 0:
            self.encoder_ticks_per_revolution = msg.resolution
        
    def right_encoder_callback(self, msg):
        """
        Callback for right wheel encoder data.
        
        Args:
            msg: WheelEncoderStamped message containing cumulative tick count
        """
        self.right_encoder_count = msg.data
        if not self.right_encoder_initialized:
            # Use first tick sample as baseline to avoid a startup jump.
            self.right_encoder_prev = msg.data
            self.right_encoder_initialized = True
        if msg.resolution > 0:
            self.encoder_ticks_per_revolution = msg.resolution
        
    def encoder_ticks_to_distance(self, ticks):
        """
        Convert encoder ticks to wheel distance traveled.
        
        Args:
            ticks: Number of encoder ticks
            
        Returns:
            Distance in meters
        """
        return (ticks / self.encoder_ticks_per_revolution) * 2 * math.pi * self.wheel_radius
        
    def update_odometry(self):
        """
        Update robot pose using differential drive kinematics.
        Uses encoder data to estimate wheel velocities and robot motion.
        """
        if not (self.left_encoder_initialized and self.right_encoder_initialized):
            return

        # Calculate distance traveled by each wheel
        left_distance = self.encoder_ticks_to_distance(
            self.left_encoder_count - self.left_encoder_prev
        )
        right_distance = self.encoder_ticks_to_distance(
            self.right_encoder_count - self.right_encoder_prev
        )
        
        # Update previous encoder counts
        self.left_encoder_prev = self.left_encoder_count
        self.right_encoder_prev = self.right_encoder_count
        
        # Differential drive kinematics
        # Average distance traveled
        distance = (left_distance + right_distance) / 2.0
        
        # Change in heading (rotation)
        delta_theta = (right_distance - left_distance) / self.wheelbase
        
        # Update heading
        self.theta += delta_theta
        self.theta = self._normalize_angle(self.theta)
        
        # Update position
        if abs(delta_theta) < 1e-6:
            # Going straight
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
        else:
            # Following curved path
            # Use radius of curvature formula
            radius = distance / delta_theta
            self.x += radius * (math.sin(self.theta) - math.sin(self.theta - delta_theta))
            self.y += radius * (math.cos(self.theta - delta_theta) - math.cos(self.theta))
        
    def _normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi].
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle in [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def publish_odometry(self):
        """Publish odometry and pose information."""
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*quaternion)

        # Non-zero covariance is required for downstream fusion/filtering.
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.odom_cov_x   # x variance
        odom.pose.covariance[7] = self.odom_cov_y   # y variance
        odom.pose.covariance[35] = self.odom_cov_yaw  # yaw variance
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Create PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.odom_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = Quaternion(*quaternion)
        
        # Publish pose
        self.pose_pub.publish(pose)
        
        # Broadcast transform
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            quaternion,
            rospy.Time.now(),
            self.base_frame,
            self.odom_frame
        )
        
    def run(self):
        """Main loop - update and publish odometry at regular intervals."""
        rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo("Odometry node running...")
        
        while not rospy.is_shutdown():
            self.update_odometry()
            self.publish_odometry()
            
            rate.sleep()


if __name__ == '__main__':
    try:
        node = OdometryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
