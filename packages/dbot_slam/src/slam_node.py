#!/usr/bin/env python3

"""
Vision-based SLAM Node for Duckiebot
Detects and tracks visual features for mapping and localization.

Topics:
    Subscribes:
        - /{robot_name}/camera/image_raw (sensor_msgs/Image) - e.g., /duckiebot/camera/image_raw
    Publishes:
        - /slam/features (sensor_msgs/PointCloud2)
        - /slam/camera_motion (geometry_msgs/PoseStamped)
        - /slam/feature_visualization (sensor_msgs/Image)
"""

import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf.transformations as tf_trans
from collections import deque


class SLAMNode:
    """
    Vision-based monocular SLAM node for Duckiebot.
    
    Detects and tracks visual features (corners, patterns) across frames.
    Estimates relative camera motion and builds a map of feature points.
    """
    
    def __init__(self):
        """Initialize SLAM node with feature detector and tracker."""
        rospy.init_node('slam_node', anonymous=False)
        
        # Robot and frame parameters
        self.robot_name = rospy.get_param('~robot_name', 'duckiebot')
        self.camera_frame = f'{self.robot_name}/camera'
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Feature detection parameters
        self.feature_max_distance = 50  # pixels
        self.min_feature_distance = 30  # pixels
        self.feature_quality = 0.01  # corner detection quality
        self.max_features = 200  # max features to track
        
        # SLAM parameters
        self.keyframe_interval = 5  # frames between keyframes
        self.frame_count = 0
        self.feature_map = {}  # 3D map of features
        self.feature_id_counter = 0
        self.last_keyframe_pose = np.eye(4)
        
        # Previous frame data for tracking
        self.prev_gray = None
        self.prev_features = None
        self.feature_tracks = deque(maxlen=100)  # Store recent feature matches
        
        # Intrinsic camera matrix (default for Duckiebot camera)
        self.fx = rospy.get_param('~focal_length_x', 183)
        self.fy = rospy.get_param('~focal_length_y', 183)
        self.cx = rospy.get_param('~principal_point_x', 160)
        self.cy = rospy.get_param('~principal_point_y', 120)
        
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Camera to robot transform (roll, pitch, yaw)
        self.camera_transform = np.eye(4)
        
        # Current estimated pose
        self.current_pose = np.eye(4)
        
        # Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Feature detector - using ORB (Oriented FAST and Rotated BRIEF)
        self.orb = cv2.ORB_create(nfeatures=self.max_features)
        
        # BFMatcher for feature matching
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Publishers
        self.viz_pub = rospy.Publisher(
            '/slam/feature_visualization',
            Image,
            queue_size=10
        )
        self.motion_pub = rospy.Publisher(
            '/slam/camera_motion',
            PoseStamped,
            queue_size=10
        )
        self.features_pub = rospy.Publisher(
            '/slam/features',
            PointCloud2,
            queue_size=10
        )
        
        # Subscriber
        rospy.Subscriber(
            f'/{self.robot_name}/camera/image_raw',
            Image,
            self.image_callback
        )
        
        rospy.loginfo(f"SLAM node initialized for {self.robot_name}")
        
    def image_callback(self, msg):
        """
        Process camera image for feature detection and tracking.
        
        Args:
            msg: sensor_msgs/Image
        """
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Process frame
            self.process_frame(frame, gray, msg.header.stamp)
            
        except Exception as e:
            rospy.logwarn(f"Error processing image: {e}")
            
    def process_frame(self, frame, gray, timestamp):
        """
        Detect and track features in frame.
        
        Args:
            frame: Color frame (BGR)
            gray: Grayscale frame
            timestamp: ROS timestamp
        """
        # Detect keypoints and descriptors
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        visualization = frame.copy()
        
        if self.prev_gray is None:
            # First frame - initialize tracking
            self.prev_gray = gray
            self.prev_features = (keypoints, descriptors)
            self.frame_count = 0
            rospy.loginfo("SLAM: First frame processed, starting feature tracking")
            return
            
        # Match features between frames
        if descriptors is not None and self.prev_features[1] is not None:
            matches = self.bf_matcher.knnMatch(
                self.prev_features[1],
                descriptors,
                k=2
            )
            
            # Apply Lowe's ratio test to filter good matches
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.7 * n.distance:
                        good_matches.append(m)
            
            # Extract matched feature points
            if good_matches:
                src_pts = np.float32([
                    self.prev_features[0][m.queryIdx].pt for m in good_matches
                ]).reshape(-1, 1, 2)
                dst_pts = np.float32([
                    keypoints[m.trainIdx].pt for m in good_matches
                ]).reshape(-1, 1, 2)
                
                # Estimate fundamental matrix
                F, mask = cv2.findFundamentalMat(src_pts, dst_pts, cv2.FM_RANSAC)
                
                # Estimate essential matrix
                if F is not None and F.shape == (3, 3):
                    E = self.camera_matrix.T @ F @ self.camera_matrix
                    
                    # Decompose essential matrix
                    _, R, t, mask = cv2.recoverPose(E, src_pts, dst_pts, self.camera_matrix)
                    
                    # Update pose
                    pose_delta = np.eye(4)
                    pose_delta[:3, :3] = R
                    pose_delta[:3, 3] = t.flatten()
                    
                    self.current_pose = self.current_pose @ np.linalg.inv(pose_delta)
                    
                    # Track features for map
                    for m in good_matches:
                        feature_id = m.queryIdx
                        if feature_id not in self.feature_map:
                            self.feature_map[feature_id] = {
                                'positions': [self.prev_features[0][m.queryIdx].pt],
                                'descriptors': self.prev_features[1][m.queryIdx],
                                'last_seen': timestamp.to_sec()
                            }
                        else:
                            self.feature_map[feature_id]['positions'].append(
                                keypoints[m.trainIdx].pt
                            )
                            self.feature_map[feature_id]['last_seen'] = timestamp.to_sec()
            
            # Draw matches on visualization
            visualization = cv2.drawMatches(
                self.prev_gray,
                self.prev_features[0],
                gray,
                keypoints,
                good_matches[:20],  # Draw top 20 matches
                None,
                matchColor=(0, 255, 0),
                singlePointColor=(255, 0, 0),
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
        
        # Draw keypoints
        visualization_with_kp = cv2.drawKeypoints(
            gray,
            keypoints,
            visualization,
            color=(0, 255, 255),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        
        # Publish visualization
        viz_msg = self.bridge.cv2_to_imgmsg(visualization_with_kp, encoding='bgr8')
        viz_msg.header.stamp = timestamp
        self.viz_pub.publish(viz_msg)
        
        # Publish camera motion
        self.publish_camera_motion(timestamp)
        
        # Publish feature cloud
        self.publish_feature_cloud(timestamp)
        
        # Update for next frame
        self.prev_gray = gray
        self.prev_features = (keypoints, descriptors)
        self.frame_count += 1
        
    def publish_camera_motion(self, timestamp):
        """
        Publish estimated camera motion as PoseStamped.
        
        Args:
            timestamp: ROS timestamp
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'camera_frame'
        
        # Extract position
        pose_msg.pose.position.x = self.current_pose[0, 3]
        pose_msg.pose.position.y = self.current_pose[1, 3]
        pose_msg.pose.position.z = self.current_pose[2, 3]
        
        # Extract rotation as quaternion
        quaternion = tf_trans.quaternion_from_matrix(self.current_pose)
        pose_msg.pose.orientation = Quaternion(*quaternion)
        
        self.motion_pub.publish(pose_msg)
        
    def publish_feature_cloud(self, timestamp):
        """
        Publish 3D feature map as PointCloud2.
        
        Args:
            timestamp: ROS timestamp
        """
        points = []
        
        # Add features from map
        for feature_id, feature_data in self.feature_map.items():
            # Use most recent position
            if feature_data['positions']:
                x_pixel, y_pixel = feature_data['positions'][-1]
                
                # Simple triangulation: project to 3D using assumed depth
                depth = 0.5  # Assume 0.5 m depth (can be improved with stereo)
                x_3d = (x_pixel - self.cx) * depth / self.fx
                y_3d = (y_pixel - self.cy) * depth / self.fy
                z_3d = depth
                
                points.append([x_3d, y_3d, z_3d])
        
        if not points:
            return
            
        # Create PointCloud2 message
        cloud = PointCloud2()
        cloud.header.stamp = timestamp
        cloud.header.frame_id = self.camera_frame
        
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_dense = True
        
        # Define fields
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        
        # Populate point data
        cloud.data = np.asarray(points, dtype=np.float32).tobytes()
        
        self.features_pub.publish(cloud)
        
    def run(self):
        """Main loop."""
        rospy.loginfo("SLAM node running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = SLAMNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
