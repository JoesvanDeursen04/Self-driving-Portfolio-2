#!/usr/bin/env python3

"""Semantic perception node for Duckiebot.

This node combines two complementary perception layers:
- AprilTag detection for fixed semantically meaningful landmarks such as signs
  and lights.
- ONNX-based object detection for duckies.

Detected landmarks are projected into the odom frame using the fused robot pose
so they can be visualized and reused by mapping or navigation components.
"""

import ast
import json
import math
import os
from dataclasses import dataclass

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as tf_trans
from duckietown.dtros import DTROS, NodeType


@dataclass
class LandmarkObservation:
    label: str
    kind: str
    x: float
    y: float
    confidence: float


class SemanticPerceptionNode(DTROS):
    def __init__(self):
        super(SemanticPerceptionNode, self).__init__(node_name='semantic_perception_node', node_type=NodeType.PERCEPTION)

        self.robot_name = rospy.get_param('~robot_name', 'duckiebot')
        self.use_compressed = rospy.get_param('~use_compressed', True)
        camera_topic_default = (
            f'/{self.robot_name}/camera_node/image/compressed'
            if self.use_compressed
            else f'/{self.robot_name}/camera/image_raw'
        )
        self.camera_topic = rospy.get_param('~camera_topic', camera_topic_default)
        self.pose_topic = rospy.get_param('~pose_topic', '/fused_pose')
        self.camera_frame = rospy.get_param('~camera_frame', f'{self.robot_name}/camera')
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.model_path = rospy.get_param('~model_path', os.path.join(package_root, 'models', 'best.onnx'))

        self.fx = float(rospy.get_param('~fx', 183.0))
        self.fy = float(rospy.get_param('~fy', 183.0))
        self.cx = float(rospy.get_param('~cx', 160.0))
        self.cy = float(rospy.get_param('~cy', 120.0))
        self.fallback_width_px = float(rospy.get_param('~fallback_width_px', 320.0))
        self.fallback_height_px = float(rospy.get_param('~fallback_height_px', 240.0))
        self.base_fx = self.fx
        self.base_fy = self.fy
        self.base_cx = self.cx
        self.base_cy = self.cy
        self.camera_info_received = False
        self.camera_matrix = np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.duckie_class_name = rospy.get_param('~duckie_class_name', 'duckie')
        self.duckie_real_height_m = float(rospy.get_param('~duckie_real_height_m', 0.15))
        self.duckie_score_threshold = float(rospy.get_param('~duckie_score_threshold', 0.35))
        self.duckie_nms_threshold = float(rospy.get_param('~duckie_nms_threshold', 0.4))
        self.tag_size_m = float(rospy.get_param('~tag_size_m', 0.08))
        self.camera_offset_x_m = float(rospy.get_param('~camera_offset_x_m', 0.06))
        self.camera_offset_y_m = float(rospy.get_param('~camera_offset_y_m', 0.0))
        self.tag_semantics_file = rospy.get_param('~tag_semantics_file', '')
        self.tag_semantics = self._load_tag_semantics(rospy.get_param('~tag_semantics', {}), self.tag_semantics_file)

        self.bridge = CvBridge()
        self.current_pose = None
        self.frame_index = 0
        self.landmarks = {}
        self.next_landmark_id = 1

        self.net = self._load_model(self.model_path)
        self.aruco_dictionary = self._load_apriltag_dictionary()
        self.aruco_params = self._create_aruco_params()

        self.marker_pub = rospy.Publisher('/semantic_perception/markers', MarkerArray, queue_size=10)
        self.debug_pub = rospy.Publisher('/semantic_perception/debug_image', Image, queue_size=10)

        # Subscribe to camera calibration; hardcoded values above are used as fallback
        rospy.Subscriber(
            f'/{self.robot_name}/camera_node/camera_info',
            CameraInfo,
            self.camera_info_callback
        )

        # Subscribe to camera images (compressed for real hardware, raw for simulator)
        if self.use_compressed:
            rospy.Subscriber(self.camera_topic, CompressedImage, self.compressed_image_callback, queue_size=1)
        else:
            rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)

        rospy.loginfo('Semantic perception node ready. Camera: %s (compressed=%s)', self.camera_topic, self.use_compressed)

    def _load_model(self, model_path):
        if not os.path.exists(model_path):
            rospy.logwarn('Duckie ONNX model not found at %s; AprilTags will still run.', model_path)
            return None
        try:
            net = cv2.dnn.readNetFromONNX(model_path)
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            return net
        except Exception as exc:
            rospy.logwarn('Could not load ONNX model %s: %s', model_path, exc)
            return None

    def _load_tag_semantics(self, raw_value, semantics_file=''):
        if semantics_file:
            semantics_file = os.path.expanduser(semantics_file)
            if os.path.exists(semantics_file):
                try:
                    with open(semantics_file, 'r', encoding='utf-8') as handle:
                        loaded = json.load(handle)
                    if isinstance(loaded, dict):
                        return {str(key): value for key, value in loaded.items()}
                except Exception as exc:
                    rospy.logwarn('Could not load tag semantics file %s: %s', semantics_file, exc)
        if isinstance(raw_value, dict):
            return {str(key): value for key, value in raw_value.items()}
        if isinstance(raw_value, str):
            try:
                parsed = ast.literal_eval(raw_value)
                if isinstance(parsed, dict):
                    return {str(key): value for key, value in parsed.items()}
            except Exception:
                return {}
        return {}

    def _load_apriltag_dictionary(self):
        aruco = getattr(cv2, 'aruco', None)
        if aruco is None:
            rospy.logwarn('OpenCV aruco module is not available; AprilTag detection is disabled.')
            return None
        if hasattr(aruco, 'DICT_APRILTAG_36h11'):
            return aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        if hasattr(aruco, 'Dictionary_get') and hasattr(aruco, 'DICT_APRILTAG_36H11'):
            return aruco.Dictionary_get(aruco.DICT_APRILTAG_36H11)
        rospy.logwarn('AprilTag dictionary not available in this OpenCV build.')
        return None

    def _create_aruco_params(self):
        aruco = getattr(cv2, 'aruco', None)
        if aruco is None:
            return None
        if hasattr(aruco, 'DetectorParameters_create'):
            return aruco.DetectorParameters_create()
        if hasattr(aruco, 'DetectorParameters'):
            return aruco.DetectorParameters()
        return None

    def camera_info_callback(self, msg):
        """Update camera calibration matrix from camera_info topic."""
        if msg.K[0] > 0:
            self.camera_matrix = np.array(msg.K, dtype=np.float32).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D, dtype=np.float32).reshape(-1, 1)
            self.fx = float(self.camera_matrix[0, 0])
            self.fy = float(self.camera_matrix[1, 1])
            self.cx = float(self.camera_matrix[0, 2])
            self.cy = float(self.camera_matrix[1, 2])
            self.camera_info_received = True

    def compressed_image_callback(self, msg):
        """Decode a CompressedImage from the real Duckiebot camera and process it."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                rospy.logwarn_throttle(5.0, "Semantic: failed to decode compressed image")
                return
            self._process_frame(frame, msg.header.stamp)
        except Exception as exc:
            rospy.logwarn('Could not decode compressed image: %s', exc)

    def pose_callback(self, msg):
        self.current_pose = msg

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn('Could not convert camera image: %s', exc)
            return
        self._process_frame(frame, msg.header.stamp)

    def _process_frame(self, frame, stamp):
        self._ensure_intrinsics_for_frame(frame)

        observations = []
        debug = frame.copy()

        observations.extend(self._detect_apriltags(frame, debug))
        observations.extend(self._detect_duckies(frame, debug))

        self._integrate_observations(observations)
        self._publish_markers(stamp)
        self._publish_debug_image(debug, stamp)
        self.frame_index += 1

    def _ensure_intrinsics_for_frame(self, frame):
        """Scale fallback intrinsics to current frame resolution until camera_info arrives."""
        if self.camera_info_received:
            return

        frame_h, frame_w = frame.shape[:2]
        scale_x = float(frame_w) / self.fallback_width_px
        scale_y = float(frame_h) / self.fallback_height_px

        self.fx = self.base_fx * scale_x
        self.fy = self.base_fy * scale_y
        self.cx = self.base_cx * scale_x
        self.cy = self.base_cy * scale_y
        self.camera_matrix = np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float32
        )

    def _detect_apriltags(self, frame, debug):
        observations = []
        if self.aruco_dictionary is None or self.aruco_params is None:
            return observations

        aruco = cv2.aruco
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dictionary, parameters=self.aruco_params)
        if ids is None:
            return observations

        if hasattr(aruco, 'drawDetectedMarkers'):
            aruco.drawDetectedMarkers(debug, corners, ids)

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            object_points = np.array([
                [-self.tag_size_m / 2.0, self.tag_size_m / 2.0, 0.0],
                [self.tag_size_m / 2.0, self.tag_size_m / 2.0, 0.0],
                [self.tag_size_m / 2.0, -self.tag_size_m / 2.0, 0.0],
                [-self.tag_size_m / 2.0, -self.tag_size_m / 2.0, 0.0],
            ], dtype=np.float32)
            image_points = marker_corners.reshape(-1, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            label = self.tag_semantics.get(str(int(marker_id)), f'apriltag_{int(marker_id)}')
            world_x, world_y = self._project_camera_point_to_world(float(tvec[2]), float(tvec[0]), label)
            observations.append(LandmarkObservation(label=label, kind='apriltag', x=world_x, y=world_y, confidence=1.0))
            self._draw_label(debug, marker_corners, f'{label} #{int(marker_id)}', (0, 0, 255))

        return observations

    def _detect_duckies(self, frame, debug):
        observations = []
        if self.net is None:
            return observations

        input_size = 320
        blob = cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (input_size, input_size), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        detections = self._parse_duckie_outputs(outputs, frame.shape[:2], input_size)
        if not detections:
            return observations

        boxes = [det['box'] for det in detections]
        scores = [det['score'] for det in detections]
        keep_indices = cv2.dnn.NMSBoxes(boxes, scores, self.duckie_score_threshold, self.duckie_nms_threshold)
        if len(keep_indices) == 0:
            return observations

        for index in np.array(keep_indices).flatten():
            det = detections[int(index)]
            x, y, w, h = det['box']
            center_x = x + (w / 2.0)
            center_y = y + (h / 2.0)
            confidence = det['score']
            if h <= 1:
                continue

            range_m = (self.fx * self.duckie_real_height_m) / float(h)
            bearing = math.atan2(center_x - self.cx, self.fx)
            world_x, world_y = self._project_camera_point_to_world(range_m, math.tan(bearing) * range_m, self.duckie_class_name)
            observations.append(LandmarkObservation(label=self.duckie_class_name, kind='duckie', x=world_x, y=world_y, confidence=confidence))

            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 165, 255), 2)
            cv2.putText(debug, f'{self.duckie_class_name} {confidence:.2f}', (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        return observations

    def _parse_duckie_outputs(self, outputs, image_shape, input_size):
        detections = []
        img_h, img_w = image_shape
        scale_x = float(img_w) / float(input_size)
        scale_y = float(img_h) / float(input_size)

        for output in outputs:
            array = np.array(output)
            if array.ndim == 3:
                array = array[0]
            if array.ndim != 2:
                continue
            if array.shape[0] < array.shape[1] and array.shape[0] in (5, 6, 7, 84, 85):
                array = array.T

            for row in array:
                flat = np.asarray(row).reshape(-1)
                if flat.size < 4:
                    continue

                if flat.size == 6:
                    x1, y1, x2, y2, score, class_id = flat[:6]
                    if score < self.duckie_score_threshold or int(class_id) != 0:
                        continue
                    box = self._to_xywh(x1, y1, x2, y2, scale_x, scale_y)
                    detections.append({'box': box, 'score': float(score)})
                    continue

                if flat.size >= 5:
                    cx, cy, bw, bh = flat[:4]
                    objectness = float(flat[4]) if flat.size > 4 else 1.0
                    if flat.size > 5:
                        class_scores = flat[5:]
                        class_id = int(np.argmax(class_scores))
                        confidence = float(objectness * np.max(class_scores))
                    else:
                        class_id = 0
                        confidence = objectness
                    if class_id != 0 or confidence < self.duckie_score_threshold:
                        continue

                    if max(cx, cy, bw, bh) <= 1.5:
                        cx *= input_size
                        cy *= input_size
                        bw *= input_size
                        bh *= input_size
                    x = int((cx - (bw / 2.0)) * scale_x)
                    y = int((cy - (bh / 2.0)) * scale_y)
                    w = int(bw * scale_x)
                    h = int(bh * scale_y)
                    detections.append({'box': [x, y, w, h], 'score': confidence})

        return detections

    def _to_xywh(self, x1, y1, x2, y2, scale_x, scale_y):
        x = int(min(x1, x2) * scale_x)
        y = int(min(y1, y2) * scale_y)
        w = int(abs(x2 - x1) * scale_x)
        h = int(abs(y2 - y1) * scale_y)
        return [x, y, w, h]

    def _project_camera_point_to_world(self, forward_m, lateral_m, label):
        # Camera lateral is positive to the right; robot/world y is positive to the left.
        lateral_left_m = -lateral_m
        forward_base_m = forward_m + self.camera_offset_x_m
        lateral_base_m = lateral_left_m + self.camera_offset_y_m
        if self.current_pose is None:
            return forward_base_m, lateral_base_m

        yaw = self._quaternion_to_yaw(self.current_pose.pose.orientation)
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        world_x = robot_x + (forward_base_m * math.cos(yaw)) - (lateral_base_m * math.sin(yaw))
        world_y = robot_y + (forward_base_m * math.sin(yaw)) + (lateral_base_m * math.cos(yaw))
        return world_x, world_y

    def _integrate_observations(self, observations):
        for observation in observations:
            landmark_id = self._match_landmark(observation)
            self.landmarks[landmark_id] = {
                'label': observation.label,
                'kind': observation.kind,
                'x': observation.x,
                'y': observation.y,
                'confidence': observation.confidence,
                'last_seen': rospy.Time.now().to_sec(),
            }

    def _match_landmark(self, observation):
        if observation.kind == 'apriltag':
            return f'{observation.kind}:{observation.label}'

        best_id = None
        best_distance = float('inf')
        for landmark_id, landmark in self.landmarks.items():
            if landmark['kind'] != observation.kind:
                continue
            distance = math.hypot(landmark['x'] - observation.x, landmark['y'] - observation.y)
            if distance < best_distance:
                best_distance = distance
                best_id = landmark_id
        if best_id is not None and best_distance < 0.5:
            return best_id

        landmark_id = f'{observation.kind}:{self.next_landmark_id}'
        self.next_landmark_id += 1
        return landmark_id

    def _publish_markers(self, stamp):
        marker_array = MarkerArray()
        marker_id = 0

        for landmark in self.landmarks.values():
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'odom'
            marker.ns = landmark['kind']
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER if landmark['kind'] == 'apriltag' else Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = landmark['x']
            marker.pose.position.y = landmark['y']
            marker.pose.position.z = 0.05
            marker.pose.orientation = Quaternion(*tf_trans.quaternion_from_euler(0.0, 0.0, 0.0))
            marker.scale.x = 0.10 if landmark['kind'] == 'apriltag' else 0.14
            marker.scale.y = 0.10 if landmark['kind'] == 'apriltag' else 0.14
            marker.scale.z = 0.05
            marker.color = ColorRGBA(0.2, 0.8, 0.2, 0.95) if landmark['kind'] == 'duckie' else ColorRGBA(0.85, 0.2, 0.2, 0.95)
            marker.lifetime = rospy.Duration(0.0)
            marker_array.markers.append(marker)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = 'odom'
            text.ns = f"{landmark['kind']}_text"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = landmark['x']
            text.pose.position.y = landmark['y']
            text.pose.position.z = 0.18
            text.scale.z = 0.12
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text.text = f"{landmark['label']} ({landmark['confidence']:.2f})"
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def _publish_debug_image(self, frame, stamp):
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = stamp
        self.debug_pub.publish(image_msg)

    def _draw_label(self, frame, corners, text, color):
        points = corners.reshape(-1, 2).astype(int)
        x, y = int(points[:, 0].min()), int(points[:, 1].min())
        cv2.polylines(frame, [points], True, color, 2)
        cv2.putText(frame, text, (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def _quaternion_to_yaw(self, quaternion):
        return tf_trans.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

    def run(self):
        rospy.loginfo('Semantic perception running.')
        rospy.spin()


if __name__ == '__main__':
    try:
        SemanticPerceptionNode().run()
    except rospy.ROSInterruptException:
        pass
