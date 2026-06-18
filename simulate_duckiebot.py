#!/usr/bin/env python3
"""
Duckiebot Virtuele Simulator
Publiceert encoder-ticks en cameraframes voor een virtuele robot die een vierkant rijdt.

Gebruik:
  Terminal 1: roscore
  Terminal 2: roslaunch dbot_odometry all.launch
  Terminal 3: python3 simulate_duckiebot.py
"""

import rospy
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

WORLD_SIZE = 3000   # pixels
CAMERA_W   = 320
CAMERA_H   = 240
SCALE      = 500    # pixels per meter


def build_world():
    """Bouw een bovenaanzicht wegkaart met rijstrookmarkeringen en willekeurige kenmerken."""
    world = np.full((WORLD_SIZE, WORLD_SIZE, 3), 55, dtype=np.uint8)

    m = 500  # marge van de rand tot het rijspoor

    # Buitenste en binnenste rijstrookgrenzen
    cv2.rectangle(world, (m, m), (WORLD_SIZE - m, WORLD_SIZE - m), (210, 210, 210), 8)
    cv2.rectangle(world, (m + 180, m + 180),
                  (WORLD_SIZE - m - 180, WORLD_SIZE - m - 180), (170, 170, 170), 4)

    # Gestippelde middenlijnen langs elke zijde
    dash, gap = 35, 25
    for i in range(m + 90, WORLD_SIZE - m - 90, dash + gap):
        cv2.line(world, (i, m + 90), (i + dash, m + 90), (240, 200, 0), 3)
        cv2.line(world, (i, WORLD_SIZE - m - 90),
                 (i + dash, WORLD_SIZE - m - 90), (240, 200, 0), 3)
        cv2.line(world, (m + 90, i), (m + 90, i + dash), (240, 200, 0), 3)
        cv2.line(world, (WORLD_SIZE - m - 90, i),
                 (WORLD_SIZE - m - 90, i + dash), (240, 200, 0), 3)

    # Hoekmarkeringen (oranje vakjes — goede features voor ORB)
    for cx, cy in [(m, m), (WORLD_SIZE - m, m),
                   (WORLD_SIZE - m, WORLD_SIZE - m), (m, WORLD_SIZE - m)]:
        cv2.rectangle(world, (cx - 25, cy - 25), (cx + 25, cy + 25), (0, 120, 255), -1)

    # Verspreide kenmerken (eendjes, borden, wegtextuurpunten)
    rng = np.random.default_rng(42)
    for _ in range(400):
        px = int(rng.integers(80, WORLD_SIZE - 80))
        py = int(rng.integers(80, WORLD_SIZE - 80))
        color = tuple(int(c) for c in rng.integers(80, 255, 3))
        cv2.circle(world, (px, py), int(rng.integers(3, 9)), color, -1)

    return world


class DuckiebotSimulator:
    def __init__(self):
        rospy.init_node('duckiebot_simulator', anonymous=False)

        self.robot_name = rospy.get_param('~robot_name', 'duckiebot')
        self.bridge = CvBridge()

        # Fysieke parameters — moeten overeenkomen met odometry_node.py
        self.wheel_radius = 0.02   # m
        self.wheelbase    = 0.10   # m
        self.ticks_per_rev = 135

        # Cumulatieve encoder-tellingen
        self.left_ticks  = 0
        self.right_ticks = 0

        # Robotpositie in wereldpixelruimte
        self.wx    = WORLD_SIZE / 2.0
        self.wy    = WORLD_SIZE / 2.0
        self.theta = 0.0

        rospy.loginfo("Virtuele wereld opbouwen…")
        self.world = build_world()

        self.left_pub  = rospy.Publisher(
            f'/{self.robot_name}/left_wheel_encoder_node/tick',  WheelEncoderStamped, queue_size=10)
        self.right_pub = rospy.Publisher(
            f'/{self.robot_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, queue_size=10)
        self.cam_pub   = rospy.Publisher(
            f'/{self.robot_name}/camera/image_raw',    Image, queue_size=10)

    # ------------------------------------------------------------------
    def _dist_to_ticks(self, dist):
        return int((dist / (2 * math.pi * self.wheel_radius)) * self.ticks_per_rev)

    def _camera_frame(self):
        hw, hh = CAMERA_W // 2, CAMERA_H // 2
        x0 = max(0, min(int(self.wx) - hw, WORLD_SIZE - CAMERA_W))
        y0 = max(0, min(int(self.wy) - hh, WORLD_SIZE - CAMERA_H))
        frame = self.world[y0:y0 + CAMERA_H, x0:x0 + CAMERA_W].copy()

        # Roteer beeld mee met de rijrichting van de robot
        M = cv2.getRotationMatrix2D(
            (CAMERA_W // 2, CAMERA_H // 2), -math.degrees(self.theta), 1.0)
        frame = cv2.warpAffine(frame, M, (CAMERA_W, CAMERA_H))

        # Klein HUD-overlay
        rx = (self.wx - WORLD_SIZE / 2) / SCALE
        ry = (self.wy - WORLD_SIZE / 2) / SCALE
        cv2.putText(frame,
                    f'x={rx:.2f}m y={ry:.2f}m th={math.degrees(self.theta):.0f}',
                    (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        return frame

    # ------------------------------------------------------------------
    def _step(self, left_vel, right_vel, dt):
        left_dist  = left_vel  * dt
        right_dist = right_vel * dt

        self.left_ticks  += self._dist_to_ticks(left_dist)
        self.right_ticks += self._dist_to_ticks(right_dist)

        # Dead-reckoning positie-update (zelfde kinematica als odometry_node.py)
        dist        = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheelbase
        self.theta += delta_theta
        self.wx    += dist * math.cos(self.theta) * SCALE
        self.wy    += dist * math.sin(self.theta) * SCALE

        now = rospy.Time.now()
        left_msg = WheelEncoderStamped()
        left_msg.header.stamp = now
        left_msg.data = self.left_ticks
        left_msg.resolution = self.ticks_per_rev
        left_msg.type = WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
        self.left_pub.publish(left_msg)

        right_msg = WheelEncoderStamped()
        right_msg.header.stamp = now
        right_msg.data = self.right_ticks
        right_msg.resolution = self.ticks_per_rev
        right_msg.type = WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
        self.right_pub.publish(right_msg)

        img_msg = self.bridge.cv2_to_imgmsg(self._camera_frame(), encoding='bgr8')
        img_msg.header.stamp = now
        self.cam_pub.publish(img_msg)

    def _drive(self, left_vel, right_vel, duration, hz=30):
        rate = rospy.Rate(hz)
        dt   = 1.0 / hz
        for _ in range(int(duration * hz)):
            if rospy.is_shutdown():
                return False
            self._step(left_vel, right_vel, dt)
            rate.sleep()
        return True

    # ------------------------------------------------------------------
    def run(self):
        rospy.loginfo("Simulator klaar — 2 s wachten tot nodes opgestart zijn…")
        rospy.sleep(2.0)
        rospy.loginfo("Vierkante route starten (4 × rechtdoor + bocht)")

        straight     = 0.15   # m/s beide wielen → ~45 cm/s
        turn_inner   = 0.04   # m/s binnenste wiel → ~90° bocht in ~2.0 s
        turn_outer   = 0.13   # m/s buitenste wiel

        lap = 0
        while not rospy.is_shutdown():
            lap += 1
            for side in range(4):
                rospy.loginfo(f"Ronde {lap}  zijde {side+1}/4 — rechtdoor")
                if not self._drive(straight, straight, 3.0):
                    return
                rospy.loginfo(f"Ronde {lap}  zijde {side+1}/4 — bocht")
                if not self._drive(turn_inner, turn_outer, 2.1):
                    return


if __name__ == '__main__':
    try:
        DuckiebotSimulator().run()
    except rospy.ROSInterruptException:
        pass
