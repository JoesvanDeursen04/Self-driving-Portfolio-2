#!/usr/bin/env python3
"""
Live visualizer — toont odometry, EKF-fusie en SLAM features via matplotlib.
Werkt als vervanging voor RViz in Docker op M-chip Mac.

Gebruik: python3 visualizer.py
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import numpy as np
import threading

# Datapunten
odom_x, odom_y   = [], []
fused_x, fused_y = [], []
feat_x,  feat_y  = [], []
lock = threading.Lock()


def odom_cb(msg):
    with lock:
        odom_x.append(msg.pose.pose.position.x)
        odom_y.append(msg.pose.pose.position.y)

def fused_cb(msg):
    with lock:
        fused_x.append(msg.pose.position.x)
        fused_y.append(msg.pose.position.y)

def feat_cb(msg):
    # Lees de eerste 50 XYZ punten uit de PointCloud2
    import struct
    pts_x, pts_y = [], []
    step = msg.point_step
    for i in range(min(50, msg.width)):
        offset = i * step
        x = struct.unpack_from('f', msg.data, offset)[0]
        y = struct.unpack_from('f', msg.data, offset + 4)[0]
        pts_x.append(x)
        pts_y.append(y)
    with lock:
        feat_x.clear(); feat_x.extend(pts_x)
        feat_y.clear(); feat_y.extend(pts_y)


rospy.init_node('visualizer', anonymous=True)
rospy.Subscriber('/odometry',        Odometry,      odom_cb)
rospy.Subscriber('/fused_pose',      PoseStamped,   fused_cb)
rospy.Subscriber('/slam/features',   PointCloud2,   feat_cb)

# Plot opzetten
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
fig.patch.set_facecolor('#1e1e1e')
ax.set_facecolor('#1e1e1e')
ax.set_title('Live Localisatie & Mapping', color='white', fontsize=14)
ax.set_xlabel('X (m)', color='white')
ax.set_ylabel('Y (m)', color='white')
ax.tick_params(colors='white')
for spine in ax.spines.values():
    spine.set_edgecolor('#444444')

line_odom,  = ax.plot([], [], 'b-',  linewidth=2, label='Odometry')
line_fused, = ax.plot([], [], 'g-',  linewidth=2, label='EKF Fusie')
scat_feat   = ax.scatter([], [], c='red', s=8, alpha=0.5, label='SLAM Features')

legenda = ax.legend(facecolor='#333333', loc='upper left')
for text in legenda.get_texts():
    text.set_color('white')
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.grid(True, color='#333333')

plt.tight_layout()
plt.show()

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    with lock:
        ox, oy = list(odom_x), list(odom_y)
        fx, fy = list(fused_x), list(fused_y)
        ex, ey = list(feat_x),  list(feat_y)

    if ox:
        line_odom.set_data(ox, oy)
    if fx:
        line_fused.set_data(fx, fy)
    if ex:
        scat_feat.set_offsets(np.c_[ex, ey])

    # Automatisch schalen op het pad
    all_x = ox + fx
    all_y = oy + fy
    if len(all_x) > 1:
        pad = 0.5
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

    fig.canvas.draw()
    fig.canvas.flush_events()
    rate.sleep()
