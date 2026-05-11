#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# Get robot parameters
ROBOT_NAME=${ROBOT_NAME:-duckiebot}
ROBOT_ID=${ROBOT_ID:-00}

# Set up ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash
source ${CATKIN_WS_DIR}/devel/setup.bash

# Launch the complete localization and mapping system
dt-exec roslaunch dbot_odometry all.launch \
    robot_name:=$ROBOT_NAME \
    robot_id:=$ROBOT_ID \
    launch_rviz:=true

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
