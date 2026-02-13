#!/bin/bash
# Simple motor test script - starts only necessary components

cd /home/kyle/qwacr_ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

# Kill any existing processes
pkill -9 ros2 2>/dev/null
sleep 1

echo "=== Starting robot_state_publisher ==="
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro install/qwacr_build/share/qwacr_build/urdf/qwacr.urdf.xacro use_gazebo:=false serial_port:=/dev/ttyACM0 baud_rate:=115200)" \
  &

sleep 2

echo "=== Starting ros2_control_node ==="
ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file install/qwacr_build/share/qwacr_build/config/diff_drive_controller.yaml \
  &

sleep 5

echo "=== Loading and activating controllers ==="
ros2 control load_controller --set-state active joint_broad
ros2 control load_controller --set-state active diff_cont

echo "=== Controllers active! ==="
echo "Test with: ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.1}}'"

# Keep script running
wait
