#!/bin/bash

# Infinite loop
while true; do
  # Launch Serial RRT Node
  echo "Launching Serial RRT"
  cd /home/moises/Documents/POFT-Final-Project/POfT-Final-Project/serial_rrt_ws && colcon build && source install/setup.bash && ros2 launch assignment3 ur5_launch.py autograde:=true &
  ROS_PID=$!
  echo "Waiting for 6 minutes and 15 seconds..."
  sleep 365
  echo "Terminating ROS2 node..."
  pkill -f ros
  sleep 3

  # Launch Parallel Validation RRT Node
  echo "Launching Parallel Validation RRT"
  cd /home/moises/Documents/POFT-Final-Project/POfT-Final-Project/parallel_rrt_ws_validation && colcon build && source install/setup.bash && ros2 launch assignment3 ur5_launch.py autograde:=true &
  ROS_PID=$!
  echo "Waiting for 6 minutes and 15 seconds..."
  sleep 365
  echo "Terminating ROS2 node..."
  pkill -f ros
  sleep 3

  # Launch Parallel Manytree RRT Node
  echo "Launching Parallel Manytree RRT"
  cd /home/moises/Documents/POFT-Final-Project/POfT-Final-Project/parallel_rrt_ws_manytree && colcon build && source install/setup.bash && ros2 launch assignment3 ur5_launch.py autograde:=true &
  ROS_PID=$!
  echo "Waiting for 6 minutes and 15 seconds..."
  sleep 365
  echo "Terminating ROS2 node..."
  pkill -f ros
  sleep 3

  # Launch Parallel Onetree RRT Node
  echo "Launching Parallel Onetree RRT"
  cd /home/moises/Documents/POFT-Final-Project/POfT-Final-Project/parallel_rrt_ws_onetree && colcon build && source install/setup.bash && ros2 launch assignment3 ur5_launch.py autograde:=true &
  ROS_PID=$!
  echo "Waiting for 6 minutes and 15 seconds..."
  sleep 365
  echo "Terminating ROS2 node..."
  pkill -f ros
  sleep 3

done
