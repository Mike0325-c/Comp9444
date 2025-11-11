ros2 run nav2_map_server map_saver_cli -f ~/map
cd ~/turtlebot3_ws/src/wall_follower/scripts
chmod +x waypoint_navigator.py


ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  map:=/home/pi/map.yaml \
  params_file:=/home/pi/turtlebot3_ws/src/wall_follower/config/waypoint_nav_params.yaml

ros2 launch nav2_bringup rviz_launch.py



