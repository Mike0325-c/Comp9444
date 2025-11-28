How to Run the System (Stage 2 Navigation)

（最简洁版运行说明）

1. 运行 Stage 1（Wall Follower + Landmark Recording）
Terminal 1 — 启动 Gazebo Maze
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py

Terminal 2 — 启动 SLAM（Cartographer）
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

Terminal 3 — 启动 Wall Follower + Marker Detection（Stage 1）
ros2 launch wall_follower wall_follower.launch.py


机器人会开始走迷宫，并且看到颜色柱子时，会：

see_marker.py 检测到颜色柱子

point_transformer.py 转换坐标并保存

运行结束后，按 Ctrl + C 停掉 wall_follower → 自动生成地图与 marker 文件

手动保存地图（在任意 terminal）：

ros2 run nav2_map_server map_saver_cli -f ~/map


Marker 坐标文件（markers.txt）会由 point_transformer 自动生成。

2. 运行 Stage 2（Navigation）
Terminal 1 — 启动 Gazebo Maze
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py

Terminal 2 — 启动 Nav2 并加载地图
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=True \
    map:=~/map.yaml \
    params_file:=~/turtlebot3_ws/src/wall_follower/config/waypoint_nav_params.yaml

在 RViz 中设置 Initial Pose

点击 2D Pose Estimate，放在地图的起点。

等待 AMCL 收敛（绿色粒子集中）。

Terminal 3 — 启动 Waypoint Navigator（读取 markers.txt 并开始导航）
ros2 run wall_follower waypoint_navigator


机器人会：

从 markers.txt 读取坐标

按顺序发送 Nav2 goal

自动走完整个迷宫
