# Sparse-Graphical-Memory-Robot-Planning

## Startup
Run the following three commands in different terminals. 
1.  `ros2 launch sgm_robot_bringup sgm_robot_gazebo.launch.xml`
2.  `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/sgm_robot_bringup/maps/standard_sgm_map_resized.yaml`
3.  `ros2 launch sgm_robot_bringup sgm_robot_navigation.launch.xml`