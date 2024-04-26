# Sparse-Graphical-Memory-Robot-Planning
This project was done on Python 3.10.12, ROS2 Humble with Gazebo, Ubuntu 22.04. This README assumes that the user already has this setup.

Detailed information of the project can be found in the /report folder.

## Setup
### Python Requirements Installation
The required python modules are listed in the requirements.txt file. You can install them with `pip install -r requirements.txt`.

### ROS2 Requirements Installation
You will need to run a few command lines and instructions to install the necessary components.
1.  Run `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*` and `sudo apt install ros-humble-rmw-cyclonedds-cpp`
2. Add the lines `export TURTLEBOT3_MODEL=waffle` and
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to the end of your ~/.bashrc.
3. Run `sudo nano /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml` and change one of the parameter 'robot_model_type' value to 'nav2_amcl::DifferentialMotionModel'
4. Run `sudo apt install ros-humble-slam-toolbox`

## Startup
Run the following three commands in different terminals. 
1.  `ros2 launch sgm_robot_bringup sgm_robot_gazebo.launch.xml`
  - This will open the Gazebo and RViz showing the robot. 
2.  `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/sgm_robot_bringup/maps/standard_sgm_map_resized.yaml`
  - This will open the RViz showing the interactable map.
  - On the left side of the interface, add in a "MarkerArray" that topic is listenting to `visualization_marker`
  - You may also want to select "Set 2D Point Estimate" as well as its orientation to get a general idea of where the robot is on the map while it is moving later. 
3.  `ros2 launch sgm_robot_bringup sgm_robot_navigation.launch.xml`
  - This will run the overall code of the program.
  - This will also launch a ROS client that sends a goal automatically after 2 mins.
4. At any point now, you may send a goal with `ros2 action send_goal /robot_navigate sgm_robot_interfaces/action/RobotNavigate "{target_node: p}`. Replace `p` with an integer that is the node index to send the robot to.