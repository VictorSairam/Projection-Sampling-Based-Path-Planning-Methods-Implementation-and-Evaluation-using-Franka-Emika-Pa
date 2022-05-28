# Projection-Sampling-Based-Path-Planning-Methods-Implementation-and-Evaluation-using-Franka-Emika-Panda

This file explains how to perform path planning using PRM, OBPRM, CBiRRT and RRT algorithms using ROS and a 7 DoF robotic arm.

1) Install python3 (ver:3.5) on your Ubuntu 16.04.
2) First install numpy, rospkg and numpy-quaternion.
3) Install libfranka and franka_ros: sudo apt install ros-kinetic-libfranka ros-kinetic-franka-ros`
4) In order to visualize the robot path planning on Rviz, change the path argumrnt in visualize.launch to the path of your working folder.\
5) We use parser to run a combination of path planning algorithms and environments.
6) To perform PRM, run the command: "python3 path_plan.py --prm", with map2: "python3 path_plan.py --map2 --prm", with map3: "python3 path_plan.py --map3 --prm".
7) To perform OBPRM, run the command: "python3 path_plan.py --obprm", with map2: "python3 path_plan.py --map2 --obprm", with map3: "python3 path_plan.py --map3 --obprm".
8) To run RRT algorithm, run the command: "python3 path_plan.py --rrt", with map2: "python3 path_plan.py --map2 --rrt", with map3: "python3 path_plan.py --map3 --rrt".
9) To run CBiRRT algorithm, run the command: "python3 path_plan.py --rrtc", with map2: "python3 path_plan.py --map2 --rrtc", with map3: "python3 path_plan.py --map3 --rrtc".
10) After the above commands, run roslaunch visualize.py.
11) After Rviz opens up, select the robot modelmm, and all the topics published to be displayed.

(NOTE: To reuse graph created in PRM and OBPRM, run "python3 path_plan.py --<algorithm> --<map_name> --reuse_graph.)

# Octomap based mapping:
  
To perform the mapping using the Harrier D7 and Kinova arm, copy the src file from the project_ws into your wokspace.
Do:
  1) catkin_make: build the workspace.
  2) source it to bashrc.
  2) Open command prompt.
  3) Write: " roslaunch aerial_manipulation aerial_manipulator.launch " to spawn the UAV.
  4) Write: " roslaunch aerial_manipulation uav_teleop.launch " to enable teleop using keyboard.
  5) After assigning the desired topic to publish to and after selecting the topic of the depth camera points, type: " roslaunch octomap_server octomap_mapping.launch.
  6) Launch Rviz: "rosrun rviz rviz".
  7) Select the robot model and the occupancy grid topic to publish and the mapping will begin.
  8) Invert the z-axis on Rviz to obtain the octomap.
  9) Perform the mapping using the teleop control.
  10) Perform SLAM using the hector slam package and dave the octomap projection and the octomap as well.
  10) Launch the map before perform path planning using one of the graph search methods.
