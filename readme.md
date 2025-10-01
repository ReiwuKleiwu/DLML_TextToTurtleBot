# Setup - Physical
1. Make sure the Turtlebot 4 is running
### Run the following commands in the Conda Environment
2. Run SLAM using `ros2 launch turtlebot4_navigation slam.launch.py namespace:=robot_1`
3. RUN Nav2 using `ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot_1`
4. Optional: Run Rviz using `ros2 launch turtlebot4_viz view_robot.launch.py namespace:=robot_1`
