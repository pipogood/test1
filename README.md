# This repository for install UR3e launch package
  Reference & More Detail: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy

# Setup
  Create a new ROS2 workspace, pull relevant packages, install dependencies, compile, and source the workspace by using:
```
cd (your workspace)
git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
# Example Commands for Testing the Driver 
1. To start the robot driver and controllers, use:
```
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```
2. Install ros2_controller for testing UR3e movement (more detail: https://github.com/ros-controls/ros2_control_demos/tree/foxy)
```
cd (your workspace)
git clone https://github.com/ros-controls/ros2_controllers
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
3. Send the command using demo node
```
ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
```
