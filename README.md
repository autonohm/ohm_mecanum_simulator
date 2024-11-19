# ohm_mecanum_sim
This package comprises a pygame-based robot simulation for mecanum-driven kinematic concepts.

![Screenshot of Robot Simulator](/images/screenshot.png)

Tests have been performed with ROS humble

## Setup
For the here presented simulator you have to install pygame via 
```
pip3 install pygame
```
other requirements should be set up automatically. 


## Starting the simulator
Go to your ROS2 workspace / execute the following commands (replace the path to your workspace accordingly):
```console
user@machine:~$ cd ros2_ws
user@machine:~/ros2_ws$ colcon build --symlink-install
user@machine:~/ros2_ws$ source install/setup.bash
user@machine:~/ros2_ws$ ros2 run ohm_mecanum_sim ohm_mecanum_sim_node
```

## Moving around the robots
One can use different tools to make the robot move:
- Publish joy messages to a certain robot instance (if you have a joystick)
```console
user@machine:~$ ros2 run joy joy_node --ros-args --remap joy:=/robot1/joy
```
- Publish a twist message manually by using rostopic pub, e.g. with a forward movement
```console
user@machine:~$ ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}"
```
- Use a keyboard tool, like teleop_twist_keyboard (install ros-melodic-teleop-twist-keyboard first)
```console
user@machine:~$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot1/cmd_vel
```
- ... or write your own node.
