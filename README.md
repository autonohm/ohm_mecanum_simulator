# ohm_mecanum_sim
This package comprises a pygame-based robot simulation for mecanum-driven kinematic concepts and is extended for kobuki robot.

<!-- <img src="./images/demo_slam.png"  width="200" height="200"> -->

# How to install 

## Dependencies

### Ubuntu 20.04 (recommended)
```
sudo apt install python3-pip
pip install pygame
```
### Ubuntu 18.04
Tests have been performed with ROS melodic, albeit the used python versions differ. While ROS melodic uses python2.x, the simulator need python3. The  reason is, that ROS noetic will require python3. In order to make the simulator work with ROS melodic, you can follow the installation hints below.

### Prerequisites on Ubuntu 18.04
Installing python3 aside python2 can be done with pip:
```console
user@machine:~$ sudo apt install python3 python-pip python3-pip python-catkin-tools
user@machine:~$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
user@machine:~$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2
user@machine:~$ pip install -U defusedxml rospkg pygame pycryptodomex gnupg pyside2 pydot numpy
```

## Building the package

Go to your catkin workspace and build the package. / execute the following commands (replace the path to your catkin workspace accordingly):

### Catkin Make

```console
user@machine:~$ cd ~/catkin_ws
user@machine:~/catkin_ws$ catkin_make
user@machine:~/catkin_ws$ source devel/setup.bash
```

### Catkin Build

```console
user@machine:~$ cd ~/catkin_ws
user@machine:~/catkin_ws$ catkin build ohm_mecanum_sim
user@machine:~/catkin_ws$ source devel/setup.bash
```

# Using the simulator

## Start the default launchfile

If you installed all dependencies and built the package without any errors,
you should be able to launch the simulator using the following command:

```console
user@machine:~/catkin_ws$ roslaunch ohm_mecanum_sim simulator.launch
```

## Expected output

A window should pop up that displays the environment, the robot and ray-traced laser beams:

<img src="./images/kobuki_demo.png"  width="300" height="300">

## Available Topics

| Name | In/Out  | Type  |
|---|---|---|
| /kobuki/cmd_vel       | In    | geometry_msgs/Twist           | 
| /kobuki/laser         | Out   | Sensor_msgs/LaserScan         | 
| /kobuki/odom          | Out   | nav_msgs/Odometry             | 
| /kobuki/wheel_speed   | Out   | ohm_mecanum_sim/WheelSpeed    | 

# Controlling the robot and using sensor data

## Moving the robot manually

The simulator listens to velocity commands, as published by e.g. the [teleop_twist_keyboard](https://wiki.ros.org/teleop_twist_keyboard) tool.
To publish them manually, you can use the following command:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/kobuki/cmd_vel
```

## Creating a map

We provide a [Gmapping](https://wiki.ros.org/gmapping) configuration for our simulator to create a map of the environment.

```
roslaunch ohm_mecanum_sim kobuki_gmapping.launch
```

This command should open up an RVIZ window with our view that shows the mapping progress:

<img src="./images/gmapping_demo.png"  width="300" height="300">



# Navigation (REMOVE)
Using [Move Base](https://wiki.ros.org/move_base) to navigate the robot and [AMCL](https://wiki.ros.org/amcl) to localize the robot on the map:
```
roslaunch ohm_mecanum_sim kobuki_navigation.launch
```

<img src="./images/navigation_demo.png"  width="300" height="300">
