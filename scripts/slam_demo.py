# ------------------------------------------------------------------------
# Author:      Dong Wang
# Date:        24.09.2024
# Description: SLAM demo for the mecanum simulator
# ------------------------------------------------------------------------
import os
import sys
import pygame
import math
import random
import rospy
from math import cos, sin, pi, sqrt
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class SLAM_Demo:

    def __init__(self, rosname, windowtitle):
        pygame.init()
        rospy.init_node(rosname, anonymous=True)
        self._map_window_name = windowtitle
        self._map_window_size = (900, 900)
        self._meter_to_pixel = 100
        img_path                = os.path.join(os.path.dirname(__file__), "../images/kobuki.png")
        self._symbol            = pygame.image.load(img_path)

        pygame.display.set_caption(self._map_window_name)
        self._map_surface = pygame.display.set_mode(self._map_window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._map_surface.fill((255, 255, 255))
        self._pixelcloud = []
        self._map = []
        self._pose = PoseStamped()
        self._twist = Twist()
        # colors
        self.black = (0, 0, 0)
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self._name = "kobuki"
        self._sub_pose          = rospy.Subscriber(str(self._name)+"/pose", PoseStamped, self.callback_pose)
        self._sub_laser         = rospy.Subscriber(str(self._name)+"/laser", LaserScan, self.callback_laser)
        self._pub_cmd_vel       = rospy.Publisher(str(self._name)+"/cmd_vel", Twist, queue_size=1)
        self._sub_odom          = rospy.Publisher(str(self._name)+"/odom", Odometry, queue_size=1)

    def build_map(self):
        for element in self._pixelcloud:
            if element not in self._map:
                self._map.append(element)
        # draw the map
        self.infomap = self._map_surface.copy()
        if len(self._map) > 1:
            # print("Drawing map")
            for element in self._map:
                # print("Drawing element: " + str(element))
                self.infomap.set_at((int(element[0]), int(element[1])), self.black)
            self._map_surface.blit(self.infomap, (0, 0))


    def callback_pose(self, msg):
        self._pose = msg
        # print("Pose received: " + str(msg))

    def callback_laser(self, msg):
        #convert laserscan to pixel coordinates
        self._laser = msg
        robot_x = self._pose.pose.position.x
        robot_y = self._pose.pose.position.y
        robot_theta = 2 * math.atan2(self._pose.pose.orientation.z, self._pose.pose.orientation.w)
        laser_ranges = msg.ranges
        laser_intensities = msg.intensities
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        laser_points = []
        # Iterate over all laser scan readings
        for i, (range, intensity) in enumerate(zip(laser_ranges, laser_intensities)):
            if  intensity >= 0.5:
                # Calculate the angle of this particular laser beam
                angle = angle_min + i * angle_increment - robot_theta
                
                # Convert from polar to Cartesian coordinates relative to the robot
                x_rel = range * math.cos(angle)
                y_rel = range * math.sin(angle)
                
                # Rotate and translate to global coordinates
                x_global = robot_x + x_rel
                y_global = robot_y - y_rel
                
                # Convert to pixel coordinates (assuming a known scale and map origin)
                pixel_laser = self.transform_to_pixelcoords([x_global, y_global])
                
                laser_points.append(pixel_laser)
        self._pixelcloud = laser_points

    def transform_to_pixelcoords(self, coords):
        pixelcoords  = [ coords[0] * self._meter_to_pixel,
                        (self._map_surface.get_height() - coords[1] * self._meter_to_pixel) ]
        return pixelcoords
    
    def exit_slam(self):
        print("Exit slam")
        sys.exit()

    def run(self):
        running = True
        rate = rospy.Rate(50)
        while running:
            mouse_focused = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    self.exit_slam()
                if pygame.mouse.get_focused():
                    mouse_focused = False
                elif not pygame.mouse.get_focused():
                    mouse_focused = False
            self._map_surface.fill(self.white)

            # draw the robot on the map
            current_position = (self._pose.pose.position.x, self._pose.pose.position.y)
            # todo: check if the latest pose is received with time stamp
            if current_position[0] == 0 and current_position[1] == 0:
                print("No pose received yet")
                continue
            current_pixel_position = self.transform_to_pixelcoords(current_position)
            current_heading = 2 * math.atan2(self._pose.pose.orientation.z, self._pose.pose.orientation.w)
            # print("Current position: " + str(current_position))
            # print("Current pixel: " + str(current_pixel_position) + "heading: " + str(current_heading))
            # draw the robot
            robot_img = pygame.transform.rotozoom(self._symbol,(current_heading-pi/2)*180.0/pi, 0.06)
            robot_rect = robot_img.get_rect()
            robot_rect.center = current_pixel_position
            # robot_rect.move(current_pixel_position) 
            self._map_surface.blit(robot_img, robot_rect)

            # idea to use the mouse to navigate the robot to a goal and do SLAM, but not fully implemented
            if mouse_focused:
                goal = pygame.mouse.get_pos()
                # print("Mouse focused: " + str(goal))

                self._twist.linear.x = 0
                self._twist.linear.y = 0
                self._twist.angular.z = 0

                goal_direction = math.atan2(goal[1] - current_pixel_position[1], goal[0] - current_pixel_position[0])

                # calculate the difference between the goal direction and the current heading
                print("Goal direction: " + str(goal_direction))
                print("Current heading: " + str(current_heading))
                angle_difference = goal_direction + current_heading
                goal_direction_tuple = [math.cos(goal_direction), math.sin(goal_direction)]
                # rotate the robot towards the goal direction
                if abs(angle_difference) > 0.1:
                    if angle_difference > 0:
                        self._twist.linear.x = 0
                        self._twist.linear.y = 0
                        self._twist.angular.z = -0.2
                    else:
                        self._twist.linear.x = 0
                        self._twist.linear.y = 0
                        self._twist.angular.z = 0.2
                else:
                    self._twist.angular.z = 0
                    if math.sqrt(abs(goal[0] - current_pixel_position[0]) + abs(goal[1] - current_pixel_position[1])) < 10:
                        self._twist.linear.x = 0
                        self._twist.linear.y = 0
                    else:
                        # move the robot linearly towards the goal
                        self._twist.linear.x = goal_direction_tuple[0] * 0.5
                        self._twist.linear.y = goal_direction_tuple[1] * 0.5
                self._pub_cmd_vel.publish(self._twist)
            self.build_map()
            pygame.display.update()
            rate.sleep()




