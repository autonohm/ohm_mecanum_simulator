#!/usr/bin/env python3

# ------------------------------------------------------------------------
# Author:      Stefan May
# Date:        01.05.2024
# Description: Pygame-based robot representation for the mecanum simulator
# ------------------------------------------------------------------------

import os
from glob import glob
import pygame
import rclpy
from rclpy.node import Node
import time, threading
import operator
import numpy as np
from math import cos, sin, pi, sqrt
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class Robot(Node):

    # Linear velocity in m/s
    _v            = [0, 0]

    # Angular velocity in rad/s
    _omega              = 0.0

    # Radius of circular obstacle region
    _obstacle_radius = 0.45

    # Angle of facing direction
    #_phi_tof            = [0, pi, pi/2, -pi/2, pi/8, -pi/8, pi+pi/8, pi-pi/8]
    _phi_tof            = []

    # Translation of ToF sensor in facing direction
    #_t_tof              = (0.4, 0.4, 0.2, 0.2, 0.45, 0.45, 0.45, 0.45)        
    _t_tof              = []
    
    # Minimum angle of laser beams (first beam)
    _angle_min = -180*pi/180

    # Angle increment between beams
    _angle_inc = 10*pi/180

    # Number of laser beams
    _laserbeams = 36

    # Gaussian Noise in lidar distance in meters
    _lasernoise = 0.02

    # Facing directions of ToF sensors
    _v_face             = []

    # Positions of ToF sensors
    _pos_tof            = []

    # Point along line of sight in the farest distance
    _far_tof            = []
    
    # Range of ToF sensors
    _rng_tof            = 8.0

    # Offset of ToF sensors from the kinematic centre
    _offset_tof         = 0.2

    # Radius of wheels
    _wheel_radius       = 0.05

    # Maximum angular rate of wheels in rad/s
    _wheel_omega_max    = 10.0

    # Center distance between front and rear wheels
    _wheel_base         = 0.3

    # Distance between left and right wheels
    _track              = 0.2

    # Zoomfactor of image representation
    _zoomfactor         = 1.0

    # Animation counter, this variable is used to switch image representation to pretend a driving robot
    _animation_cnt      = 0

    def __init__(self, x, y, theta, name, callback_group):
        super().__init__(name)
        self._initial_coords = [x, y]
        self._initial_theta  = theta
        self._reset = False
        self._coords = [x, y]
        self._theta = theta
        self._lock = threading.Lock()

        # Matrix of kinematic concept
        lxly = (self._wheel_base/2 + self._track/2) / self._wheel_radius
        rinv = 1/self._wheel_radius
        self._T = np.matrix([[rinv, -rinv, -lxly],
                            [-rinv, -rinv, -lxly],
                            [ rinv,  rinv, -lxly],
                            [-rinv,  rinv, -lxly]])
        # Inverse of matrix is used for setting individual wheel speeds
        self._Tinv = np.linalg.pinv(self._T)

        # Calculate maximum linear speed in m/s
        self._max_speed = self._wheel_omega_max * self._wheel_radius

        # Calculate maximum angular rate of robot in rad/s
        self._max_omega = self._max_speed / (self._wheel_base/2.0 + self._track/2.0)

        self._angle_max = self._angle_min+(self._laserbeams-1)*self._angle_inc
        if(self._angle_max != -self._angle_min):
            print("Warning: laserbeams should be symmetric. angle_min = " + str(self._angle_min) + ", angle_max = " + str(self._angle_max))
        for i in range(0, self._laserbeams):
            self._phi_tof.append(i*self._angle_inc+self._angle_min)
            self._t_tof.append(self._offset_tof)


        for i in range(0, len(self._phi_tof)):
            self._v_face.append((0,0))
            self._pos_tof.append((0,0))
            self._far_tof.append((0,0))

        self._name              = name
        img_path                = os.path.join(os.path.dirname(__file__), "../../../install/ohm_mecanum_sim/share/ohm_mecanum_sim/images/mecanum_ohm_1.png")
        img_path2               = os.path.join(os.path.dirname(__file__), "../../../install/ohm_mecanum_sim/share/ohm_mecanum_sim/images/mecanum_ohm_2.png")
        img_path_crash          = os.path.join(os.path.dirname(__file__), "../../../install/ohm_mecanum_sim/share/ohm_mecanum_sim/images/mecanum_crash_2.png")
        self._symbol            = pygame.image.load(img_path)
        self._symbol2           = pygame.image.load(img_path2)
        self._symbol_crash      = pygame.image.load(img_path_crash)
        self._img               = pygame.transform.rotozoom(self._symbol, self._theta, self._zoomfactor)
        self._img2              = pygame.transform.rotozoom(self._symbol2, self._theta, self._zoomfactor)
        self._img_crash         = pygame.transform.rotozoom(self._symbol_crash, self._theta, self._zoomfactor)
        self._robotrect         = self._img.get_rect()
        self._robotrect.center  = self._coords
        self._sub_twist         = self.create_subscription(Twist, str(self._name)+"/cmd_vel", self.callback_twist, 1)
        self._sub_joy           = self.create_subscription(Joy, str(self._name)+"/joy", self.callback_joy, 1)
        self._pub_pose          = self.create_publisher(PoseStamped, str(self._name)+"/pose", 1)
        self._pub_odom          = self.create_publisher(Odometry, str(self._name)+"/odom", 1)
        self._pub_tof           = self.create_publisher(Float32MultiArray, str(self._name)+"/tof", 1)
        self._pub_laser         = self.create_publisher(LaserScan, str(self._name)+"/laser", 1)

        self.tf_broadcaster     = TransformBroadcaster(self)

        self._run               = True
        self._thread            = threading.Timer(0.1, self.trigger)
        self._thread.start()
        self._timestamp         = self.get_clock().now()
        self._last_command      = self._timestamp

    def __del__(self):
        self.stop()

    def reset_pose(self):
        self._reset = True

    def set_max_velocity(self, vel):
        self._max_speed = vel

    def set_wheel_speed(self, omega_wheel):
        w = np.array([omega_wheel[0], omega_wheel[1], omega_wheel[2], omega_wheel[3]])
        res = self._Tinv.dot(w)
        self.set_velocity(res[0,0], res[0,1], res[0,2])

    def set_velocity(self, vx, vy, omega):
        x = np.array([vx, vy, omega])
        omega_i = self._T.dot(x)
        self._v = [vx, vy]
        self._omega = omega

    def acquire_lock(self):
        self._lock.acquire()

    def release_lock(self):
        self._lock.release()

    def stop(self):
        self.set_velocity(0, 0, 0)
        self._run = False

    def trigger(self):
        while(self._run):
            self.acquire_lock()

            # Measure elapsed time
            timestamp = self.get_clock().now()
            elapsed = (timestamp - self._timestamp).nanoseconds * 1e-9
            self._timestamp = timestamp

            # Check, whether commands arrived recently
            last_command_arrival = timestamp - self._last_command
            if last_command_arrival.nanoseconds * 1e-9 > 0.5:
                self._v[0] = 0.0
                self._v[1] = 0.0
                self._omega = 0.0

            # Change orientation
            self._theta += self._omega * elapsed

            # Transform velocity vectors to global frame
            cos_theta = cos(self._theta)
            sin_theta = sin(self._theta)
            v =   [self._v[0], self._v[1]]
            v[0] = cos_theta*self._v[0] - sin_theta * self._v[1]
            v[1] = sin_theta*self._v[0] + cos_theta * self._v[1]

            # Move robot
            self._coords[0] += v[0]  * elapsed
            self._coords[1] += v[1]  * elapsed

            # # Publish pose
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = self._timestamp.to_msg()
            p.pose.position.x = self._coords[0]
            p.pose.position.y = self._coords[1]
            p.pose.position.z = 0.0
            p.pose.orientation.w = cos(self._theta/2.0)
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = sin(self._theta/2.0)
            self._pub_pose.publish(p)

            t = TransformStamped()
            t.header.stamp = self._timestamp.to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = p.pose.position.x
            t.transform.translation.y = p.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = p.pose.orientation.x
            t.transform.rotation.y = p.pose.orientation.y
            t.transform.rotation.z = p.pose.orientation.z
            t.transform.rotation.w = p.pose.orientation.w

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

            # Publish odometry
            o = Odometry()
            o.header.stamp = self._timestamp.to_msg()
            o.header.frame_id ="map"
            o.child_frame_id = "base_link"
            o.pose.pose.position = p.pose.position
            o.pose.pose.orientation = p.pose.orientation
            o.twist.twist.linear.x = v[0];
            o.twist.twist.linear.y = v[1];
            o.twist.twist.angular.z = self._omega;
            self._pub_odom.publish(o)

            if(self._reset):
                time.sleep(1.0)
                self._coords[0] = self._initial_coords[0]
                self._coords[1] = self._initial_coords[1]
                self._theta  = self._initial_theta
                self._reset = False

            self.release_lock()
            time.sleep(0.04)

    def publish_tof(self, distances):
        msg = Float32MultiArray(data=distances)
        self._pub_tof.publish(msg)

        scan = LaserScan()
        scan.header.stamp = self._timestamp.to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_inc
        scan.time_increment = 1.0/50.0
        scan.range_min = 0.0
        scan.range_max = self._rng_tof
        scan.ranges = []
        scan.intensities = []
        for i in range(0, self._laserbeams):
            scan.ranges.append(distances[i] + self._lasernoise*np.random.randn())
            scan.intensities.append(1)
        self._pub_laser.publish(scan)

    def get_coords(self):
        return self._coords

    def get_rect(self):
        self._img       = pygame.transform.rotozoom(self._symbol,       (self._theta-pi/2)*180.0/pi, self._zoomfactor)
        self._img2      = pygame.transform.rotozoom(self._symbol2,      (self._theta-pi/2)*180.0/pi, self._zoomfactor)
        self._img_crash = pygame.transform.rotozoom(self._symbol_crash, (self._theta-pi/2)*180.0/pi, self._zoomfactor)
        self._robotrect = self._img.get_rect()
        return self._robotrect

    def get_image(self):
        if(not self._reset):
            self._animation_cnt += 1
        magnitude = abs(self._v[0])
        if(abs(self._v[1]) > magnitude):
            magnitude = abs(self._v[1])
        if(abs(self._omega)>magnitude):
            magnitude = abs(self._omega)
        if magnitude < 0.5:
            moduloval = 6
        else:
            moduloval = 2
        
        if(self._reset):
            return self._img_crash
        elif(self._animation_cnt % moduloval < moduloval/2 and (self._v[0]!=0 or self._v[1]!=0 or self._omega!=0)):
            return self._img
        else:
            return self._img2

    def get_obstacle_radius(self):
        return self._obstacle_radius

    def get_tof_count(self):
        return len(self._phi_tof)

    def get_pos_tof(self):
        v_face = self.get_facing_tof()
        for i in range(0, len(self._phi_tof)):
            self._pos_tof[i]    = (self._coords[0]+v_face[i][0]*self._t_tof[i],
                                   self._coords[1]+v_face[i][1]*self._t_tof[i])
        return self._pos_tof

    def get_tof_range(self):
        return self._rng_tof

    def get_far_tof(self):
        v_face = self.get_facing_tof()
        for i in range(0, len(self._phi_tof)):
            self._far_tof[i]    = (self._coords[0]+v_face[i][0]*(self._t_tof[i]+self._rng_tof),
                                   self._coords[1]+v_face[i][1]*(self._t_tof[i]+self._rng_tof))
        return self._far_tof

    def get_hit_tof(self, dist):
        v_face = self.get_facing_tof()
        for i in range(0, len(self._phi_tof)):
            d = dist[i]
            if(d<0):
                d = self._rng_tof
            self._far_tof[i]    = (self._coords[0]+v_face[i][0]*d,
                                   self._coords[1]+v_face[i][1]*d)
        return self._far_tof

    def get_facing_tof(self):
        i = 0
        for phi in self._phi_tof:
            cos_theta = cos(self._theta+phi)
            sin_theta = sin(self._theta+phi)
            self._v_face[i] = [cos_theta*1.0 - sin_theta*0.0,
                               sin_theta*1.0 + cos_theta*0.0]
            i += 1
        return self._v_face

    def get_distance_to_line_obstacle(self, start_line, end_line, dist_to_obstacles):
        if(len(dist_to_obstacles)!=len(self._phi_tof)):
            for i in range(0, len(self._phi_tof)):
                dist_to_obstacles.append(self._rng_tof)
        pos_tof = self.get_pos_tof()
        far_tof = self.get_far_tof()
        for i in range(0, len(self._phi_tof)):
            dist = self.line_line_intersection(start_line, end_line, pos_tof[i], far_tof[i])+self._t_tof[i]
            if(dist<dist_to_obstacles[i] and dist>0):
                dist_to_obstacles[i] = dist
        return dist_to_obstacles

    def get_distance_to_circular_obstacle(self, pos_obstacle, obstacle_radius, dist_to_obstacles):
        if(len(dist_to_obstacles)!=len(self._phi_tof)):
            for i in range(0, len(self._phi_tof)):
                dist_to_obstacles.append(self._rng_tof)
        pos_tof = self.get_pos_tof()
        far_tof = self.get_far_tof()
        for i in range(0, len(self._phi_tof)):
            dist = self.circle_line_intersection(pos_obstacle, obstacle_radius, pos_tof[i], far_tof[i])
            if(dist<dist_to_obstacles[i] and dist>0):
                dist_to_obstacles[i] = dist
        return dist_to_obstacles

    def callback_twist(self, data):
        self.set_velocity(data.linear.x, data.linear.y, data.angular.z)
        self._last_command = self.get_clock().now()

    def callback_joy(self, data):
        self.set_velocity(data.axes[1]*self._max_speed, data.axes[0]*self._max_speed, data.axes[2]*self._max_omega)
        self._last_command = self.get_clock().now()

    def line_length(self, p1, p2):
        return sqrt( (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) )
        
    def line_line_intersection(self, start_line, end_line, coords_sensor, coords_far):

        def line(p1, p2):
            A = (p1[1] - p2[1])
            B = (p2[0] - p1[0])
            C = (p1[0]*p2[1] - p2[0]*p1[1])
            return A, B, -C

        def intersection(L1, L2):
            D  = L1[0] * L2[1] - L1[1] * L2[0]
            Dx = L1[2] * L2[1] - L1[1] * L2[2]
            Dy = L1[0] * L2[2] - L1[2] * L2[0]
            if D != 0:
                x = Dx / D
                y = Dy / D
                return x,y
            else:
                return False

        def dot_product(p1, p2):
            return p1[0]*p2[0]+p1[1]*p2[1]

        L1 = line(start_line, end_line)
        L2 = line(coords_sensor, coords_far)

        coords_inter = intersection(L1, L2)

        if(coords_inter):
            v1 = tuple(map(operator.sub, coords_inter, coords_sensor))
            v2 = tuple(map(operator.sub, coords_inter, coords_far))
            dot1 = dot_product(v1, v2)
            v1 = tuple(map(operator.sub, coords_inter, start_line))
            v2 = tuple(map(operator.sub, coords_inter, end_line))
            dot2 = dot_product(v1, v2)
            if(dot1>=0 or dot2>=0):
                return -1
            else:
                return self.line_length(coords_inter, coords_sensor)
        else:
            return -1
        
    def circle_line_intersection(self, coords_obstacle, r, coords_sensor, coords_far):
        # Shift coordinate system, so that the circular obstacle is in the origin
        x1c = coords_sensor[0] - coords_obstacle[0]
        y1c = coords_sensor[1] - coords_obstacle[1]
        x2c = coords_far[0] - coords_obstacle[0]
        y2c = coords_far[1] - coords_obstacle[1]

        # ----------------------------------------------------------
        # Calculation of intersection points taken from:
        # https://mathworld.wolfram.com/Circle-LineIntersection.html
        # ----------------------------------------------------------
        dx = x2c - x1c
        dy = y2c - y1c
        dr = sqrt(dx*dx + dy*dy)

        # Determinant
        det = x1c * y2c - x2c * y1c

        dist = -1
        if(dy<0):
            sgn = -1
        else:
            sgn = 1
        lam = r*r*dr*dr-det*det

        v_hit = [0,0]
        if(lam > 0):
            s = sqrt(lam)
            # Coordinates of intersection
            coords_inter1 = [(det * dy + sgn * dx * s) / (dr*dr) + coords_obstacle[0], (-det * dx + abs(dy) * s) / (dr*dr) + coords_obstacle[1]]
            coords_inter2 = [(det * dy - sgn * dx * s) / (dr*dr) + coords_obstacle[0], (-det * dx - abs(dy) * s) / (dr*dr) + coords_obstacle[1]]

            # The closest distance belongs to the visible surface
            dist1 = self.line_length(coords_inter1, coords_sensor)
            dist2 = self.line_length(coords_inter2, coords_sensor)

            if(dist1<dist2):
                dist = dist1
                v_hit = tuple(map(operator.sub, coords_inter1, coords_sensor))
            else:
                dist = dist2
                v_hit = tuple(map(operator.sub, coords_inter2, coords_sensor))

        # If the dot product is not equal 0, the intersection lays behind us
        v_face = tuple(map(operator.sub, coords_far, coords_sensor))
        dot = v_face[0]*v_hit[0]+v_face[1]*v_hit[1]

        if(dist> 0 and dot>0):
            return dist
        else:
            return -1
