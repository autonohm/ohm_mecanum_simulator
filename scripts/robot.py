# ------------------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Updated:     24.09.2024 by Dong Wang
# Updated:     16.10.2024 by Marco Masannek
# Description: Pygame-based robot representation for the mecanum simulator
# ------------------------------------------------------------------------
import math
import os
import pygame
import rospy
import time, threading
import concurrent.futures
import operator
import numpy as np
import tf
from math import cos, sin, pi, sqrt
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ohm_mecanum_sim.msg import WheelSpeed
from tf.transformations import euler_from_quaternion

use_mrc_config = True

class Robot:

    # Linear velocity in m/s
    _v            = [0, 0]

    # Angular velocity in rad/s
    _omega              = 0

    # Radius of circular obstacle region
    _obstacle_radius = 0.2

    # Angle of facing direction
    #_phi_tof            = [0, pi, pi/2, -pi/2, pi/8, -pi/8, pi+pi/8, pi-pi/8]
    _phi_tof            = []

    # Translation of ToF sensor in facing direction
    #_t_tof              = (0.4, 0.4, 0.2, 0.2, 0.45, 0.45, 0.45, 0.45)        
    _t_tof              = []
    
    # Minimum angle of laser beams (first beam)
    _angle_min = math.radians(-135)

    # Angle increment between beams
    _angle_inc = math.radians(1.0)

    # Number of laser beams
    _laserbeams = 271

    #range of laser
    _laser_range = 8.0

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
    _wheel_omega_max    = 10

    # Center distance between front and rear wheels
    _wheel_base         = 0.3

    # Distance between left and right wheels
    _track              = 0.2

    # Zoomfactor of image representation
    _zoomfactor         = 1.0

    _zoomfactor_kobuki  = 0.06

    # Animation counter, this variable is used to switch image representation to pretend a driving robot
    _animation_cnt      = 0

    def __init__(self, x, y, theta, name):
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
        self._max_omega = self._max_speed / (self._wheel_base/2 + self._track/2)

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
        img_path                = os.path.join(os.path.dirname(__file__), "../images/kobuki.png")
        img_path2               = os.path.join(os.path.dirname(__file__), "../images/kobuki.png")
        img_path_crash          = os.path.join(os.path.dirname(__file__), "../images/mecanum_crash.png")
        self._symbol            = pygame.image.load(img_path)
        self._symbol2           = pygame.image.load(img_path2)
        self._symbol_crash      = pygame.image.load(img_path_crash)
        self._img               = pygame.transform.rotozoom(self._symbol, self._theta, self._zoomfactor_kobuki)
        self._img2              = pygame.transform.rotozoom(self._symbol2, self._theta, self._zoomfactor_kobuki)
        self._img_crash         = pygame.transform.rotozoom(self._symbol_crash, self._theta, self._zoomfactor)
        self._robotrect         = self._img.get_rect()
        self._robotrect.center  = self._coords
        self._sub_twist         = rospy.Subscriber(str(self._name)+"/cmd_vel", Twist, self.callback_twist)
        self._sub_wheelspeed    = rospy.Subscriber(str(self._name)+"/wheel_speed", WheelSpeed, self.callback_wheel_speed)
        self._pub_odom          = rospy.Publisher(str(self._name)+"/odom", Odometry, queue_size=1)


        if use_mrc_config:
            self._pub_laser         = rospy.Publisher(str(self._name)+"/laser", LaserScan, queue_size=1)
        
        else:
          self._sub_joy           = rospy.Subscriber(str(self._name)+"/joy", Joy, self.callback_joy)
          self._pub_pose          = rospy.Publisher(str(self._name)+"/pose", PoseStamped, queue_size=1)
          self._pub_tof           = rospy.Publisher(str(self._name)+"/tof", Float32MultiArray, queue_size=1)

        self._run               = True
        self._thread            = threading.Timer(0.1, self.trigger)
        self._thread.start()
        self._timestamp         = rospy.Time.now()#time.process_time()
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
            timestamp = rospy.Time.now()#time.process_time()
            elapsed = (timestamp - self._timestamp).to_sec()
            self._timestamp = timestamp

            # Check, whether commands arrived recently
            last_command_arrival = timestamp - self._last_command
            if last_command_arrival.to_sec() > 0.5:
                self._v[0] = 0
                self._v[1] = 0
                self._omega = 0

            # Change orientation
            self._theta += self._omega * elapsed

            # Transform velocity vectors to global frame
            cos_theta = math.cos(self._theta)
            sin_theta = math.sin(self._theta)
            v =   [self._v[0], self._v[1]]
            v[0] = cos_theta*self._v[0] - sin_theta * self._v[1]
            v[1] = sin_theta*self._v[0] + cos_theta * self._v[1]

            # Move robot
            self._coords[0] += v[0]  * elapsed
            self._coords[1] += v[1]  * elapsed

            # Publish pose
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = self._timestamp
            # hard coded offset of 2m in x and y direction to match the intial position of the robot
            # todo: make this configurable
            p.pose.position.x = self._coords[0] - 2
            p.pose.position.y = self._coords[1] - 2
            p.pose.position.z = 0
            p.pose.orientation.w = math.cos(self._theta/2.0)
            p.pose.orientation.x = 0
            p.pose.orientation.y = 0
            p.pose.orientation.z = math.sin(self._theta/2.0)
            
            if not use_mrc_config:
              self._pub_pose.publish(p)

            # Publish odometry
            o = Odometry()
            o.header.frame_id ="odom"
            o.header.stamp = self._timestamp
            o.pose.pose.position = p.pose.position
            o.pose.pose.orientation = p.pose.orientation
            o.child_frame_id = "base_link"
            o.twist.twist.linear.x = v[0]
            o.twist.twist.linear.y = v[1]
            o.twist.twist.angular.z = self._omega
            # Add covariance
            o.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                0, 0.01, 0, 0, 0, 0,
                                0, 0, 0.01, 0, 0, 0,
                                0, 0, 0, 0.01, 0, 0,
                                0, 0, 0, 0, 0.01, 0,
                                0, 0, 0, 0, 0, 0.01]
            self._pub_odom.publish(o)

            # Publish TF odom to base_link
            br = tf.TransformBroadcaster()
            br.sendTransform((self._coords[0] - 2, self._coords[1] -2, 0),
                             tf.transformations.quaternion_from_euler(0, 0, self._theta),
                             self._timestamp,
                             "base_link",
                             "odom")
            
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

    # Bresenham's line algorithm to calculate all points between two points
    def bresenham_line(self, x0, y0, x1, y1, step = 3):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        count = 0  # Initialize a counter to track steps
        while True:
            # Only add point to list every 'step' intervals to reduce the number of points
            if count % step == 0:
                points.append((x0, y0))
            count += 1
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points
    # LiDAR sensing calculation for a single angle
    def calculate_lidar_angle(self, index, angle, robot_pose, map, meter_to_pixel=100):
        # distances = []
        r_x, r_y, _ = robot_pose
        min_range = self._obstacle_radius
        # Calculate laser end point (max range of laser)
        x2 = int(r_x + self._laser_range * meter_to_pixel * math.cos(angle))
        y2 = int(r_y + self._laser_range * meter_to_pixel * math.sin(angle))
        
        # Calculate minimum range start point
        x1 = int(min_range * meter_to_pixel * math.cos(angle) + r_x)
        y1 = int(min_range * meter_to_pixel * math.sin(angle) + r_y)
        
        # Get all points along the laser beam using Bresenham's line algorithm
        points_on_line = self.bresenham_line(x1, y1, x2, y2)
        # Iterate over the points and check for obstacles
        distance = self._laser_range
        for x, y in points_on_line:
            if 0 < x < map.get_width() and 0 < y < map.get_height():
                color = map.get_at((x, y))
                
                # If an obstacle (black or red), calculate distance and break
                if (color[0], color[1], color[2]) == (0, 0, 0) or (color[0], color[1], color[2]) == (255, 0, 0):
                    obstacle = math.sqrt((x - r_x) ** 2 + (y - r_y) ** 2)
                    distance = obstacle / meter_to_pixel
                    break
                map.set_at((x, y), (0, 208, 255))
        return index, distance

    # Main LiDAR sensing function using multithreading
    def LiDAR_sensing(self, robot_pose, map, num_threads=2):
        distances = []
        r_heading = robot_pose[2]
        angle_min = self._angle_min
        angle_max = self._angle_max
        angle_inc = self._angle_inc
        start_angle = -r_heading + angle_min
        end_angle = -r_heading + angle_max
        
        # List to store results in the correct order
        angles = np.arange(end_angle, start_angle, -angle_inc)
        num_angles = len(angles)
        distances = [0] * num_angles

        ### using a defined number of threads may be faster than using all available threads 
        # Define a helper function to process a range of angles

        # def process_angles(start_idx, end_idx):
        #     for i in range(start_idx, end_idx):
        #         angle = angles[i]
        #         distances[i] = self.calculate_lidar_angle(i, angle, robot_pose, map)
                
        # # Using ThreadPoolExecutor for multithreading
        # # Divide angles among threads: deprecated due to GIL (use a defined number of threads)

        # with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
        #     step = num_angles // num_threads
        #     futures = [
        #         executor.submit(process_angles, i * step, (i + 1) * step)
        #         for i in range(num_threads)
        #     ]
        #     # Collect the results
        #     concurrent.futures.wait(futures)

        # use maximal threads 
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Submit a thread for each angle calculation
            futures = [
                executor.submit(self.calculate_lidar_angle, i, angle, robot_pose, map)
                for i, angle in enumerate(angles)
            ]
            # Collect the results as they complete
            for future in concurrent.futures.as_completed(futures):
                index, distance = future.result()
                distances[index] = distance
        return distances    
    
    def publish_LiDAR(self, distances):
        if not use_mrc_config:
            exit()
        scan = LaserScan()  
        scan.header.stamp = self._timestamp
        scan.header.frame_id = "laser"
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_inc
        scan.time_increment = 1.0/5000.0
        scan.scan_time = 1.0/500.0
        scan.range_min = 0.0
        scan.range_max = self._laser_range
        scan.ranges = []
        scan.intensities = []
        for i in range(0, len(distances)):
            # scan.ranges.append(distances[i])
            if distances[i] < self._laser_range:
                # scan.ranges.append(distances[i]+ self._lasernoise*np.random.randn())
                scan.ranges.append(distances[i])
                scan.intensities.append(1)
            else:
                scan.ranges.append(distances[i] )
                scan.intensities.append(0)

        self._pub_laser.publish(scan)
         
    def get_coords(self):
        return self._coords
    
    def get_heading(self):
        return self._theta
    
    def get_rect(self):
        self._img       = pygame.transform.rotozoom(self._symbol,       (self._theta-pi/2)*180.0/pi, self._zoomfactor_kobuki)
        self._img2      = pygame.transform.rotozoom(self._symbol2,      (self._theta-pi/2)*180.0/pi, self._zoomfactor_kobuki)
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
        self._last_command = rospy.Time.now()

    def callback_joy(self, data):
        self.set_velocity(data.axes[1]*self._max_speed, data.axes[0]*self._max_speed, data.axes[2]*self._max_omega)
        self._last_command = rospy.Time.now()

    def callback_wheel_speed(self, data):
        omega = [data.w_front_left, data.w_front_right, data.w_rear_left, data.w_rear_right]
        self.set_wheel_speed(omega);
        self._last_command = rospy.Time.now()

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
