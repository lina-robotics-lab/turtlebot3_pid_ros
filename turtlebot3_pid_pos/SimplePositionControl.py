#!/usr/bin/env python3

import math
from math import cos, sin
import time
import csv
import datetime
import os
import pandas as pd
FEEDBACK_P = 3.5
FEEDBACK_I = 0.3
FEEDBACK_D = 0.6
CONTROLLER_FREQ = 30
DIST = 0.3
LINEAR_VELO_MAX = 0.2


from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Float32
from rclpy.node import Node
from collections import deque
from rclpy.qos import QoSProfile
import rclpy
import numpy as np

def simple_truncate(control, upper, lower):
    return min(max(control, lower), upper)

def stop_twist():
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    return twist
    def __init__(self) -> None:
        self.last_dist = 100.0 # a large value that must be larger than the first distance
        self._last_angle_error = 0.0
        self._integral_angle_error = 0.0

    def turn(self, angle_error, angular_velocity, step):
        twist = Twist()
        if angle_error <= -math.pi:
            angle_error = angle_error + 2 * math.pi
        fbk_anglar_vel = simple_truncate(FEEDBACK_P*math.fabs(angle_error), angular_velocity, 0.1*angular_velocity)
        if math.fabs(angle_error) > 0.01:  # 0.01 is small enough value
            if angle_error >= math.pi:
                twist.angular.z = -fbk_anglar_vel
            elif math.pi > angle_error and angle_error >= 0:
                twist.angular.z = fbk_anglar_vel
            elif 0 > angle_error and angle_error >= -math.pi:
                twist.angular.z = -fbk_anglar_vel
            elif angle_error > -math.pi:
                twist.angular.z = fbk_anglar_vel
        else:
            time.sleep(0.5)
            step += 1

        return twist, step

    def go_straight(self, distance, angle_error, linear_velocity, angular_velocity, step):
        twist = Twist()
        if distance > 0.03:  # 0.01 is small enough value
            twist.linear.x = simple_truncate(FEEDBACK_P * distance, linear_velocity, 0.5 * linear_velocity)
            self._integral_angle_error += angle_error
            p_out = FEEDBACK_P * angle_error
            i_out = FEEDBACK_I * self._integral_angle_error
            d_out = FEEDBACK_D * (angle_error - self._last_angle_error)
            print(p_out, i_out, d_out)
            self._last_angle_error = angle_error
            twist.angular.z = simple_truncate(p_out+i_out+d_out, angular_velocity, -angular_velocity)
        else:
            step += 1
        self.last_dist = distance
        return twist, step
    
class SimplePIDController(object):

    def __init__(self) -> None:
        self.last_dist = 100.0 # a large value that must be larger than the first distance
        self._last_distance_error = None
        self._integral_distance_error = 0.0
        self.handle_init_lag = False

    def control(self, distance_error):
        twist = Twist()
        self._integral_distance_error += distance_error
        if self._last_distance_error is None:
            self._last_distance_error = distance_error
        p_out = FEEDBACK_P * distance_error
        i_out = FEEDBACK_I * self._integral_distance_error
        d_out = FEEDBACK_D * (distance_error - self._last_distance_error) * CONTROLLER_FREQ
        if (not self.handle_init_lag) and (np.abs(distance_error - self._last_distance_error) < 0.001):
            self._integral_distance_error = 0.0
        elif (not self.handle_init_lag) and (np.abs(distance_error - self._last_distance_error) > 0.001):
            self.handle_init_lag = True
            print('handle init lag')
        # print(p_out, i_out, d_out)
        # print(np.abs(distance_error - self._last_distance_error))
        twist.linear.x = simple_truncate(p_out + i_out + d_out, LINEAR_VELO_MAX, -1. * LINEAR_VELO_MAX)
        self._last_distance_error = distance_error
        twist.angular.z = 0.
        return twist

def posestmp2xy(pose):
    return np.array([pose.pose.position.x,pose.pose.position.y])
def posestmp2yaw(pose):
    return quaternion2yaw(pose.pose.orientation)

def quaternion2yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

    
class pid_control(Node):
    
    def __init__(self, robot_namespace):
        super().__init__(node_name='pid', namespace=robot_namespace)
        self.robot_pose_stack = deque(maxlen=10)
        # pose_type_string=PoseStamped
        qos = QoSProfile(depth=10)
        rpose_topic="/vrpn_client_node/{}/pose".format(robot_namespace)
        self.create_subscription(PoseStamped, rpose_topic, self.robot_pose_callback_, qos)
        # time.sleep(5.)
        # init_loc = None
        # init_yaw = self.get_yaw()
        self.target_loc = np.empty([2,])
        self.target_set = False
        # self.get_logger().info('target: {}'.format(self.target_loc))
        
        self.vel_pub = self.create_publisher(Twist, '/{}/cmd_vel'.format(robot_namespace), qos)
        self.dist_pub = self.create_publisher(Float32, "/{}/pos_error".format(robot_namespace), qos)
        self.controller = SimplePIDController()
        self.create_timer(float(1/CONTROLLER_FREQ), self.pid_callback)
        self.data = {
            'time':[],
            'pos_error':[]
        }
        self.init_time_fix = False
        

    def robot_pose_callback_(self,data):
        # print(self.get_loc())
        self.robot_pose_stack.append(data)
        
    def get_loc(self):
        if len(self.robot_pose_stack)>0:
            return posestmp2xy(self.robot_pose_stack[-1])
        else:
            return None
    
    def get_yaw(self):
        if len(self.robot_pose_stack) > 0:
            return posestmp2yaw(self.robot_pose_stack[-1])
        else:
            return None

    def pid_callback(self):
        if self.get_loc() is None:
            pass
        else:
            if not self.target_set:
                init_loc = self.get_loc()
                init_yaw = self.get_yaw()
                self.get_logger().info('loc: {}'.format(init_loc))
                self.get_logger().info('yaw: {}'.format(init_yaw))  
                self.target_loc = np.array(init_loc) + DIST * np.array([cos(init_yaw), sin(init_yaw)])
                self.get_logger().info('set target loc: {}'.format(self.target_loc))
                self.target_set = True
                self.get_logger().info(str(self.get_clock().now()))
                
            loc = self.get_loc()
            distance_error = np.sign(loc[0] - self.target_loc[0]) * np.sqrt((loc - self.target_loc)[0] ** 2 + 
                                                                            (loc - self.target_loc)[1] ** 2)
            # print(loc, distance_error)
            print(np.sign(loc[0] - self.target_loc[0]), np.linalg.norm(loc - self.target_loc))
            twist = self.controller.control(distance_error)     
            self.vel_pub.publish(twist)
            pos_error_msg = Float32()
            pos_error_msg.data = distance_error
            self.dist_pub.publish(pos_error_msg)
            if not self.init_time_fix:
                if self.controller.handle_init_lag:
                    self.init_time = time.time()
                    self.init_time_fix = True
            if self.init_time_fix:
                self.data['time'].append(time.time() - self.init_time)
                self.data['pos_error'].append(distance_error)

    def save_data(self):

        # Get the current time
        current_time = datetime.datetime.now()

        # Format the current time as a string without special characters
        time_str = current_time.strftime("%Y-%m-%d_%H-%M-%S")

        # Specify the file name using the formatted current time
        file_name = f"data_{time_str}_DIST_{DIST}_P_{FEEDBACK_P}_I_{FEEDBACK_I}_D_{FEEDBACK_D}.csv"
        dir = '/home/mht/turtlebot3_ws/src/turtlebot3_pid_pos/turtlebot3_pid_pos/data'
        file_name = os.path.join(dir, file_name)
        # Write data to the CSV file
        df = pd.DataFrame.from_dict(self.data)
        df.to_csv(file_name, index=False)

        print(f"Dictionary has been written to '{file_name}' in CSV format.")


def main():
    rclpy.init()
    robot_namespace = 'MobileSensor3'
    pid = pid_control(robot_namespace)
    try:
        print('pid controller')
        rclpy.spin(pid)
    except KeyboardInterrupt:
        print("Keyboard Interrupt. Shutting Down...")
        for _ in range(10):# Publish consecutive stop twist for 3 seconds to ensure the robot steps.
            pid.vel_pub.publish(stop_twist())
            time.sleep(0.1)
        pid.save_data()
    finally:
        pid.destroy_node()
        print('Distributed Seeking Node Down')
        rclpy.shutdown()

if __name__ == '__main__':
	main()


        
        
        