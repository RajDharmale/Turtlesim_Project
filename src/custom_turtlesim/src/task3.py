#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist, Point
import math
import random

class TurtlePIDController:
    def __init__(self):
        rospy.init_node('turtle_pid_controller', anonymous=True)
        self.rate = rospy.Rate(10)  
        self.pose = Pose()
        self.target_pose = Pose()
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.real_pose_publisher = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)
        self.noisy_pose_publisher = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10)
        
        self.Kp_linear = 1.5
        self.Ki_linear = 0
        self.Kd_linear = 0
        
        self.Kp_angular = 4.0
        
        self.linear_error_prev = 0.0
        self.linear_integral = 0.0

        self.max_acceleration = 0.3  
        self.max_deceleration = 0.9  
        self.max_speed = 12.0  
        self.min_speed = 0.1  

        self.current_linear_velocity = 0.0

    def update_pose(self, data):
        self.pose = data

    def compute_pid(self, error, error_prev, integral, Kp, Ki, Kd):
        integral += error
        derivative = error - error_prev
        output = Kp * error + Ki * integral + Kd * derivative
        return output, integral, error

    def move_to_target(self, target_x, target_y):
        distance = math.sqrt((target_x - self.pose.x)**2 + (target_y - self.pose.y)**2)
        
        twist = Twist()
        
        while not rospy.is_shutdown() and distance > 0.1:  
            distance = math.sqrt((target_x - self.pose.x)**2 + (target_y - self.pose.y)**2)
            
            angle_to_target = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
            
            distance_error = distance
            angular_error = angle_to_target - self.pose.theta
            
            if angular_error > math.pi:
                angular_error -= 2 * math.pi
            elif angular_error < -math.pi:
                angular_error += 2 * math.pi
            
            linear_output, self.linear_integral, self.linear_error_prev = self.compute_pid(
                distance_error, self.linear_error_prev, self.linear_integral,
                self.Kp_linear, self.Ki_linear, self.Kd_linear)
            
            acceleration = linear_output - self.current_linear_velocity
            if acceleration > self.max_acceleration:
                linear_output = self.current_linear_velocity + self.max_acceleration
            elif acceleration < -self.max_deceleration:
                linear_output = self.current_linear_velocity - self.max_deceleration

            if linear_output > self.max_speed:
                linear_output = self.max_speed
            elif linear_output < self.min_speed:
                linear_output = self.min_speed
            
            self.current_linear_velocity = linear_output
            if abs(angular_error)>=0.01:
                linear_output = 0
            
            twist.linear.x = linear_output
            twist.linear.y = 0
            twist.linear.z = 0
            
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.Kp_angular * angular_error
            
            self.velocity_publisher.publish(twist)
            
            self.real_pose_publisher.publish(self.pose)
            
            self.rate.sleep()

        twist.linear.x = 0
        twist.angular.z = 0
        self.velocity_publisher.publish(twist)
    
    def move_in_circle(self, radius=1.0, linear_velocity=2.0):
        twist = Twist()
        while not rospy.is_shutdown():
            twist.linear.x = linear_velocity
            twist.angular.z = linear_velocity / radius
            self.velocity_publisher.publish(twist)
            
            self.real_pose_publisher.publish(self.pose)
            
            noisy_pose = Pose()
            noisy_pose.x = self.pose.x + random.gauss(0, 10)
            noisy_pose.y = self.pose.y + random.gauss(0, 10)
            noisy_pose.theta = self.pose.theta + random.gauss(0, 10)
            self.noisy_pose_publisher.publish(noisy_pose)
            
            self.rate.sleep()

    def spawn_new_turtle(self, name, x, y, theta):
        rospy.wait_for_service('/spawn')
        try:
            spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
            spawn_proxy(x, y, theta, name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def delete_previous_turtle(self, name):
        rospy.wait_for_service('/kill')
        try:
            kill_proxy = rospy.ServiceProxy('/kill', Kill)
            kill_proxy(name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        controller = TurtlePIDController()
        rospy.wait_for_service('/kill')
        try:
            kill_proxy = rospy.ServiceProxy('/kill', Kill)
            kill_proxy('turtle1')
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        
        rospy.wait_for_service('/spawn')
        try:
            spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
            spawn_proxy(1.0, 1.0, 0.0, 'turtle1')
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        
        controller.move_to_target(1, 1)
        controller.move_to_target(9, 1)
        controller.move_to_target(9, 3)
        controller.move_to_target(1, 3)
        controller.move_to_target(1, 6)
        controller.move_to_target(9, 6)
        controller.move_to_target(9, 9)
        controller.move_to_target(1, 9)
        
        controller.move_in_circle(radius=3.0, linear_velocity=2.0)

    except rospy.ROSInterruptException:
        pass
