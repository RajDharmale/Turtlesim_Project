#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
import random
import math
import threading
import numpy as np

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz
        
        self.rt_pose = Pose()
        self.rt_velocity_publisher = rospy.Publisher('/rt/cmd_vel', Twist, queue_size=10)
        self.rt_real_pose_publisher = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)
        self.rt_noisy_pose_publisher = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10)
        rospy.Subscriber('/rt/pose', Pose, self.update_rt_pose)
        
        self.pt_pose = Pose()
        self.pt_velocity_publisher = rospy.Publisher('/pt/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/rt_real_pose', Pose, self.update_rt_pose)
        rospy.Subscriber('/pt/pose', Pose, self.update_pt_pose)
        
        self.Kp_linear = 3
        self.Ki_linear = 0.01
        self.Kd_linear = 0.1
        
        self.Kp_angular = 4.0
        
        self.linear_error_prev = 0.0
        self.linear_integral = 0.0
        
        self.max_speed = 100
        self.min_speed = 0.1
        self.chase_complete = False

    def update_rt_pose(self, data):
        self.rt_pose = data

    def update_pt_pose(self, data):
        self.pt_pose = data
    
    def compute_pid(self, error, error_prev, integral, Kp, Ki, Kd):
        integral += error
        derivative = error - error_prev
        output = Kp * error + Ki * integral + Kd * derivative
        return output, integral, error

    def move_rt_in_circle(self, radius=3.2, linear_velocity=2.0):
        twist = Twist()
        while not rospy.is_shutdown():
            twist.linear.x = 2.0
            twist.angular.z = 2.0 / 3.2

            self.rt_velocity_publisher.publish(twist)
            
            self.rt_real_pose_publisher.publish(self.rt_pose)
            if self.chase_complete == True:
                return
            # noisy_pose = Pose()
            # noisy_pose.x = self.rt_pose.x + np.random.normal(0, 10)
            # noisy_pose.y = self.rt_pose.y + np.random.normal(0, 10)
            # noisy_pose.theta = self.rt_pose.theta + np.random.normal(0, 10)
            # self.rt_noisy_pose_publisher.publish(noisy_pose)

    def move_pt_towards_rt(self):
        while not rospy.is_shutdown() and not self.chase_complete:
            distance = math.sqrt((self.rt_pose.x - self.pt_pose.x)**2 + (self.rt_pose.y - self.pt_pose.y)**2)
            if distance <= 0.5:
                rospy.loginfo("Caught! Police turtle caught robber turtle Yay!")
                self.chase_complete = True
                print("distance: ", distance)
                return
            
            angle_to_target = math.atan2(self.rt_pose.y - self.pt_pose.y, self.rt_pose.x - self.pt_pose.x)
            distance_error = distance
            angular_error = angle_to_target - self.pt_pose.theta
            
            if angular_error > math.pi:
                angular_error -= 2 * math.pi
            elif angular_error < -math.pi:
                angular_error += 2 * math.pi
            
            linear_output, self.linear_integral, self.linear_error_prev = self.compute_pid(
                distance_error, self.linear_error_prev, self.linear_integral,
                self.Kp_linear, self.Ki_linear, self.Kd_linear)

            if linear_output > self.max_speed:
                linear_output = self.max_speed
            elif linear_output < self.min_speed:
                linear_output = self.min_speed
            
            self.current_linear_velocity = linear_output
            
            twist = Twist()
            twist.linear.x = linear_output
            twist.angular.z = self.Kp_angular * angular_error
            print("distance: ", distance)
            self.pt_velocity_publisher.publish(twist)
            self.rate.sleep()

    def spawn_turtle(self, name, x, y, theta):
        rospy.wait_for_service('/spawn')
        try:
            spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
            spawn_proxy(x, y, theta, name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':
    try:
        controller = TurtleController()
        controller.spawn_turtle('rt', 5.0, 3.0, 0.0)
        print("spawn rt")
        rt_thread = threading.Thread(target=controller.move_rt_in_circle, args=(5.0, 1.0))
        rt_thread.start()
        print("after start")
        rospy.sleep(5)
        random_x = random.uniform(0, 10)
        random_y = random.uniform(0, 10)
        controller.spawn_turtle('pt', random_x, random_y, 0.0)
        
        pt_thread = threading.Thread(target=controller.move_pt_towards_rt)
        pt_thread.start()
        
        rt_thread.join()
        pt_thread.join()

        
    except rospy.ROSInterruptException:
        pass
