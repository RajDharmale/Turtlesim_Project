#!/usr/bin/env python3

import rospy
import math
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import threading
import numpy as np

target_pos_x = 0.0
target_pos_y = 0.0
chaser_pos_x = 0.0
chaser_pos_y = 0.0
chaser_orientation = 0.0 
separation_distance = 0

velocity_publisher = None

CHASE_DISTANCE_THRESHOLD = 3.0
TARGET_LINEAR_SPEED = 1.0

class TurtleController:
    def __init__(self):
        self.rt_pose = Pose()
        self.noisy_pose = Pose()
        self.rt_velocity_publisher = rospy.Publisher('/rt/cmd_vel', Twist, queue_size=10)
        self.rt_real_pose_publisher = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)
        self.rt_noisy_pose_publisher = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10)
        rospy.Subscriber('/rt/pose', Pose, self.update_rt_pose)
        self.chase_complete = False

    def update_rt_pose(self, data):
        self.rt_pose = data

    def add_noise_to_pose(self):
        noisy_pose = Pose()
        noisy_pose.x = self.rt_pose.x + np.random.normal(0, 0.5)
        noisy_pose.y = self.rt_pose.y + np.random.normal(0, 0.5)
        noisy_pose.theta = self.rt_pose.theta + np.random.normal(0, 0.1)
        return noisy_pose

    def move_rt_in_circle(self, radius=3.2, linear_velocity=2.0):
        twist = Twist()
        while not rospy.is_shutdown():
            twist.linear.x = linear_velocity
            twist.angular.z = linear_velocity / radius

            self.rt_velocity_publisher.publish(twist)
            self.rt_real_pose_publisher.publish(self.rt_pose)

            noisy_pose = self.add_noise_to_pose()
            self.rt_noisy_pose_publisher.publish(noisy_pose)

            if self.chase_complete:
                return

    def spawn_turtle(self, name, x, y, theta):
        rospy.wait_for_service('/spawn')
        try:
            spawn_service = rospy.ServiceProxy('/spawn', Spawn)
            spawn_service(x, y, theta, name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

def target_position_callback(data):
    global target_pos_x, target_pos_y
    target_pos_x = data.x
    target_pos_y = data.y

def chaser_position_callback(data):
    global chaser_pos_x, chaser_pos_y, chaser_orientation
    chaser_pos_x = data.x
    chaser_pos_y = data.y
    chaser_orientation = data.theta  

def kalman_filter(measurement, estimated, variance_measurement, variance_estimated):
    kalman_gain = variance_estimated / (variance_estimated + variance_measurement)
    estimated_position = estimated + kalman_gain * (measurement - estimated)
    variance_estimated = (1 - kalman_gain) * variance_estimated
    return estimated_position, variance_estimated

def navigate_to_target(goal_x, goal_y):
    global chaser_pos_x, chaser_pos_y, chaser_orientation, velocity_publisher, separation_distance

    separation_distance = math.sqrt((goal_x - chaser_pos_x) ** 2 + (goal_y - chaser_pos_y) ** 2)
    
    if separation_distance <= CHASE_DISTANCE_THRESHOLD:
        angle_to_goal = math.atan2(goal_y - chaser_pos_y, goal_x - chaser_pos_x)
        acceleration = 0.1
        linear_velocity = min(TARGET_LINEAR_SPEED, separation_distance) + acceleration
        acceleration += 0.1
        angular_velocity = 2.0 * (angle_to_goal - chaser_orientation)

        velocity_command = Twist()
        velocity_command.linear.x = linear_velocity
        velocity_command.angular.z = angular_velocity
        velocity_publisher.publish(velocity_command)
        rospy.loginfo("Separation distance: %f ", separation_distance)

        if separation_distance <= 0.8:
            velocity_command.linear.x = linear_velocity + 0.1
            velocity_command.angular.z = angular_velocity + 0.1
            velocity_publisher.publish(velocity_command)
            rospy.loginfo("Target reached")
            rospy.signal_shutdown("Target reached")
    else:
        velocity_command = Twist()
        velocity_command.linear.x = 0
        velocity_command.angular.z = 0
        velocity_publisher.publish(velocity_command)

def main():
    global velocity_publisher
    rospy.init_node('turtle_pursuer')
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/rt_noisy_pose", Pose, callback=target_position_callback)
    rospy.Subscriber("/turtle1/pose", Pose, callback=chaser_position_callback)
    
    rospy.wait_for_service('/kill')
    try:
        kill_service = rospy.ServiceProxy('/kill', Kill)
        kill_service('turtle1')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
    
    rospy.wait_for_service('/spawn')
    try:
        spawn_service = rospy.ServiceProxy('/spawn', Spawn)
        spawn_service(2.0, 2.0, 0.0, 'turtle1')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

    controller = TurtleController()
    controller.spawn_turtle('rt', 5.0, 3.0, 0.0)
    rt_thread = threading.Thread(target=controller.move_rt_in_circle, args=(3.2, 2.0))
    rt_thread.start()
    
    rospy.sleep(10)  

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        navigate_to_target(target_pos_x, target_pos_y)
        rate.sleep()

if __name__ == '__main__':
    main()
