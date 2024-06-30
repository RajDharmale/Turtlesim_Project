#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from math import atan2, sqrt

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt((goal_pose.x - self.pose.x) ** 2 + (goal_pose.y - self.pose.y) ** 2)

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move_to_goal(self, goal_x, goal_y, distance_tolerance):
        goal_pose = Pose()
        goal_pose.x = goal_x
        goal_pose.y = goal_y

        vel_msg = Twist()
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    def spawn_new_turtle(name, x, y, theta):
     rospy.wait_for_service('/spawn')
     try:
        spawn_proxy = rospy.ServiceProxy('/spawn',Spawn)
        spawn_proxy(x, y, theta, name)
     except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

    def delete_previous_turtle(name):
     rospy.wait_for_service('/kill')
     try:
        kill_proxy = rospy.ServiceProxy('/kill', Kill)
        kill_proxy(name)
     except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
if __name__ == '__main__':
    try:
        x = TurtleBot()
        try:
          kill_proxy = rospy.ServiceProxy('/kill', Kill)
          kill_proxy('turtle1')
        except rospy.ServiceException as e:
         rospy.logerr("Service call failed: %s", e)
    
    # Spawn new turtle at custom location
        rospy.wait_for_service('/spawn')
        try:
          spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
          spawn_proxy(1.0, 1.0, 0.0, 'turtle1')
        except rospy.ServiceException as e:
          rospy.logerr("Service call failed: %s", e)
        x.move_to_goal(float(input("Set your x goal: ")), float(input("Set your y goal: ")), float(input("Set your tolerance: ")))
    except rospy.ROSInterruptException:
        pass


