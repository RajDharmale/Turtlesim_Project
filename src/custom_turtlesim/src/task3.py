#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def move_and_publish_pose():
   

    pub_pose = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)
    pub_pose2 = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10)
    pub_cmd_vel = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    def pose_callback(data):
        pose_msg = Pose()
        pose_msg.x = data.x
        pose_msg.y = data.y
        pose_msg.theta = data.theta
        rospy.sleep(rospy.Duration(5))
        pub_pose.publish(pose_msg)
        pose_msg = Pose()
        pose_msg.x = data.x-1
        pose_msg.y = data.y-1
        rospy.sleep(rospy.Duration(5))
        pub_pose2.publish(pose_msg)

    rospy.Subscriber('/turtle2/pose', Pose, pose_callback)

    rate = rospy.Rate(10)  # 10 Hz

    vel_cmd = Twist()
    vel_cmd.linear.x = 2 # linear velocity
    vel_cmd.angular.z = 0.4  # angular velocity

    while not rospy.is_shutdown():
        pub_cmd_vel.publish(vel_cmd)
        rate.sleep()

def spawn_new_turtle(name, x, y, theta):
    rospy.wait_for_service('/spawn')
    try:
        spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
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

def main():
    rospy.init_node('turtle_circle')
    spawn_new_turtle('turtle2', 5.5, 0.8, 0.0)
    move_and_publish_pose()

if __name__ == '__main__':
    main()
