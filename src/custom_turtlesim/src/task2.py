#!/usr/bin/env python

from os import kill
import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
import math

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz
        self.pose = Pose()
        self.target_pose = Pose()
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
    def update_pose(self, data):
        self.pose = data

    def move_to_target(self, target_x, target_y, acceleration, deceleration):
        # Calculate distance to target
        distance = math.sqrt((target_x - self.pose.x)**2 + (target_y - self.pose.y)**2)
        
        # Set initial velocity
        velocity = 0.0
        twist = Twist()
        
        # Loop until target reached
        while distance > 0.1:  # 0.1 is the tolerance for reaching the target
            # Calculate current distance to target
            distance = math.sqrt((target_x - self.pose.x)**2 + (target_y - self.pose.y)**2)
            
            # Update velocity based on distance
            if distance > deceleration:
                velocity += acceleration * 0.1  # 0.1 is the time step
            elif distance <= deceleration:
                velocity -= acceleration * 0.1  # Decelerate
            
            # Limit velocity to avoid overshooting
            if velocity > distance:
                velocity = distance
            
            # Calculate angle to target
            angle_to_target = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
            
            # Set linear velocity
            twist.linear.x = velocity
            twist.linear.y = 0
            twist.linear.z = 0
            
            # Set angular velocity to turn towards target
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 4 * (angle_to_target - self.pose.theta)
            
            # Publish velocity
            self.velocity_publisher.publish(twist)
            
            # Sleep for 0.1 seconds
            self.rate.sleep()
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
        controller = TurtleController() 
        rospy.wait_for_service('/kill')
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
    
        controller.move_to_target(10, 1, 0.1, 0.1)
        controller.move_to_target(10, 2, 0.1, 0.1)
        controller.move_to_target(1, 2, 0.1, 0.1)
        controller.move_to_target(1, 3, 0.1, 0.1)
        controller.move_to_target(10, 3, 0.1, 0.1)
        controller.move_to_target(10, 4, 0.1, 0.1)
        controller.move_to_target(1, 4, 0.1, 0.1)
        controller.move_to_target(1, 5, 0.1, 0.1)
        controller.move_to_target(10, 5, 0.1, 0.1)
        controller.move_to_target(10, 6, 0.1, 0.1)
        controller.move_to_target(1,6,0.1,0.1)
        controller.move_to_target(1,7,0.1,0.1)
        controller.move_to_target(10,7,0.1,0.1)
        controller.move_to_target(10,8,0.1,0.1)
        

    except rospy.ROSInterruptException:
        pass















