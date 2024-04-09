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

    except rospy.ROSInterruptException:
        pass












# #!/usr/bin/env python3

# import rospy
# from turtlesim.srv import Spawn, Kill
# from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose
# import math

# # Global variables for current position
# x = 0
# y = 0
# theta = 0

# # Constants
# TOLERANCE = 0.5
# SPEED = 2.0

# def our_position_callback(data):
#     global x, y, theta
#     x = data.x
#     y = data.y
#     theta = data.theta
#     # rospy.loginfo("Position: x=%f, y=%f, theta=%f", x, y, theta)

# def go_to_goal(target_x, target_y):
#     global x, y, theta, SPEED, TOLERANCE
#     rospy.loginfo("Position: x=%f, y=%f, theta=%f", x, y, theta)
#     dist = math.sqrt((target_x - x)**2 + (target_y - y)**2)
#     angle = math.atan2(target_y - y, target_x - x)
    
#     if dist > TOLERANCE:
#         linear_velocity = min(SPEED, dist)
#         angular_velocity = 2.0 * (angle - theta)
        
#         cmd = Twist()
#         cmd.linear.x = linear_velocity
#         cmd.angular.z = angular_velocity
#         pub.publish(cmd)
#     else:
#         cmd = Twist()
#         cmd.linear.x = 0
#         cmd.angular.z = 0
#         pub.publish(cmd)
#         rospy.loginfo("Goal reached")

# def main():
#     global pub
#     rospy.init_node('turtle_spawn_delete')
#     pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
#     rospy.Subscriber('/turtle1/pose', Pose, our_position_callback)
    
#     # Delete previous turtle
#     rospy.wait_for_service('/kill')
#     try:
#         kill_proxy = rospy.ServiceProxy('/kill', Kill)
#         kill_proxy('turtle1')
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", e)
    
#     # Spawn new turtle
#     rospy.wait_for_service('/spawn')
#     try:
#         spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
#         spawn_proxy(1.0, 1.0, 0.0, 'turtle1')
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", e)

#     # Set the goal position and move to it
#     go_to_goal(1, 8)

#     rospy.spin()

# if __name__ == '__main__':
#     main()







# # # #!/usr/bin/env python3

# # # import rospy
# # # from turtlesim.srv import Spawn, Kill
# # # from geometry_msgs.msg import Twist
# # # from turtlesim.msg import Pose
# # # import math

# # # # Global variables for current position
# # # x = 0
# # # y = 0
# # # theta = 0

# # # # Constants
# # # TOLERANCE = 0.5
# # # SPEED = 2.0

# # # def our_position_callback(data):
# # #     global x, y, theta
# # #     x = data.x
# # #     y = data.y
# # #     theta = data.theta
# # #     # rospy.loginfo("Position: x=%f, y=%f, theta=%f", x, y, theta)
# # # def movebot(self, x, y, z, x1, y1, z1, duration):
# # #         cmd = Twist()
# # #         cmd.linear.x = x
# # #         cmd.linear.y = y
# # #         cmd.linear.z = z
# # #         cmd.angular.x = x1
# # #         cmd.angular.y = y1
# # #         cmd.angular.z = z1
# # #         rate = rospy.Rate(10)
# # #         start_time = rospy.Time.now()
# # #         while rospy.Time.now() - start_time < rospy.Duration(duration):
# # #             pub.publish(cmd)
# # #             rate.sleep()
# # # def go_to_goal(target_x, target_y):
# # #     global x, y, theta, SPEED, TOLERANCE
# # #     rospy.loginfo("our Position: x=%f, y=%f", x, y)
# # #     dist = math.sqrt((target_x - x)**2 + (target_y - y)**2)
# # #     angle = math.atan2(target_y - y, target_x - x)
    
# # #     if dist > TOLERANCE:
# # #         linear_velocity = min(SPEED, dist)
# # #         angular_velocity = 2.0 * (angle - theta)
        
# # #         cmd = Twist()
# # #         cmd.linear.x = linear_velocity
# # #         cmd.angular.z = angular_velocity
# # #         pub.publish(cmd)
# # #     else:
# # #         cmd = Twist()
# # #         cmd.linear.x = 0
# # #         cmd.angular.z = 0
# # #         pub.publish(cmd)
# # #         rospy.loginfo("Goal reached")

# # # def main():
# # #     global pub
# # #     rospy.init_node('turtle_spawn_delete')
# # #     pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
# # #     rospy.Subscriber('/turtle1/pose', Pose, our_position_callback)
    
# # #     # Delete previous turtle
# # #     rospy.wait_for_service('/kill')
# # #     try:
# # #         kill_proxy = rospy.ServiceProxy('/kill', Kill)
# # #         kill_proxy('turtle1')
# # #     except rospy.ServiceException as e:
# # #         rospy.logerr("Service call failed: %s", e)
    
# # #     # Spawn new turtle
# # #     rospy.wait_for_service('/spawn')
# # #     try:
# # #         spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
# # #         spawn_proxy(1.0, 1.0, 0.0, 'turtle1')
# # #     except rospy.ServiceException as e:
# # #         rospy.logerr("Service call failed: %s", e)

# # #     # Set the goal position and move to it
# # #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# # #         x.movebot(2.4, 0, 0, 0, 0, 0, 4)
# # #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# # #         x.movebot(0, 2, 0, 0, 0, 0, 0.4)
# # #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# # #         x.movebot(-2.4, 0, 0, 0, 0, 0, 4)
# # #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# # #         x.movebot(0, 2, 0, 0, 0, 0, 0.4)
# # #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)

# # #     rospy.spin()

# # # if __name__ == '__main__':
# # #     main()




# # #!/usr/bin/env python

# # import rospy
# # from geometry_msgs.msg import Twist, Pose
# # from turtlesim.msg import Pose
# # from math import pow, atan2, sqrt

# # class TurtleBot:

# #     def __init__(self):
# #         rospy.init_node('single_turtle_controller', anonymous=True)
# #         self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
# #         self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
# #         self.pose = Pose()
# #         self.rate = rospy.Rate(10)

# #     def update_pose(self, data):
# #         self.pose = data



# #     def euclidean_distance(self, goal_pose):
# #         return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

# #     def linear_vel(self, goal_pose, constant=1.5):
# #         return constant * self.euclidean_distance(goal_pose)

# #     def steering_angle(self, goal_pose):
# #         return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

# #     def angular_vel(self, goal_pose, constant=6):
# #         return constant * (self.steering_angle(goal_pose) - self.pose.theta)

# #     def move2goal(self, tx, ty):
# #         goal_pose = Pose()
# #         goal_pose.x = tx
# #         goal_pose.y = ty
# #         distance_tolerance = 0.5
        
# #         vel_msg = Twist()

# #         while self.euclidean_distance(goal_pose) >= distance_tolerance:
# #             vel_msg.linear.x = self.linear_vel(goal_pose)
# #             vel_msg.angular.z = self.angular_vel(goal_pose)
# #             self.velocity_publisher.publish(vel_msg)
# #             self.rate.sleep()

# #         vel_msg.linear.x = 0
# #         vel_msg.angular.z = 0
# #         self.velocity_publisher.publish(vel_msg)

# # if __name__ == '__main__':
# #     try:
# #         x = TurtleBot()
# #         # x.move2goal(1, 1)
# #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# #         x.movebot(2.4, 0, 0, 0, 0, 0, 4)
# #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# #         x.movebot(0, 2, 0, 0, 0, 0, 0.4)
# #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# #         x.movebot(-2.4, 0, 0, 0, 0, 0, 4)
# #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# #         x.movebot(0, 2, 0, 0, 0, 0, 0.4)
# #         x.movebot(0, 0, 0, 0, 0, 0, 0.5)
# #     except rospy.ROSInterruptException:
# #         pass








