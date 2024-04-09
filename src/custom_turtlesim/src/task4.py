#!/usr/bin/env python3


import rospy
import math
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Global variables for positions and angles
target_x = 0.0
target_y = 0.0
chaser_x = 0.0
chaser_y = 0.0
chaser_theta = 0.0  # Angle of the chaser turtle
distance=0

# Global publisher for velocity commands
pub = None

# Constants for chasing behavior
CHASE_DISTANCE_THRESHOLD = 3.0
CHASE_LINEAR_SPEED = 10.0
TARGET_LINEAR_SPEED = 4


def target_pose_callback(data):
    global target_x, target_y
   
    target_x = data.x
    target_y = data.y 
       
def movebot(x,y,z,x1,y1,z1,duration):
    global distance
    cmd=Twist()
    cmd.linear.x=x
    cmd.linear.y=y 
    cmd.linear.z=z 
    cmd.angular.x=x1 
    cmd.angular.y=y1 
    cmd.angular.z=z1
    rate = rospy.Rate(10)
    start_time=rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(duration):
       if distance<=CHASE_DISTANCE_THRESHOLD:
           return
       pub.publish(cmd)
       if distance<=CHASE_DISTANCE_THRESHOLD:
        return
       rate.sleep
def chaser_pose_callback(data):
    global chaser_x, chaser_y, chaser_theta
    chaser_x = data.x
    chaser_y = data.y
    chaser_theta = data.theta  # Update chaser's angle

def move_to_goal(goal_x, goal_y):
    global chaser_x, chaser_y, chaser_theta, pub,distance

    # rospy.loginfo("%f, %f , %f ,%f",goal_x,goal_y,chaser_x,chaser_y)
    distance = math.sqrt((goal_x - chaser_x) *(goal_x - chaser_x) + (goal_y - chaser_y) *(goal_y - chaser_y))
    flag=False
    if(distance<=5 or flag):
        flag=True
        angle = math.atan2(goal_y - chaser_y, goal_x - chaser_x)
        ac=0.1
    # Calculate linear velocity towards the goal
        linear_velocity = min(TARGET_LINEAR_SPEED, distance)+ac
        ac=ac+0.1
    # Calculate angular velocity towards the goal
        angular_velocity = 2.0 * (angle - chaser_theta)

    # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        pub.publish(cmd)
        rospy.loginfo("distance %f ",distance)
        # If goal is reached, stop and shutdown
        if distance <= 0.1 and distance!=0:
            cmd = Twist()
            cmd.linear.x = linear_velocity+0.1
            cmd.angular.z = angular_velocity+0.1
            pub.publish(cmd)
            rospy.loginfo("Goal reached")
            rospy.signal_shutdown("Goal reached")
    else:
        movebot(0,0,0,0,0,0,0.5)
        movebot(2.4,0,0,0,0,0,4)
        movebot(0,0,0,0,0,0,0.5)
        movebot(0,2,0,0,0,0,0.4)
        movebot(0,0,0,0,0,0,0.5)
        movebot(-2.4,0,0,0,0,0,4)
        movebot(0,0,0,0,0,0,0.5)
        movebot(0,2,0,0,0,0,0.4)
        movebot(0,0,0,0,0,0,0.5)
                

def main():
    global pub
    rospy.init_node('turtle_chaser')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/rt_real_pose", Pose, callback=target_pose_callback)
    rospy.Subscriber("/turtle1/pose", Pose, callback=chaser_pose_callback)
    
    # Delete previous turtle
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
        spawn_proxy(1,1, 0.0, 'turtle1')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
    
    # Main loop to move the turtle to the goal
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move_to_goal(target_x, target_y)
        rate.sleep()

if __name__ == '__main__':
    main()











# #!/usr/bin/env python3


# import rospy
# import math
# from turtlesim.srv import Spawn, Kill
# from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose

# # Global variables for positions and angles
# target_x = 0.0
# target_y = 0.0
# chaser_x = 0.0
# chaser_y = 0.0
# chaser_theta = 0.0  # Angle of the chaser turtle
# distance=0

# # Global publisher for velocity commands
# pub = None

# # Constants for chasing behavior
# CHASE_DISTANCE_THRESHOLD = 3.0
# CHASE_LINEAR_SPEED = 10.0
# TARGET_LINEAR_SPEED = 2.0


# def target_pose_callback(data):
#     global target_x, target_y
    
    
#       # 0.2 Hz, i.e., 5 seconds interval
#      # Wait for the flag to be set
#     target_x = data.x
#     target_y = data.y 
       
# def movebot(x,y,z,x1,y1,z1,duration):
#     global distance
#     cmd=Twist()
#     cmd.linear.x=x
#     cmd.linear.y=y 
#     cmd.linear.z=z 
#     cmd.angular.x=x1 
#     cmd.angular.y=y1 
#     cmd.angular.z=z1
#     rate = rospy.Rate(10)
#     start_time=rospy.Time.now()
#     while rospy.Time.now() - start_time < rospy.Duration(duration):
#        if distance<=CHASE_DISTANCE_THRESHOLD:
#            return
#        pub.publish(cmd)
#        rate.sleep
# def chaser_pose_callback(data):
#     global chaser_x, chaser_y, chaser_theta
#     chaser_x = data.x
#     chaser_y = data.y
#     chaser_theta = data.theta  # Update chaser's angle

# def move_to_goal(goal_x, goal_y):
#     global chaser_x, chaser_y, chaser_theta, pub,distance

#     # rospy.loginfo("%f, %f , %f ,%f",goal_x,goal_y,chaser_x,chaser_y)
#     distance = math.sqrt((goal_x - chaser_x) *(goal_x - chaser_x) + (goal_y - chaser_y) *(goal_y - chaser_y))
    
#     if(distance<=3):
    
#         angle = math.atan2(goal_y - chaser_y, goal_x - chaser_x)
#         ac=0.1
#     # Calculate linear velocity towards the goal
#         linear_velocity = min(TARGET_LINEAR_SPEED, distance)+ac
#         ac=ac+0.1
#     # Calculate angular velocity towards the goal
#         angular_velocity = 2.0 * (angle - chaser_theta)

#     # Publish velocity command
#         cmd = Twist()
#         cmd.linear.x = linear_velocity
#         cmd.angular.z = angular_velocity
#         pub.publish(cmd)
#         rospy.loginfo("distance %f ",distance)
#         # If goal is reached, stop and shutdown
#         if distance <= 0.8:
#             cmd = Twist()
#             cmd.linear.x = linear_velocity+0.1
#             cmd.angular.z = angular_velocity+0.1
#             pub.publish(cmd)
#             rospy.loginfo("Goal reached")
#             rospy.signal_shutdown("Goal reached")
#     else:
#         movebot(0,0,0,0,0,0,0.5)
#         movebot(2.4,0,0,0,0,0,4)
#         movebot(0,0,0,0,0,0,0.5)
#         movebot(0,2,0,0,0,0,0.4)
#         movebot(0,0,0,0,0,0,0.5)
#         movebot(-2.4,0,0,0,0,0,4)
#         movebot(0,0,0,0,0,0,0.5)
#         movebot(0,2,0,0,0,0,0.4)
#         movebot(0,0,0,0,0,0,0.5)
                

# def main():
#     global pub
#     rospy.init_node('turtle_chaser')
#     pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
#     rospy.Subscriber("/rt_real_pose", Pose, callback=target_pose_callback)
#     rospy.Subscriber("/turtle1/pose", Pose, callback=chaser_pose_callback)
    
#     # Delete previous turtle
#     rospy.wait_for_service('/kill')
#     try:
#         kill_proxy = rospy.ServiceProxy('/kill', Kill)
#         kill_proxy('turtle1')
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", e)
    
#     # Spawn new turtle at custom location
#     rospy.wait_for_service('/spawn')
#     try:
#         spawn_proxy = rospy.ServiceProxy('/spawn', Spawn)
#         spawn_proxy(2.0, 2.0, 0.0, 'turtle1')
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", e)
    
#     # Main loop to move the turtle to the goal
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         move_to_goal(target_x, target_y)
#         rate.sleep()

# if __name__ == '__main__':
#     main()

