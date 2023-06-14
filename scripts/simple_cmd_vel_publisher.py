#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)

def simple_cmd_vel_publisher():
    rospy.init_node('simple_cmd_vel_publisher', anonymous=True)
    cmd_vel_pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
    cmd_vel_pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
    cmd_vel_pub3 = rospy.Publisher('/robot3/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('common_topic', String, queue_size=10)
    
    target_x = 1.0  # Target position x-coordinate
    target_y = 1.0  # Target position y-coordinate
    linear_velocity = 0.2  # Linear velocity of the robots
    
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Calculate the direction towards the target
        delta_x = target_x - 0  # Replace 0 with the current x-coordinate of the robot
        delta_y = target_y - 0  # Replace 0 with the current y-coordinate of the robot
        
        # Calculate the angle to the target position
        angle = math.atan2(delta_y, delta_x)
        
        # Create a Twist message with linear velocity towards the target and zero angular velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = 0.0
        
        # Publish the Twist message
        cmd_vel_pub1.publish(cmd_vel)
        cmd_vel_pub2.publish(cmd_vel)
        cmd_vel_pub3.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        simple_cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass
