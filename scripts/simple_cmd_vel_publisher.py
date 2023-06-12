#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)

def simple_cmd_vel_publisher():
    rospy.init_node('simple_cmd_vel_publisher', anonymous=True)
    cmd_vel_pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
    cmd_vel_pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('common_topic', String, queue_size=10)
    # Set the desired linear and angular velocity
    linear_velocity = 0.2  # m/s
    angular_velocity = 0.1  # rad/s

    # Create a Twist message
    cmd_vel = Twist()
    cmd_vel.linear.x = linear_velocity
    cmd_vel.angular.z = angular_velocity

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Publish the Twist message
        cmd_vel_pub1.publish(cmd_vel)
        cmd_vel_pub2.publish(cmd_vel)
        message="Hello"
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass
