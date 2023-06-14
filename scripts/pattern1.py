#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def move_robot1():
    rospy.init_node('robot1_movebase_node')

    # Create a publisher to send the goal for robot 1
    goal_pub = rospy.Publisher('/robot1/move_base_simple/goal', PoseStamped, queue_size=10)

    # Create a PoseStamped message with the desired goal
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = '/robot1/map'
    goal_msg.pose.position.x = 1.0  # Adjust the desired x-coordinate
    goal_msg.pose.position.y = 2.0  # Adjust the desired y-coordinate
    goal_msg.pose.orientation.w = 1.0  # Adjust the desired orientation

    # Publish the goal message
    goal_pub.publish(goal_msg)
    rospy.loginfo('Goal sent for robot 1')

if __name__ == '__main__':
    try:
        move_robot1()
    except rospy.ROSInterruptException:
        pass
