#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf import TransformListener

def goToPose(robot_ns):
    pub_gotopose = rospy.Publisher(robot_ns + '/move_base_simple/goal', PoseStamped, queue_size=1)
    map_frame = rospy.get_param("~map_frame", robot_ns +'map')
    robot_frame = rospy.get_param("~robot_frame",  '/base_link')

    # Create target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = map_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = 1
    target_pose.pose.position.y = 1
    target_pose.pose.position.z = 0
    target_pose.pose.orientation.x = 0
    target_pose.pose.orientation.y = 0
    target_pose.pose.orientation.z = 0
    target_pose.pose.orientation.w = 1

    rospy.loginfo(target_pose)

    # Send goal
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.loginfo("Publishing goal for %s", robot_ns)
    pub_gotopose.publish(target_pose)

if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_gotopose', anonymous=True)

        # Move robot1 to the goal pose
        robot1_ns = 'robot1'
        goToPose(robot1_ns)

        # Move robot2 to the goal pose
        robot2_ns = 'robot2'
        goToPose(robot2_ns)

    except rospy.ROSInterruptException:
        pass
