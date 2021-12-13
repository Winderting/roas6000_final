#!/usr/bin/env python

'''
version 1
@time       :2021/12/13 15:48:23
@author     :Ge Sun
'''

import rospy
import tf2_ros
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseStamped


def pose_callback(pose_msg):
    # the relative position of robot in 2D (ignore z axis)
    x = pose_msg.pose.position.x
    y = pose_msg.pose.position.y
    d = [x, y]
    # area = 'A'
    # hardcode the area separation
    if d[0] > 5.08:
        area = 'D'
    elif ((d[1] < 0.325) & (d[1] > -3.675)):
        area = 'A'
    elif ((d[1] < -3.675) & (d[1] > -6.75)):
        area = 'B'
    elif ((d[1] < -6.75) & (d[1] > -14)):
        area = 'C'
    area_str = 'The area is ' + area
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo(area_str)
        rate.sleep()
    # pub.publish(area_str)


if __name__ == '__main__':
    # Initialize the ROS Node named 'area_check'
    rospy.init_node('area_check', anonymous=True)
    # subcribe the /slam_out_pose topic to get the pose of the robot relative to map
    sub = rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()