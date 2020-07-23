#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math

ackermann_data = AckermannDriveStamped()

def scan_callback(scan_data):
    scan = scan_data.ranges

    left = scan[840]
    right = scan[290]
    front = scan[560]
    back = scan [10]

    # print("front :", front)
    # print("right :", right)
    # print("left :", left)
    

    if front < 3:
        if left <= 2 and right >= 2:
            ackermann_data.drive.steering_angle = -1    
            ackermann_data.drive.speed = 1

        elif right <= 2 and left >= 2:
            ackermann_data.drive.steering_angle = 1
            ackermann_data.drive.speed = 1
    else:
        if 1 <= left <= 1.5:
            ackermann_data.drive.steering_angle = 0
            ackermann_data.drive.speed = 1
        elif left < 1:
            ackermann_data.drive.steering_angle = -0.05
            ackermann_data.drive.speed = 1
        elif left > 1.5:
            ackermann_data.drive.steering_angle = 0.05
            ackermann_data.drive.speed = 1


def talker():
    rospy.init_node('publisher', anonymous = True)
    rate = rospy.Rate(10)
    rospy.loginfo('start node')

    pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1 )
    sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    
    while not rospy.is_shutdown():
        # data.header.stamp = rospy.get_time()
        pub.publish(ackermann_data)
        rospy.loginfo('publish data')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
