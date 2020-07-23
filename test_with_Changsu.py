#!/usr/bin/env python

import rospy
import random
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


ackermann_data = AckermannDriveStamped()

def scan_callback(scan_data):
    scan = scan_data.ranges
    front = 0
    left = 0
    right = 0

    for x in range(825,855):
        left = left + scan[x]
    left = left / 30
    # print("left : ", left)

    for c in range(275,305):
        right = right + scan[c]
    right = right / 30
    # print("right : ", right)

    for z in range(540,580):
        front = front + scan[z]
    front = front / 40
    # print("front : ", front)

    print("front : ", front)
    print("right : ", right)

    if front < 1.0:
        if front < 0.4 and right < 0.4:
            for i in range(10000):
                ackermann_data.drive.steering_angle = -1
                ackermann_data.drive.speed = -0.5
        if front > 0.4 and right > 0.4:
            ackermann_data.drive.steering_angle = 1
            ackermann_data.drive.speed = 0.5
        else:
            ackermann_data.drive.steering_angle = 5
            ackermann_data.drive.speed = 1
    else:
        if 0.5 < right < 100:
            ackermann_data.drive.steering_angle = -1
            ackermann_data.drive.speed = 0.5
        elif right < 0.4:
            ackermann_data.drive.steering_angle = 1
            ackermann_data.drive.speed = 0.5
        else:
            ackermann_data.drive.steering_angle = 0
            ackermann_data.drive.speed = 0.5
    
def talker():
    rospy.init_node('publisher', anonymous = True)
    rate = rospy.Rate(10)
    rospy.loginfo('start node')

    pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1 )
    rospy.Subscriber('/scan', LaserScan, scan_callback)

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






