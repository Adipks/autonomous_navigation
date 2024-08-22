#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(data):
    # Update the timestamp
    data.header.stamp = rospy.Time.now()
    imu_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('imu_republisher', anonymous=True)
    imu_pub = rospy.Publisher('/zed2i/imu_with_header', Imu, queue_size=10)
    rospy.Subscriber('/zed2i/zed_node/imu/data', Imu, imu_callback)
    rospy.spin()
