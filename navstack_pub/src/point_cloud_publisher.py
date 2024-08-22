#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

def point_cloud_callback(data):
    data.header.stamp = rospy.Time.now()
    data.header.frame_id = "zed2i_point_cloud_frame"
    point_cloud_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('point_cloud_republisher', anonymous=True)
    point_cloud_pub = rospy.Publisher('/zed2i/point_cloud_with_header', PointCloud2, queue_size=10)
    rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, point_cloud_callback)
    rospy.spin()
