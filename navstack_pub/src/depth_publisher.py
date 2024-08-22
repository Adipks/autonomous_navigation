#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

def depth_callback(data):
    data.header.stamp = rospy.Time.now()
    data.header.frame_id = "zed2i_depth_frame"
    depth_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('depth_republisher', anonymous=True)
    depth_pub = rospy.Publisher('/zed2i/depth_with_header', Image, queue_size=10)
    rospy.Subscriber('/zed2i/zed_node/depth/depth_registered', Image, depth_callback)
    rospy.spin()
