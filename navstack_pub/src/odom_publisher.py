#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def odom_callback(data):
    # Update the timestamp
    data.header.stamp = rospy.Time.now()
    
    # Set the frame IDs
    data.header.frame_id = "odom"
    # data.child_frame_id = "base_footprint"
    
    # Republish the odometry data with the updated header
    odom_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('odom_republisher', anonymous=True)
    
    # Publisher for the modified odometry data
    odom_pub = rospy.Publisher('/zed2i/odom_with_header', Odometry, queue_size=10)
    
    # Subscriber for the original odometry data from ZED2i
    rospy.Subscriber('/zed2i/zed_node/odom', Odometry, odom_callback)
    
    rospy.spin()
