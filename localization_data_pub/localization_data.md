## Rover Localization

This folder contains files for localizing the rover, even in dynamic environments.

### Overview

The system utilizes an **Extended Kalman Filter (EKF)** that integrates data from multiple sources:

- **Visual Odometry:** From the ZED-2i Stereo Camera.
- **Wheel Encoder Odometry**
- **IMU Readings:** From the ZED-2i Camera.
- **Initial Pose**

The EKF helps **AMCL (Adaptive Monte Carlo Localization)** to more accurately localize the rover's footprint by:
- Filtering out noise from these sensors.
- Fusing data to enhance localization precision.


> Robot's Intrinsic Parameters (Varies for Individual Rovers)

```cpp
// Robot physical constants
const double TICKS_PER_REVOLUTION = 620; // For reference purposes.
const double WHEEL_RADIUS = 0.033; // Wheel radius in meters
const double WHEEL_BASE = 0.17; // Center of left tire to center of right tire
const double TICKS_PER_METER = 2880;
```

> Function to Interactively set the Robot's pose in RViZ

```cpp
// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {

  cout << "Received initial pose." << endl;

  headingOffset = rvizClick.pose.orientation.z - imuHeading;
  
  cout << "heading offset = " << rvizClick.pose.orientation.z << " - " << imuHeading << " = " << headingOffset << endl;

  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
  imuHeadingInitialized = true;
}
```

> Fused Odometry Message format initialisation

```cpp
  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;
```

> Subscribers that gets information from Sensor topics

```cpp
  // Subscribe to ROS topics
  ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
  ros::Subscriber subImu = node.subscribe("imu/data", 100, update_heading);
```

Refer here for Full Code: [Code](https://github.com/Adipks/autonomous_navigation/blob/main/localization_data_pub/src/ekf_odom_pub.cpp)

Navigate back: [Back](https://github.com/Adipks/autonomous_navigation?tab=readme-ov-file)
