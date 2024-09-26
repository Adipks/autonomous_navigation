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
