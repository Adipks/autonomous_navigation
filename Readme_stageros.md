
# Stage ROS

## Overview

**Stage** is a 2D multi-robot simulator that is used in conjunction with ROS (Robot Operating System) to simulate large environments, multiple robots, and their interactions. It provides a virtual world where robots, sensors, and objects can be simulated in a 2D grid. Stage ROS is a ROS package that integrates Stage into the ROS ecosystem, allowing for seamless simulation of robots and their interactions within ROS nodes.

## Features

- Simulate large-scale multi-robot systems in a 2D environment.
- Supports a variety of robot models, including differential drive and holonomic robots.
- Configurable world files (.world) to define environments with obstacles, landmarks, and robots.
- Visualize sensor data like laser scans, odometry, and camera images from simulated robots.
- Use standard ROS topics and services to control and interact with robots in the simulated environment.

## Installation

To install Stage ROS, ensure you have ROS installed (preferably ROS Noetic). You can install the `stage_ros` package using the following command:

```bash
sudo apt-get install ros-noetic-stage-ros
```

After installation, clone the `stage_ros` repository into your workspace (if you need the latest version):

```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/stage_ros.git
cd ~/catkin_ws
catkin_make
```

## Running Stage ROS

1. First, ensure that your workspace is sourced:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

2. Start the Stage simulator with an example world file:

   ```bash
   rosrun stage_ros stageros <world_file>
   ```

   Replace `<world_file>` with the path to your desired `.world` file. Example world files can be found in the `stage_ros` package.
   Here our world file is [iroc_arena.world](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/stage/iroc_arena.world)
   and our [robot model](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/stage/robots/carlike_robot.inc)

## Example: Running a TurtleBot in Stage

1. Install the TurtleBot3 simulation package:

   ```bash
   sudo apt-get install ros-noetic-turtlebot3-stage
   ```

2. Launch the TurtleBot3 simulation in Stage:

   ```bash
   roslaunch turtlebot3_stage turtlebot3_stage.launch
   ```

   This will open the Stage interface and load a simple world with a TurtleBot robot that you can control using ROS commands.

## ROS Topics and Services

Stage ROS publishes and subscribes to standard ROS topics, including:

- `/odom` - Odometry data for the robot.
- `/cmd_vel` - Velocity commands for controlling the robot.
- `/scan` - Laser scan data from a simulated LIDAR.

You can control the robot using teleop tools or by publishing directly to `/cmd_vel`.

## Customizing Worlds

Stage uses `.world` files to define the environment. These files allow you to specify elements like:

- Obstacles and walls
- Robot models
- Sensor configurations (e.g., LIDAR, cameras)
- Dynamic objects

For more information on creating and customizing `.world` files, consult the Stage documentation.

## Troubleshooting

- **Stage does not start:** Make sure the Stage package is correctly installed and sourced.
- **ROS topic issues:** Verify the topics and services are correctly set up by running `rostopic list`.
- **Performance problems:** Simulating large numbers of robots or complex environments can slow down Stage. Consider reducing the number of objects or adjusting the simulation parameters.

## Further Resources

- [ROS Wiki: Stage](http://wiki.ros.org/stage_ros)
- [GitHub Repository: Stage ROS](https://github.com/ros-simulation/stage_ros)

---

By using Stage ROS, you can efficiently simulate and test multi-robot systems in 2D environments, leveraging the power of ROS for robot control and sensor integration.
[Back](https://github.com/Adipks/autonomous_navigation/tree/main)
