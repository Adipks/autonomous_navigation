# Autonomous Drive Files

> This folder contains the necessary files for the autonomous drive system of a 4-wheeled Rover


### Folder Structure

├── [Localization](https://github.com/Adipks/autonomous_navigation/blob/main/localization_data_pub/localization_data.md)

└── [Navigation Stack](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/autonomous_navigation.md)

# Launchfiles and other params

## to launch the stack run
```bash
roslaunch navstack_pub navigation_main.launch
```
## launchfile params
## Contents

- [Global Parameters](#global-parameters)
- [Stage Simulator](#stage-simulator)
- [Navigation Parameters](#navigation-parameters)
- [Map Server](#map-server)
- [AMCL](#amcl)
- [Static Transforms](#static-transforms)
- [Robot Pose EKF](#robot-pose-ekf)
- [RTAB-Map](#rtab-map)
- [Point Cloud to LaserScan Conversion](#point-cloud-to-laserscan-conversion)
- [Manual Pose Publisher](#manual-pose-publisher)
- [RViz Visualisation](#rviz-visualisation)

---

### Global Parameters
- **use_sim_time**: Defines whether the ROS nodes should use simulated time. It’s currently set to `false` (real-time). This can be set to `true` when running the Stage simulation.
```yaml
 <!-- ########## Global Parameters ########## -->
  <!-- <param name="/use_sim_time" value="true"/> -->
  <param name="/use_sim_time" value="false"/>
```

### Stage Simulator
- **stage_ros**: Launches the Stage ROS simulator with a specified world file, `iroc_arena.world`. It publishes scan data remapped to the `scan` topic.
```yaml
<!-- ########## Stage Simulator ########## -->
<!-- <node pkg="stage_ros" type="stageros" name="stageros" args="$(find navstack_pub)/stage/iroc_arena.world">
<remap from="base_scan" to="scan"/>
</node> -->
```

### Navigation Parameters
- **move_base**: Core node for autonomous navigation, which integrates:
  - `costmap_common_params.yaml`: Configures global and local costmaps.
  - `local_costmap_params.yaml` and `global_costmap_params.yaml`: Parameters for local and global costmaps, respectively.
  - **TEB Local Planner**: Configured for car-like or differential drive by loading parameters from `teb_local_planner_params.yaml` (for car-like) or `teb_local_planner_diff_drive.yaml` (for differential drive).  
  - **Planner and Controller Settings**: Defines frequency and patience of the planner and controller, as well as settings for clearing rotations which are disabled for a car-like robot.
 
```yaml
<!-- Navigation Parameters -->
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find navstack_pub)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find navstack_pub)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find navstack_pub)/param/local_costmap_params.yaml" command="load" ns="load"/>
    <rosparam file="$(find navstack_pub)/param/global_costmap_params.yaml" command="load" ns="load"/>
    <!-- <rosparam file="$(find navstack_pub)/param/teb_local_planner_params.yaml" command="load"/>  for a carlike system -->
    <rosparam file="$(find navstack_pub)/param/teb_local_planner_diff_drive.yaml" command="load"/> <!-- for a diff drive system -->
    <param name="base_global_planner" value="global_planner/GlobalPlanner" />
    <param name="planner_frequency" value="1.0" />
    <param name="planner_patience" value="5.0" />
    <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS" />
    <param name="controller_frequency" value="5.0" />
    <param name="controller_patience" value="15.0" />
    <param name="clearing_rotation_allowed" value="false" /> <!-- Our carlike robot is not able to rotate in place -->
</node>
```

### Map Server
- **map_server**: Loads the static map (`iroc_arena.yaml`) and publishes it to provide the robot’s environment.
```yaml
<!-- ########## Map ########## -->
<!-- <arg name="map_file" default="$(find navstack_pub)/maps/final_map.yaml"/>
<node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)" >
<param name="frame_id" value="map"/>
</node> -->
<node name="map_server" pkg="map_server" type="map_server" args="$(find navstack_pub)/maps/iroc_arena.yaml" output="screen">
<param name="frame_id" value="map"/>
</node>
```

### AMCL
- **amcl**: Adaptive Monte Carlo Localization node for localizing the robot within the static map. Loads parameters from `amcl_params.yaml` and sets initial positions (x, y, and orientation) for the robot.
```yaml
<!-- ########## AMCL ########## -->
<node pkg="amcl" type="amcl" name="amcl" output="screen">
<rosparam file="$(find navstack_pub)/param/amcl_params.yaml" command="load" />
<param name="initial_pose_x" value="0.7"/> <!-- 0.5 -->
<param name="initial_pose_y" value="2.5"/> <!-- 2.5 -->
<param name="initial_pose_a" value="0"/>
</node>
```

### Static Transforms
- **tf2_ros**: Static transformations to handle coordinate frames, e.g., `base_footprint` to `base_link`. Additional transformations can link sensor frames to the robot base.
```yaml
<!-- ########## Static Transforms ########## -->
<!-- <node pkg="tf2_ros" type="static_transform_publisher" name="base_footprint_to_base_link" args="0 0 0.4 0 0 0  base_footprint base_link 10" /> -->
<!-- <node pkg="tf2_ros" type="static_transform_publisher" name="imu_broadcaster" args="0.2 -0.1 0.24 0 0 -1.5708 base_link zed2_imu_link 30" /> -->
<!-- <node pkg="tf2_ros" type="static_transform_publisher" name="map_to_base_footprint" args="0 0 0 0 0 0 map base_footprint 30" /> -->
```

### Robot Pose EKF
- **robot_pose_ekf**: Combines data from multiple sensors, such as the IMU and odometry, to estimate the robot’s position more accurately. 
  - `output_frame`: Defines the output frame (`odom`) for the combined position estimate.
  - `base_footprint_frame`: Frame of the robot’s base (`base_link`).
  - **Sensor Configurations**: IMU, odometry, and VO (Visual Odometry) inputs are enabled for enhanced accuracy, while GPS is disabled.
```yaml
<!-- ########## Robot Pose EKF ########## -->
  <remap from="imu_data" to="/zed2i/zed_node/imu/data" />
  <remap from="odom" to="/zed2i/zed_node/odom" />
  <node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
      <param name="output_frame" value="odom"/>
      <param name="base_footprint_frame" value="base_link"/>
      <param name="freq" value="30"/>
      <param name="sensor_timeout" value="3.0"/>
      <param name="odom_used" value="true"/>
      <param name="imu_used" value="true"/>
      <param name="vo_used" value="true"/>
      <param name="gps_used" value="false"/>
      <param name="debug" value="true"/>
      <param name="self_diagnose" value="true"/>
</node>
```

### RTAB-Map
- **rtabmap_odom**: Optional RTAB-Map node for stereo odometry using ZED2 images, which remaps ZED2 stereo image and camera info topics for visual odometry.
```yaml
<!-- ########## RtabMap ########## -->
<!-- <node pkg="rtabmap_odom" type="stereo_odometry" name="rtabmap_stereo_odometry" output="screen">
    <remap from="/left/image_rect" to="/zed2i/zed_node/left/image_rect_color"/>
    <remap from="/right/image_rect" to="/zed2i/zed_node/right/image_rect_color"/>
    <remap from="/left/camera_info" to="/zed2i/zed_node/left/camera_info"/>
    <remap from="/right/camera_info" to="/zed2i/zed_node/right/camera_info"/>
    <param name="approx_sync" value="true"/>
    <param name="approx_sync_max_interval" value="0.017"/>
</node> -->
```

### Point Cloud to LaserScan Conversion
- **pointcloud_to_laserscan**: Converts ZED2 point cloud data to LaserScan format for use in 2D navigation. Launches a sample from the `depthimage_to_laserscan` package for this conversion.
```yaml
  <!-- ########## Convert ZED2 Point CLoud / Depth Image to LaserScan ########## -->
  <!-- <include file="$(find depthimage_to_laserscan)/launch/launchfile_sample.launch"/> -->
  <include file="$(find navstack_pub)/launch/pointcloud_to_laserscan.launch"/>
```

### Manual Pose Publisher
- **localization_data_pub**: Publishes robot’s pose manually by interacting with RViz, aiding manual localization setup if required.
```yaml
  <!-- ########## Publishing Pose to Localize Manually ########## -->
  <node pkg="localization_data_pub" type="rviz_click_to_2d" name="rviz_click_to_2d"></node>
```

### RViz Visualisation
- **rviz**: Launches RViz with a predefined configuration file, `rviz_navigation.rviz`, to visualize the robot’s environment, map, and navigation status.
```yaml
<!-- ########## Rviz Visualisation ########## -->
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find navstack_pub)/rviz/rviz_navigation.rviz"/>
```
---

## Misc

├──[Stage ROS](https://github.com/Adipks/autonomous_navigation/blob/main/Readme_stageros.md)

├──[Mapserver](https://github.com/Adipks/autonomous_navigation/blob/main/mapserver.md)

└──[Rviz configs](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/rviz/rviz_navigation.rviz)
