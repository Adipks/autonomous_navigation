# Map Server Package

The `map_server` package in ROS is responsible for loading and publishing maps for 2D navigation. It provides static map data to other nodes, especially for autonomous navigation tasks, by reading the map from a YAML file and broadcasting it on the `/map` topic. This package is commonly used in mobile robot navigation stacks to localize the robot within a predefined environment.

## Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Usage](#usage)
- [Parameters](#parameters)
- [Example Launch](#example-launch)
- [Common Issues](#common-issues)

---

### Overview

The `map_server` node loads a map file and publishes it as a static 2D occupancy grid. This is essential for navigation tasks where a robot must know its position and plan routes within a given environment. The map is commonly represented as an occupancy grid (a grid where each cell is either free, occupied, or unknown).

### Key Features

- **Static Map Loading**: Loads a map only once at the start and keeps it static (useful for predefined environments).
- **Occupancy Grid Format**: The map is published as an `OccupancyGrid` message.
- **Frame ID Customization**: Allows customization of the map's coordinate frame.
  
### Usage

To use `map_server`, ensure you have a `.yaml` file for your map, which should point to an image file representing the map. 

#### Map File Structure

The YAML file should contain the following fields:
- `image`: Path to the image file representing the map.
- `resolution`: Map resolution in meters per pixel.
- `origin`: [x, y, yaw] representing the origin of the map in the world frame.
- `occupied_thresh` and `free_thresh`: Thresholds to distinguish occupied and free cells in the image.
- `negate`: Whether to invert the colors (e.g., `1` for white = free, black = occupied).

#### Example YAML File

```yaml
image: "path/to/map.pgm"
resolution: 0.05
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

### Parameters

The `map_server` node takes the following parameters:

- **frame_id**: The name of the coordinate frame to associate with the map (`map` by default).
- **filename**: The path to the YAML file that describes the map (specified in launch files).

### Example Launch

Here’s an example of how to launch `map_server` in a ROS launch file:

```xml
<launch>
  <arg name="map_file" default="$(find my_robot_package)/maps/my_map.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)">
    <param name="frame_id" value="map"/>
  </node>
</launch>
```

### Common Issues

- **Incorrect File Path**: Ensure the path to the YAML map file is correct.
- **Resolution Mismatch**: Verify that the resolution in the YAML file matches the actual map.
- **Thresholding Issues**: Adjust `occupied_thresh` and `free_thresh` if map obstacles or free spaces aren’t accurately represented.

### Additional Resources

- [ROS Wiki - map_server](http://wiki.ros.org/map_server)
- [OccupancyGrid Message Documentation](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)

---

This file provides a basic guide to setting up and using `map_server` in ROS, which is integral to autonomous navigation in known environments.
Navigate back: [Back](https://github.com/Adipks/autonomous_navigation/tree/main)
