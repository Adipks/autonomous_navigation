
# Set up and Test Optimization

## Description

you will learn how to run the trajectory optimization and how to change the underlying parameters in order to set up custom behavior and performance.





---

## Contents
1. Install Package
2. Configure Optimization
3. Optimization of a single Trajectory
4. Optimization of multiple Trajectories in distinctive Topologies

---

## Install Package

Install the `teb_local_planner` package from the official ROS repositories:

```bash
sudo apt-get install ros-noetic-teb-local-planner
```

If you build the package from source, make sure to install the dependencies first:

```bash
rosdep install teb_local_planner
```

Supplementary material for the following tutorials is available in the `teb_local_planner_tutorials` package.  
Check it out from source in order to inspect the files and easily change parameters:

```bash
cd ~/catkin_ws/src
git clone https://github.com/rst-tu-dortmund/teb_local_planner_tutorials.git
# Install dependencies, e.g.
sudo apt-get install ros-noetic-stage-ros
```

Or install the examples from the official repositories if you just want to run the scripts:

```bash
sudo apt-get install ros-noetic-teb-local-planner-tutorials
```

---

## Configure Optimization

The package includes a simple test node (`test_optim_node`) that optimizes a trajectory between a fixed start and goal pose. First, we will configure the planning of a single trajectory (Timed-Elastic-Band) between start and goal. Afterward, we will activate and set up planning in distinctive topologies.

---

## Optimization of a Single Trajectory

### Step 1: Deactivate Parallel Planning
Deactivate parallel planning using the ROS parameter server (make sure to have a roscore running):

```bash
rosparam set /test_optim_node/enable_homotopy_class_planning False
```

### Step 2: Launch the Test Node
Launch `test_optim_node` in combination with the preconfigured `rviz` node for visualization:

```bash
roslaunch teb_local_planner test_optim_node.launch
```

A new `rviz` window should open, showing the following visualization:  
*Optimization of a single trajectory*

Three point obstacles are included. They are represented as `interactive_markers` and can be changed by clicking and holding the blue circle around each individual obstacle.

*Optimization of a single trajectory in a different obstacle configuration*

Since the Timed-Elastic-Band utilizes a local optimization scheme, the trajectory cannot transit across obstacles.

### Step 3: Customize the Optimization with `rqt_reconfigure`
Run `rqt_reconfigure` to adjust optimization parameters at runtime:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

You can customize the optimization, but be careful when modifying parameters, as it might lead to undesired convergence behavior or bad performance. If you experience bad performance, try adjusting the following parameters:

- Decrease `no_inner_iterations`
- Decrease `no_outer_iterations`
- Increase `dt_ref`
- Decrease `obstacle_poses_affected`

---

## Optimization of Multiple Trajectories in Distinctive Topologies

Now, we address the problem of local optimization schemes and enable parallel planning in distinctive topologies. This extended planner is enabled by default and requires more computational resources.

### Step 1: Enable Parallel Planning
Restart `roscore` or reactivate the extended planner:

```bash
rosparam set /test_optim_node/enable_homotopy_class_planning True
```

### Step 2: Launch the Test Node for Multiple Trajectories
Launch `test_optim_node` in combination with the preconfigured `rviz` node for visualization:

```bash
roslaunch teb_local_planner test_optim_node.launch
```

A new `rviz` window should open, showing the following visualization:  
*Optimization of multiple trajectories*

As in the first section, all obstacles can now be moved using the mouse. The currently best trajectory (in terms of lowest optimization cost) is highlighted with red arrows at each trajectory configuration.

### Step 3: Customize with `rqt_reconfigure`
Run `rqt_reconfigure` again to customize the parameters for parallel planning in distinctive topologies:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

---

Now, you can experiment with different parameters and optimize your robot's navigation in different environments.

## Additional links 
├── [TEB_Tutorials ROS](https://wiki.ros.org/teb_local_planner/Tutorials)

├── [Tuning Parameters](https://wiki.ros.org/teb_local_planner/Tutorials/Setup%20and%20test%20Optimization)


## Note
the configurations of the param files given below are only a small portion of the teb_planner_param file present in the param folder ,it is esential you understand the difference between all the parameters and how by changing these parameters we can configure the planner to work based on 
our specific needs.

[Carlike param configs](https://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots)

[diff_drive param configs](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/param/teb_local_planner_diff_drive.yaml)

[holonomic robots config params](https://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20holonomic%20robots)

### Diff drive config params 
```yaml
 # Robot
         
 max_vel_x: 0.4
 max_vel_x_backwards: 0.25
 max_vel_y: 0.0
 max_vel_theta: 0.3
 acc_lim_x: 0.5
 acc_lim_theta: 0.5
 min_turning_radius: 0.0 # diff-drive robot (can turn on place!)
```
### Carlike drive config params
```yaml
  # Robot
          
  max_vel_x: 0.5 # earlier 2.0
  max_vel_x_backwards: 0.25 # earlier 2.0
  max_vel_y: 0
  max_vel_theta: 0.3 # the angular velocity is also bounded by min_turning_radius in case of a carlike robot (r = v / omega)
  acc_lim_x: 0.5 # earlier 2.0
  acc_lim_theta: 0.5 # earlier 0.5

  # ********************** Carlike robot parameters ********************
  min_turning_radius: 1.0 # 0.672       # Min turning radius of the carlike robot (compute value using a model or adjust with rqt_reconfigure manually)
  wheelbase: 0.70               # Wheelbase of our robot
  cmd_angle_instead_rotvel: True # stage simulator takes the angle instead of the rotvel as input (twist message)
  # ********************************************************************
```
[Back](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/local_costmap.md)
