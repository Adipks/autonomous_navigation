<h1>Time Elastic Band - Local Planner</h1>


<p>Time Elastic Band (TEB) is a dynamic local planner used in robotic navigation, especially for systems that need to account for the robot's dynamics and optimize
paths in real time. TEB stands out because it creates flexible, optimized trajectories in both space and time. This makes it especially useful for robots that have
non-holonomic constraints (such as car-like robots) and require obstacle avoidance, smooth motion, and responsiveness to dynamic environments.</p>

<h3>How TEB Works</h3>

TEB plans a trajectory by modeling it as a time-parameterized elastic band, which represents a series of connected waypoints or "poses" along a path. These poses are adjusted dynamically to optimize the path, considering several factors:

<h4>Trajectory Representation:</h4>
The trajectory consists of a sequence of time-stamped waypoints, each including the robot’s position, orientation, and time intervals between them. These time intervals are adjusted based on the robot's dynamics and constraints, ensuring smooth movement.
<h4>Cost Function and Optimization:</h4>
TEB uses a cost function to optimize the trajectory. This function combines various aspects like distance from obstacles, the robot's kinematic constraints, time efficiency, and velocity smoothness. The planner seeks to minimize the overall cost by iteratively adjusting the trajectory.
<h4>Handling Obstacle Avoidance:</h4>
During the optimization process, the trajectory is "stretched" and "compressed" like an elastic band to avoid obstacles. As the robot moves, TEB continuously recalculates the path, ensuring it adapts to changing environments while avoiding collisions.
<h4>Time-Dependent Optimization:</h4>
TEB takes into account the robot's velocity and acceleration limits. By adjusting the time intervals between waypoints, TEB ensures the robot doesn't exceed its dynamic constraints (e.g., max velocity or acceleration) and reaches its goal efficiently.
<h4>Non-Holonomic Constraints:</h4>
For robots that can't move sideways (like car-like robots), TEB integrates these non-holonomic constraints into the trajectory optimization. It ensures that the planned path is feasible for the robot's motion model, meaning the robot can physically follow the path.
<h4>Multi-Horizon Planning:</h4>

TEB can plan in both short-term and long-term horizons. In dynamic environments where obstacles might appear unexpectedly, TEB provides short-term optimizations that focus on the immediate vicinity of the robot. It adjusts the trajectory in real time while still maintaining a broader long-term goal.


The following Documentation provides a overview of the parameters that have been tuned to satisfy our robot's constraints. 
[Time Elastic Band](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/teb_params.md)


Reference [ ROS WIKI ](http://wiki.ros.org/teb_local_planner)
