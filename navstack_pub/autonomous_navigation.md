## Launchfiles and params


> The complete navigation stack has move_base as its fundamental base over which the Autnomous Stack has been built.

Move Base Definitions [Move Base](https://github.com/Adipks/rover_repo/blob/main/README_move_base.md)

Since the Autonomous Stack has been built upon the Move Base there are various parameters that needed to be customized when convoluting various algorithms and localization which is inherently dependent on the Physics of the Rover both from Hardware and the Software.

Here is the list of Params that were fine tuned to acheive the Autonomy:

### Params

├── Adaptive Monte Carlo Localization [AMCL](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/param/amcl_params.yaml)

#### Costmap Parameters

│ ├── [Global Costmap](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/global_costmap.md)

│ ├── [Local Costmap](https://github.com/Adipks/autonomous_navigation/blob/main/navstack_pub/local_costmap.md)

Navigate Back[Back](https://github.com/Adipks/autonomous_navigation/tree/main)

