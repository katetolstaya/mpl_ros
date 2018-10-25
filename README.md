# MRSL Motion Primitive Library ROS
[![wercker status](https://app.wercker.com/status/d282a628f39dac13997c792b2298bde0/s/master "wercker status")](https://app.wercker.com/project/byKey/d282a628f39dac13997c792b2298bde0)
- - -
A ROS wrapper for [Motion Primitive Library](https://sikang.github.io/motion_primitive_library/) v1.2. Video of the original paper of "Search-based Motion Planning for Quadrotors using Linear Quadratic Minimum Time Control" has been uploaded at the follwing link: [youtube](https://youtu.be/LMe72buMky8).
The package is still under maintenance, the API may change occasionally, please use `git log` to track the latest update.

Packages:
  - `motion_primitive_library`: back-end for planning trajectory in various environments
  - `planning_ros_msgs`: ROS msgs used in storing, visualizing and communicating
  - `planning_ros_utils`: ROS utils for interfacing with MPL, it also includes mapping and rviz plugins
  - `DecompROS`: tool for convex decomposition and visualization
  - `mpl_external_planner`: several planners that build on the `motion_primitive_library`
  - `mpl_test_node`: example ROS nodes (see following Examples)

## Installation
#### Dependancy:
  - `ROS`(Indigo+)
  - [`catkin_simple`](https://github.com/catkin/catkin_simple)

##### Compile
Before compiling, make sure submodules are on their corresponding commits.
To initialize the submodule `motion_primitive_library` and `DecompROS`, run following commands:
```bash
$ cd /PATH/TO/mpl_ros
$ git submodule update --init --recursive
```

###### 1) Using Catkin:
```bash
$ mv mpl_ros ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

###### 2) Using Catkin Tools (recommended):
```bash
$ mv mpl_ros ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin config -DCMAKE_BUILD_TYPE=Release
$ catkin b
```

## Example Usage
The planner inside `mpl_ros` including:
  - `OccMapPlanner`: uses 2D occupancy grid map
  - `VoxelMapPlanner`: uses 3D voxel grid map
  - `EllipsoidPlanner`: uses 3D point cloud and models robot as ellipsoid in SE(3)
  - `PolyMapPlanner2D`: uses 2D polygonal map and moving obstacles

Following examples demonstrate some of these planners:

#### Example 1 (plan in occ/voxel map)
Simple test using the built-in data in a voxel map can be run using the following commands:
```bash
$ cd ./mpl_test_node/launch/map_replanner_node
$ roslaunch rviz.launch
$ roslaunch test.launch
```

Then, you can set a new goal and then replan by running:

```bash
$ set_goal.sh
$ replan.sh
```


The planning results are visualized in Rviz as following:

2D Occ Map | 3D Voxel Map
:--------- | :-----------
<img src="./mpl_test_node/samples/sample1.png" width="220"> | <img src="./mpl_test_node/samples/sample2.png" width="256">




