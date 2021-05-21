# Dirt detection
This package contains the source code and resources to implement the functionality of the Dirt detector node.

Author: Team 2, SSACPS Praktikum 2019

Adapted by: Sergio Quijano

## Description
With the help of the laser scanner that the robots are equipped with, this node is in charge of publishing information of the detected dirt locations. As the robots move around in the environment, the information of the laser scanner is used to model a partial observation of the context, that later can be used for knowledge aggregation and tasks management. Each robot is equipped with a 2D laser scanner capable of sensing 360 degrees that collects a set of data around the robot, and limited to the maximum laser scanner’s detection distance. The collected data is transformed into a point cloud, in which each point corresponds to a beam of the laser scanner and represents an object in the context. The detection node only needs to consider dirt tasks. Therefore, a filtering process is carried out to eliminate points in the cloud that correspond to static objects in the context and other robots present in the environment. Then, the remaining points in the cloud are published to the Goal manager as a list of tasks in the context map.

A robot $`R`$ issues a subjective opinion $`w_{i,j}^{R}`$ for each cell $`(i,j)`$ in the grid map that it is within its detection range $`r`$ (see picture below). This opinion depends on the Euclidean distance $`D`$ of the cell from the robot and models whether that cell contains dirt or not.

<img src="ros/documentation_resources/dirt_detection.png" alt="Dirt detection" width="270" height="270" />

TODO include formula to calculate subjective opinions (or reference to it)


The further a cell is from the robot, the higher the uncertainty mass $`u_X`$ of the robot's opinion. Observations at the edge of LIDAR sensor range are considered highly uncertain, but can still contribute during knowledge aggregation; hence, we assign an uncertainty value of 0.99 instead of a totally uncertain opinion (i.e., $`u_X=1`$). If a robot detects dirt on a cell, the belief mass $`b_X`$ becomes the complement of $`u_X`$ and the disbelief mass $`d_X`$ for that cell becomes zero. On the contrary, if no dirt is detected,  $`b_X`$ becomes zero and $`d_X`$ becomes the complement of $`u_X`$. The base rate $`a_X`$ is always considered to be the default base rate for a binary domain, i.e. $`a_X=1/2`$. Finally, no subjective opinions are issued for (i) cells which are occupied by static obstacles (e.g. walls) since these are assumed to be accurately detected, (ii) cells that lie outside of the detection range of the robot.

When all the subjective opinions for the partial context are generated, a list of these opinions is published to the Knowledge aggregator node.

## ROS Topics
### Publishes to:
* `/detected_dirt` [[goal_manager_msgs/GoalObject.msg](ros/src/goal_manager_msgs/msg/GoalObject.msg))]: Publishes information about a detected dirt location.
* `/partial_observation` [[knowledge_aggregator_msgs/PartialObservation.msg](ros/src/goal_manager_msgs/msg/GoalObjectList.msg))]: Publishes a list of subjective logic opinions for the partial observations of the robot

### Subscribed to:
* `/scan` [sensor_msgs/LaserScan]: Listen for robot's sensor scan data.
* `/map` [nav_msgs/OccupancyGrid]: Listen for info about the occupancy map.
* `/robot_locations` [[robot_pose_publisher/RobotLocations.msg](ros/src/robot_pose_publisher/msg/RobotLocations.msg))]: Listen for a list with the poses of all robots in the simulation.

## Launch parameters
The following parameters need to be provided to initialize the node.  
* `robot_specifier`: ID of the robot running the current instance of this node
* `false_positive`: Flag to enable generation false positive.
* `false_positive_prob`: False positive probability threshold  

These parameters can be edited in the corresponding master launch file (see [test directory](ros/test)) and, they in turn are passed to the [robot_meta](ros/src/robot_meta/launch/robot_meta.launch) launch file to start the node.


