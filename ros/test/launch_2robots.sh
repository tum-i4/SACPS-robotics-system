#!/bin/sh

# Set rocore and gazebo ports where the nodes will pub/sub
#export ROS_MASTER_URI=http://localhost:11315
#export GAZEBO_MASTER_URI=http://localhost:11335

# gnome-terminal --tab -- bash -c "source /home/sebastian/catkin_ws_Malte/knowledge_aggregation/ros/devel/setup.bash; export TURTLEBOT3_MODEL=burger; roslaunch master.launch gen_seed:=115 use_sl:=true sl_operator:=CBF false_positive:=true false_negative:=true adaptive_scheduling:=true spawn_interval:=40 gui:=false; exec bash"
# without false_positive and false_negative:
gnome-terminal --tab -- bash -c "source /home/sebastian/catkin_ws_Malte/knowledge_aggregation/ros/devel/setup.bash; export TURTLEBOT3_MODEL=burger; roslaunch master.launch gen_seed:=115 use_sl:=true sl_operator:=CBF false_positive:=false false_negative:=false adaptive_scheduling:=true spawn_interval:=5 gui:=true; exec bash"

# Storing data
# name format is r#_sl(0/1)_fp(0/1)_fn(0/1), where # is number of robots (here 2), sl is subjective logic, fn false negatives etc. 0 = false, 1 = true
# gnome-terminal --tab --working-directory=/home/sebastian/catkin_ws_Malte/knowledge_aggregation/ros/results/data -- bash -c "rosbag record -O r2_sl1_fp1_fn1_spi80_comp /active_tasks /opinion_map /modified_occupancy_grid /robot_0/amcl_pose /robot_1/amcl_pose /partial_observation; exec bash"

sleep 2
gnome-terminal --tab -- bash -c "rviz -d multi_robot.rviz; exec bash"
