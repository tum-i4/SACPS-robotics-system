#!/bin/bash

# the following parameters are not altered
use_sl=false
false_positive=true
false_negative=true
adaptive_scheduling=true
robot_0_fpp=0.2
robot_0_fnp=0.2
robot_1_fpp=0.2
robot_1_fnp=0.2

# Initial values of seed (incremented by 1)
gen_seed=131

# THIS IS WHEN ABLE TO OPTN MULTIPLE TABS (NOT SSH)
#num=1
#for i in `seq 4`;
#do
#	# Initial value of spawn_interval (in seconds) (inremented by 10s)
#	spawn_interval=50
#
#	for j in `seq 5`;
#	do
#		# Bash statement
#		test_str="Test Num. "$num": gen_seed = "$gen_seed" spawn_interval = "$spawn_interval
#		echo $test_str
#
#		# Running Test	
#		gnome-terminal --tab -- bash -c "roslaunch master.launch gen_seed:=$gen_seed use_sl:=$use_sl false_positive:=$false_positive false_negative:=$false_negative adaptive_scheduling:=$adaptive_scheduling spawn_interval:=$spawn_interval robot_0_fpp:=$robot_0_fpp robot_0_fnp:=$robot_0_fnp robot_1_fpp:=$robot_1_fpp robot_1_fnp:=$robot_1_fnp"
#
#		# Storing data
#		gnome-terminal --tab --working-directory=/home/ge73xus/catkin_ws/src/knowledge_aggregation/ros/results/data -- bash -c "rosbag record -O r2_sl"$use_sl"_fp"$false_positive"_fn"$false_negative"_spi"$spawn_interval"_seed"$gen_seed"_r0fpp"$robot_0_fpp"_r0fnp"$robot_0_fnp"_r1fpp"$robot_1_fpp"_r1fnp"$robot_1_fnp" /active_tasks /opinion_map /modified_occupancy_grid /robot_0/amcl_pose /robot_1/amcl_pose /partial_observation" 
#		# Note: Add: ; exec bash     to the end for the shell to remain after killing it
#
#		# Let the ROS test run for 45 minuites before stopping it
#		sleep 45m
#		rosnode kill -a; killall -9 rosmaster; killall -9 roscore
#
#		# wait for 20 seconds to make sure that all the rosnodes have stopped properly
#		sleep 20s
#		
#		# increment variables
#		let num=`expr $num + 1`
#		let spawn_interval=$((spawn_interval + 10))
#	done
#
#	# increment variable
#	gen_seed=$((gen_seed + 1))
#done


# When using SSH use Mutex instead

num=1
for i in `seq 4`;
do
	# Initial value of spawn_interval (in seconds) (inremented by 10s)
	spawn_interval=50

	for j in `seq 5`;
	do
		# Bash statement
		test_str="Test Num. "$num": gen_seed = "$gen_seed" spawn_interval = "$spawn_interval
		echo $test_str

		filename="r2_sl"$use_sl"_fp"$false_positive"_fn"$false_negative"_spi"$spawn_interval"_seed"$gen_seed"_r0fpp"$robot_0_fpp"_r0fnp"$robot_0_fnp"_r1fpp"$robot_1_fpp"_r1fnp"$robot_1_fnp
		FILE="../results/data/"$filename".bag"
		echo $FILE
		
		# check if file already exists, then ext loop iteration
		if [ -f "$FILE" ];
		then
			echo "File already exists, move on to next test"
		else
			# Run test in detached mutex session:	
			tmux new-session -d -s "ros" "roslaunch master.launch gen_seed:=$gen_seed use_sl:=$use_sl false_positive:=$false_positive false_negative:=$false_negative adaptive_scheduling:=$adaptive_scheduling spawn_interval:=$spawn_interval robot_0_fpp:=$robot_0_fpp robot_0_fnp:=$robot_0_fnp robot_1_fpp:=$robot_1_fpp robot_1_fnp:=$robot_1_fnp"

			# Storing data in detached mutex session (first go to correct directory)
			tmux new-session -d -s "saving" "cd /home/ge73xus/catkin_ws/src/knowledge_aggregation/ros/results/data; rosbag record -O "$filename" /active_tasks /opinion_map /modified_occupancy_grid /robot_0/amcl_pose /robot_1/amcl_pose /partial_observation"

			# Let the ROS test run for 45 minuites before stopping it
			sleep 45m
			rosnode kill -a; killall -9 rosmaster; killall -9 roscore

			# wait for 20 seconds to make sure that all the rosnodes have stopped properly
			sleep 20s

			# delete the tmux sessions (should have automatically closed when killing ros)
			tmux kill-session -a -t saving
			tmux kill-session -a -t ros
		
			# increment variables
		fi
		let num=`expr $num + 1`
		let spawn_interval=$((spawn_interval + 10))
	done

	# increment variable
	gen_seed=$((gen_seed + 1))
done



