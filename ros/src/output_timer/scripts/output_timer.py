#! /usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "12/2020"

""" 
This is only needed for the wrapper / evaluation framework.
It sends a signal every second to the wrapper to trigger the saving of the latest recorded outputs.
Also it can be used to collect all needed output values and publish them on the correct topic for the wrapper.
"""

import math
import time

import rospy
from std_msgs.msg import Empty, Int32, String, Float32, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Pose, Twist, Point

from goal_manager_msgs.msg import DirtModel
from commons_msgs.msg import Goal
from robot_pose_publisher.msg import RobotLocation, RobotLocations

# Node name
NODE_NAME = "output_timer"

terminate_node = False

robot_count = 2  # will be updated by parameters

robots_started_status = []

spawned_dirt_counter = 0
detected_dirt_counter = 0
finished_dirt_counter = 0
finished_dirt_counter_msgs = 0
traveled_distance_total = 0.0
roaming_status = []
robots_in_exploration = 0
robot_crashes = 0  # currently not supported (will always be zero)
completed_tasks_per_robot = []
execution_ratio_between_robots = ""
robot_last_locations = {}
current_robot_locations = None
current_goal_locations = None
# list of lists with two elements [x,y]:
current_undetected_dirt_locations = []
# list of lists with two elements [x,y]:
current_detected_dirt_locations = []
# for each dirt another list in form [x, y, "status", spawn_time, finished_time] with status=undetected/detected/finished
all_dirt_ever = []

theoretical_time_pub = None
spawned_dirt_pub = None
detected_dirt_pub = None
finished_dirt_pub = None
traveled_distance_total_pub = None
robots_in_exploration_pub = None
robot_crashes_pub = None
execution_ratio_between_robots_pub = None
actual_dirt_existence_time_pub = None
theoretical_dirt_existence_time_pub = None
current_robot_locations_pub = None
current_undetected_dirt_locations_pub = None
current_detected_dirt_locations_pub = None
current_goals_locations_pub = None


def shutdown_callback(message):
    global terminate_node
    # message is of type Empty and means that the nodes should terminate
    terminate_node = True


def spawned_dirt_callback(message):
    global spawned_dirt_counter, current_undetected_dirt_locations, all_dirt_ever
    # when a new message is received, it means that a new dirt has spawned
    # print("Spawned:\n" + str(message))
    current_undetected_dirt_locations.append([
        message.pose.position.x, message.pose.position.y])

    spawned_dirt_counter += 1
    spawned_dirt_pub.publish(Int32(data=spawned_dirt_counter))

    all_dirt_ever.append(
        [message.pose.position.x, message.pose.position.y, "undetected", rospy.get_time(), None])


def detected_dirt_callback(message):
    global detected_dirt_counter, current_detected_dirt_locations, current_undetected_dirt_locations, all_dirt_ever
    # when a new message is received, it means that a new dirt was detected (can already be detected!)
    # print("Detected:\n" + str(message))
    dirt = [message.x, message.y]
    # check if it is already detected:
    if dirt in current_detected_dirt_locations:
        rospy.loginfo(
            f"'{rospy.get_caller_id()}': Received detected dirt was already detected (can happen!): {str(dirt)}\n")
    else:
        # now, it is definitely a new detected dirt:
        if dirt in current_undetected_dirt_locations:
            current_undetected_dirt_locations.remove(dirt)
        else:
            rospy.logerr(
                f"'{rospy.get_caller_id()}': A newly detected dirt was not in the undetected dirt list: {str(dirt)}\n")
        current_detected_dirt_locations.append(dirt)

        detected_dirt_counter += 1
        detected_dirt_pub.publish(Int32(data=detected_dirt_counter))

        for dirt_element in all_dirt_ever:
            if dirt_element[0] == dirt[0] and dirt_element[1] == dirt[1]:
                if dirt_element[2] == "undetected":
                    # detected one was found
                    dirt_element[2] = "detected"
                    break
                # otherwise it is another one which was earlier at this position: do nothing and continue with search


def finished_dirt_callback(message):
    global finished_dirt_counter, finished_dirt_counter_msgs, current_detected_dirt_locations, all_dirt_ever
    # when a new message is received, it means that a new dirt was finished
    # print("Finished:\n" + str(message))
    dirt = [message.x, message.y]
    if dirt in current_detected_dirt_locations:
        current_detected_dirt_locations.remove(dirt)

        finished_dirt_counter += 1
        finished_dirt_pub.publish(Int32(data=finished_dirt_counter))

        for dirt_element in all_dirt_ever:
            if dirt_element[0] == dirt[0] and dirt_element[1] == dirt[1]:
                if dirt_element[2] == "detected":
                    # finished one was found
                    dirt_element[2] = "finished"
                    dirt_element[4] = rospy.get_time()
                    break
                # otherwise it is another one which was earlier at this position: do nothing and continue with search
    else:
        rospy.logerr(
            f"'{rospy.get_caller_id()}': A finished dirt was not in the detected dirt list (we will not count it as finished!): {str(dirt)}\n")


def robot_crashes_callback(message):
    global robot_crashes
    robot_crashes += 1
    robot_crashes_pub.publish(Int32(data=robot_crashes))


def locations_callback(message):
    global robot_last_locations, traveled_distance_total, current_robot_locations
    # get travaled distance from location differences and publish also just the current positions

    # for each robot:
    for current_location in message.locations:
        if current_location.robot_id in robot_last_locations:
            last_location = robot_last_locations[current_location.robot_id]
            x_diff = abs(current_location.pose.position.x -
                         last_location.pose.position.x)
            y_diff = abs(current_location.pose.position.y -
                         last_location.pose.position.y)
            last_step = math.sqrt(x_diff * x_diff + y_diff * y_diff)
            traveled_distance_total += last_step
        robot_last_locations[current_location.robot_id] = current_location
        id_splits = current_location.robot_id.split("_")
        r_index = int(id_splits[-1])
        current_robot_locations.data[r_index *
                                     2] = current_location.pose.position.x
        current_robot_locations.data[r_index *
                                     2 + 1] = current_location.pose.position.y
    current_robot_locations_pub.publish(current_robot_locations)
    traveled_distance_total_pub.publish(Float32(data=traveled_distance_total))


def roaming_callback_creator(robot_index):
    # can dynamically create callback functions for the roaming status of each robot (during run-time)
    def _function(message):
        global roaming_status
        if message.data == "roaming":
            roaming_status[robot_index] = True
        else:
            roaming_status[robot_index] = False

    return _function


def cmd_vel_callback_creator(robot_index, publisher):
    # can dynamically create callback functions for the cmd_vel of each robot (during run-time)
    def _function(message):
        # directly forward it to the correct topic (via correct publisher)
        publisher.publish(message)

    return _function


def robot_start_callback_creator(robot_index):
    # can dynamically create callback functions for the robot_start_movement of each robot (during run-time)
    def _function(message):
        global robots_started_status
        robots_started_status[robot_index] = True

    return _function


def current_goal_callback_creator(robot_index, publisher):
    # can dynamically create callback functions for the current goal location of each robot (during run-time)
    def _function(message):
        global current_goal_locations
        current_goal_locations.data[robot_index * 2] = message.x
        current_goal_locations.data[robot_index * 2 + 1] = message.y
        publisher.publish(current_goal_locations)

    return _function


def task_completed_callback_creator(robot_index):
    # can dynamically create callback functions for the completed tasks of each robot (during run-time)
    def _function(message):
        global completed_tasks_per_robot, execution_ratio_between_robots
        # if we get a message, it means the robot has finished a new task
        completed_tasks_per_robot[robot_index] += 1

        execution_ratio_between_robots = str(completed_tasks_per_robot[0])
        for index in range(1, len(completed_tasks_per_robot)):
            execution_ratio_between_robots += "-" + \
                str(completed_tasks_per_robot[index])
        execution_ratio_between_robots_pub.publish(
            String(data=execution_ratio_between_robots))

    return _function


def main_loop():
    global spawned_dirt_pub, detected_dirt_pub, finished_dirt_pub, robot_count, traveled_distance_total_pub, completed_tasks_per_robot, current_robot_locations, current_goal_locations, \
        robots_in_exploration_pub, robot_crashes_pub, execution_ratio_between_robots_pub, roaming_status, robots_in_exploration, execution_ratio_between_robots, theoretical_time_pub, robots_started_status, \
        actual_dirt_existence_time_pub, theoretical_dirt_existence_time_pub, current_robot_locations_pub, current_undetected_dirt_locations_pub, current_detected_dirt_locations_pub, current_goals_locations_pub, \
        current_undetected_dirt_locations, current_detected_dirt_locations, all_dirt_ever

    rospy.init_node(NODE_NAME, anonymous=True)

    # check if wrapper/framework is active or not
    if rospy.has_param("/wrapper_namespace"):
        wrapper_namespace = rospy.get_param("/wrapper_namespace")
        # the simulator was started with the wrapper, but it could be that
        # the wrapper is not active (does not provide dirt etc.)
        wrapper_active = rospy.get_param(
            "/" + wrapper_namespace + "/active_wrapper")

    if wrapper_active:
        # Parameters
        recording_trigger_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/recording_trigger_topic")
        shutdown_sim_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/shutdown_sim_topic")
        mandatory_data_output = rospy.get_param(
            "/" + wrapper_namespace + "/mandatory_data_output")
        # optional_data_output = rospy.get_param(
        #     "/" + wrapper_namespace + "/optional_data_output")
        robot_count = rospy.get_param(
            "/" + wrapper_namespace + "/sim_no_of_robots")
        sim_setup_end_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/sim_setup_end_topic")

        # Publishers
        sim_setup_end_pub = rospy.Publisher(
            sim_setup_end_topic, Empty, queue_size=100)
        trigger_pub = rospy.Publisher(
            recording_trigger_topic, Empty, queue_size=100)
        theoretical_time_pub = rospy.Publisher(
            mandatory_data_output["theoretical_elapsed_time"]["topic"], Float32, queue_size=100)
        spawned_dirt_pub = rospy.Publisher(
            mandatory_data_output["spawned_dirt_number"]["topic"], Int32, queue_size=100)
        detected_dirt_pub = rospy.Publisher(
            mandatory_data_output["detected_dirt_number"]["topic"], Int32, queue_size=100)
        finished_dirt_pub = rospy.Publisher(
            mandatory_data_output["finished_dirt_number"]["topic"], Int32, queue_size=100)
        traveled_distance_total_pub = rospy.Publisher(
            mandatory_data_output["traveled_distance_total"]["topic"], Float32, queue_size=100)
        robots_in_exploration_pub = rospy.Publisher(
            mandatory_data_output["robots_in_exploration"]["topic"], Int32, queue_size=100)
        robot_crashes_pub = rospy.Publisher(
            mandatory_data_output["robot_crashes"]["topic"], Int32, queue_size=100)
        execution_ratio_between_robots_pub = rospy.Publisher(
            mandatory_data_output["execution_ratio_between_robots"]["topic"], String, queue_size=100)
        actual_dirt_existence_time_pub = rospy.Publisher(
            mandatory_data_output["actual_dirt_existence_time"]["topic"], Float32, queue_size=100)
        theoretical_dirt_existence_time_pub = rospy.Publisher(
            mandatory_data_output["theoretical_dirt_existence_time"]["topic"], Float32, queue_size=100)
        current_robot_locations_pub = rospy.Publisher(
            mandatory_data_output["current_robot_locations"]["topic"], Float32MultiArray, queue_size=100)
        current_undetected_dirt_locations_pub = rospy.Publisher(
            mandatory_data_output["current_undetected_dirt_locations"]["topic"], Float32MultiArray, queue_size=100)
        current_detected_dirt_locations_pub = rospy.Publisher(
            mandatory_data_output["current_detected_dirt_locations"]["topic"], Float32MultiArray, queue_size=100)
        current_goals_locations_pub = rospy.Publisher(
            mandatory_data_output["current_goals_locations"]["topic"], Float32MultiArray, queue_size=100)

        # r0_cmd_vel_pub = rospy.Publisher(
        #     optional_data_output["r0_cmd_vel"]["topic"], Twist, queue_size=100)
        # r1_cmd_vel_pub = rospy.Publisher(
        #     optional_data_output["r1_cmd_vel"]["topic"], Twist, queue_size=100)
        # cmd_vel_pubs = [r0_cmd_vel_pub, r1_cmd_vel_pub]

        # Subscribers
        shutdown_sub = rospy.Subscriber(
            shutdown_sim_topic, Empty, shutdown_callback)
        robot_crash_sub = rospy.Subscriber(
            "/robot_crash", Empty, robot_crashes_callback)
        spawned_dirt_sub = rospy.Subscriber(
            "new_dirt", DirtModel, spawned_dirt_callback)
        detected_dirt_sub = rospy.Subscriber(
            "new_goal", Goal, detected_dirt_callback)
        finished_dirt_sub = rospy.Subscriber(
            "goal_attained", Goal, finished_dirt_callback)
        locations_sub = rospy.Subscriber(
            "robots_locations", RobotLocations, locations_callback)
        roaming_subs = []
        # cmd_vel_subs = []
        task_completed_subs = []
        robots_start_moving_subs = []
        current_goal_subs = []
        for robot_index in range(robot_count):
            roaming_subs.append(rospy.Subscriber("robot_" + str(robot_index) +
                                                 "/roaming_status", String, roaming_callback_creator(robot_index)))
            # cmd_vel_subs.append(rospy.Subscriber("robot_" + str(robot_index) +
            #                                      "/cmd_vel", Twist, cmd_vel_callback_creator(robot_index, cmd_vel_pubs[robot_index])))
            task_completed_subs.append(rospy.Subscriber("robot_" + str(robot_index) +
                                                        "/task_completed", String, task_completed_callback_creator(robot_index)))
            robots_start_moving_subs.append(rospy.Subscriber("robot_" + str(robot_index) +
                                                             "/robot_start_moving", Empty, robot_start_callback_creator(robot_index)))
            current_goal_subs.append(rospy.Subscriber("robot_" + str(robot_index) +
                                                      "/current_goal", Point, current_goal_callback_creator(robot_index, current_goals_locations_pub)))

        pubs = [theoretical_time_pub, spawned_dirt_pub, detected_dirt_pub, finished_dirt_pub, traveled_distance_total_pub,
                robots_in_exploration_pub, robot_crashes_pub, execution_ratio_between_robots_pub, actual_dirt_existence_time_pub,
                theoretical_dirt_existence_time_pub, current_robot_locations_pub, current_undetected_dirt_locations_pub,
                current_detected_dirt_locations_pub, current_goals_locations_pub]
        # pubs.extend(cmd_vel_pubs)
        subs = [robot_crash_sub, spawned_dirt_sub, detected_dirt_sub,
                finished_dirt_sub, locations_sub]
        subs.extend(roaming_subs)
        # subs.extend(cmd_vel_subs)
        subs.extend(task_completed_subs)
        subs.extend(robots_start_moving_subs)
        subs.extend(current_goal_subs)
        pub_topics = [pub.resolved_name for pub in pubs]
        sub_topics = [sub.resolved_name for sub in subs]
        rospy.loginfo(f"'{rospy.get_caller_id()}' is ready -"
                      f"\n\t...will publish the end of the sim setup to '{sim_setup_end_pub.resolved_name}'"
                      f"\n\t...will listen continously to '{shutdown_sub.resolved_name}' for shutdown signal"
                      f"\n\t...will publish the recording trigger signal to '{trigger_pub.resolved_name}'"
                      f"\n\t...will listen continously to these topics for new output values/metrics:\n\t{str(sub_topics)}"
                      f"\n\t...will publish all recorded values to the correctly topics for wrapper:\n\t{str(pub_topics)}\n")

        # Initial values:
        robots_started_status = [False for _ in range(robot_count)]
        roaming_status = [False for _ in range(robot_count)]
        completed_tasks_per_robot = [0 for _ in range(robot_count)]
        execution_ratio_between_robots = str(completed_tasks_per_robot[0])
        for index in range(1, len(completed_tasks_per_robot)):
            execution_ratio_between_robots += "-" + \
                str(completed_tasks_per_robot[index])

        current_robot_locations = Float32MultiArray(data=[], layout=MultiArrayLayout(data_offset=0, dim=[MultiArrayDimension(
            label="robots", size=robot_count, stride=robot_count)]))
        current_robot_locations.data = [0.0 for _ in range(robot_count * 2)]

        current_goal_locations = Float32MultiArray(data=[], layout=MultiArrayLayout(data_offset=0, dim=[MultiArrayDimension(
            label="robots", size=robot_count, stride=robot_count)]))
        current_goal_locations.data = [0.0 for _ in range(robot_count * 2)]

        current_undetected_dirt_locations_msg = Float32MultiArray(data=[], layout=MultiArrayLayout(data_offset=0, dim=[MultiArrayDimension(
            label="robots", size=robot_count, stride=robot_count)]))
        current_undetected_dirt_locations_msg.data = [
            0.0 for _ in range(robot_count * 2)]

        current_detected_dirt_locations_msg = Float32MultiArray(data=[], layout=MultiArrayLayout(data_offset=0, dim=[MultiArrayDimension(
            label="robots", size=robot_count, stride=robot_count)]))
        current_detected_dirt_locations_msg.data = [
            0.0 for _ in range(robot_count * 2)]

        # Initial publications:
        rospy.sleep(0.1)
        theoretical_time_pub.publish(
            Float32(data=0.0))
        spawned_dirt_pub.publish(Int32(data=spawned_dirt_counter))
        detected_dirt_pub.publish(Int32(data=detected_dirt_counter))
        finished_dirt_pub.publish(Int32(data=finished_dirt_counter))
        traveled_distance_total_pub.publish(
            Float32(data=traveled_distance_total))
        robots_in_exploration_pub.publish(Int32(data=robots_in_exploration))
        robot_crashes_pub.publish(Int32(data=robot_crashes))
        execution_ratio_between_robots_pub.publish(
            String(data=execution_ratio_between_robots))
        current_robot_locations_pub.publish(current_robot_locations)
        current_goals_locations_pub.publish(current_goal_locations)
        current_undetected_dirt_locations_pub.publish(
            current_undetected_dirt_locations_msg)
        current_detected_dirt_locations_pub.publish(
            current_detected_dirt_locations_msg)
        actual_dirt_existence_time_pub.publish(Float32(data=0.0))
        theoretical_dirt_existence_time_pub.publish(Float32(data=0.0))

        robots_started = False
        time_interval = 1.0  # every second
        # Attention: it seems like rospy-time/rate has often a delay, which grows (not in all nodes)
        # Therefore, to be sure, I will use not rate = rospy.Rate and rate.sleep()
        # start_time_ros = rospy.get_time()
        # rate = rospy.Rate(1 / time_interval)
        start_time = time.time()
        initial_start_time = time.time()
        while not rospy.is_shutdown() and not terminate_node:
            if not robots_started:
                if all(robots_started_status):
                    robots_started = True
                    sim_setup_end_pub.publish(Empty())
                    # start_time_ros = rospy.get_time()
                    start_time = time.time()

            if robots_started:
                current_undetected_dirt_locations_msg.data = [
                    item for sublist in current_undetected_dirt_locations for item in sublist]
                current_detected_dirt_locations_msg.data = [
                    item for sublist in current_detected_dirt_locations for item in sublist]
                current_undetected_dirt_locations_pub.publish(
                    current_undetected_dirt_locations_msg)
                current_detected_dirt_locations_pub.publish(
                    current_detected_dirt_locations_msg)

                actual_existence_sum = 0.0
                theoretical_max_existence_sum = 0.0
                current_time = rospy.get_time()
                for dirt_element in all_dirt_ever:
                    theoretical_max_existence_sum += (
                        current_time - dirt_element[3])
                    if dirt_element[2] == "finished":
                        actual_existence_sum += (dirt_element[4] -
                                                 dirt_element[3])
                    else:
                        actual_existence_sum += (current_time -
                                                 dirt_element[3])
                actual_dirt_existence_time_pub.publish(
                    Float32(data=actual_existence_sum))
                theoretical_dirt_existence_time_pub.publish(
                    Float32(data=theoretical_max_existence_sum))

                robots_in_exploration = 0
                for is_roaming in roaming_status:
                    if is_roaming:
                        robots_in_exploration += 1
                robots_in_exploration_pub.publish(
                    Int32(data=robots_in_exploration))

                # theoretical_elapsed_time_ROS = rospy.get_time() - start_time_ros
                theoretical_elapsed_time = time.time() - start_time
                # print("ROS: " + str(theoretical_elapsed_time_ROS) +
                #       " - Real: " + str(theoretical_elapsed_time) + " - Diff: " + str(theoretical_elapsed_time_ROS - theoretical_elapsed_time))
                theoretical_time_pub.publish(
                    Float32(data=theoretical_elapsed_time))

                rospy.sleep(0.1)
                trigger_pub.publish(Empty())
            # rate.sleep()
            time.sleep(time_interval -
                       ((time.time() - initial_start_time) % time_interval))


if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
