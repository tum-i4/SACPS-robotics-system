#! /usr/bin/env python3

"""
Important changes due to evaluation framework:
The actual dirt generator is not needed anymore when using the evaluation
framework. Now, this node only starts the global dirt generator of the
wrapper/evaluation framework and then listen to its published dirt.
As soon as one is published, this node will handle it like if the node itself
has created it itself (spawn it). It also forward the active goal list to the
wrapper (at the correct topic).
"""

import random
import numpy
import copy
import threading
import math
from typing import List

import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from std_msgs.msg import Header, Bool

from goal_manager_msgs.msg import GoalObject, GoalObjectList, DirtModel
from commons.OccupancyMap import OccupancyMap, Cell

from wrapper_msgs.msg import DirtObject, DirtObjectList
from robot_pose_publisher.msg import RobotLocation, RobotLocations

# Node name
NODE_NAME = 'dirt_generator'

# Topics and services
# BASE_MAP_TOPIC = '/robot_0/map'
BASE_MAP_TOPIC = 'modified_occupancy_grid'
ACTIVE_TASKS_TOPIC = 'active_tasks'
NEW_DIRT_TOPIC = 'new_dirt'
# same as new dirt topic but of type GoalObject
NEW_DIRT_GOALOBJECT_TOPIC = 'new_dirt_goalObject'

# Gazebo model params
DIRT_MODEL_NAME = "dirt_object_undetected.sdf"
DIRT_MODEL_PACKAGE = "dirt_generator"

# Sedd to be used during simulation
SEED = 120

# Min and max values for random timers
TIME_MIN = 10
TIME_MAX = 10

# Min and max trust level, that is, amount of confidence about the dirt observation
TRUST_MIN = 100
TRUST_MAX = 100

# Debug control constant
DEBUG_ON = True

# will get updated/overridden in the beginning:
# former boundary_x_min, etc.
boundary_x_min = 0.0
boundary_x_max = 0.0
boundary_y_min = 0.0
boundary_y_max = 0.0

wrapper_namespace = ""
wrapper_active = False


class DirtGenerator:
    def __init__(self, seed, spawn_interval):
        self.position_map: List[Point] = []
        self.spawn_number: int = 1
        self.robot_size: float = 0.105 * 2
        self.dirt_pos_tolerance: float = 0.25
        self.occupancy_map: OccupancyGrid
        self.active_dirt_list: List[GoalObject] = list()

        self.model_pub = None
        self.goal_pub = None
        self.scan_sub = None
        self.active_tasks_sub = None
        self.locations_sub = None

        self.new_dirt_sub = None
        self.active_tasks_pub = None

        self.seed = seed
        self.time_interval_min = spawn_interval
        self.time_interval_max = spawn_interval

        self.current_robot_locations = []

        self.occupancy_map = None

        self.__init_publishers()
        self.__init_subscribers()

        # Getting parameters from parameter server
        self.false_positive = rospy.get_param('~false_positive', False)
        if self.false_positive:
            self.num_robots = rospy.get_param('~no_of_robots')
            self.fpp_props = numpy.zeros(self.num_robots)

            # TODO: What about 0 probability i.e. disabelling FP for one of them
            # Assume robots labelled robot_0_.... withi increasing integers
            for i in range(0, self.num_robots):
                self.fpp_props[i] = rospy.get_param('~robot_%d_fpp' % i)

            # convert probabilities into timers
            fp_spawn_intervals = (1.0 - self.fpp_props) / \
                self.fpp_props * spawn_interval

            # Store all spawn intervals in this array. The order is ground truth, robot 0, robot 1, ...
            self.spawn_intervals = numpy.zeros(self.num_robots + 1)
            self.spawn_intervals[0] = spawn_interval
            self.spawn_intervals[1:] = fp_spawn_intervals

        if wrapper_active:
            rospy.loginfo(f"[{NODE_NAME}] node is ready - "
                          f"\n\tlistening for active goals on '{self.active_tasks_sub.resolved_name}"
                          f"\n\tlistening for new (global) dirt on '{self.new_dirt_sub.resolved_name}"
                          f"\n\tpublishing new random dirt to '{self.model_pub.resolved_name}'"
                          f"\n\tpublishing active goals for wrapper to '{self.active_tasks_pub.resolved_name}'")
        else:
            rospy.loginfo(f"[{NODE_NAME}] node is ready - "
                          f"\n\tlistening for active goals on '{self.active_tasks_sub.resolved_name}"
                          f"\n\tpublishing new random dirt to '{self.model_pub.resolved_name}'")

    def __init_subscribers(self):
        self.active_tasks_sub = rospy.Subscriber(ACTIVE_TASKS_TOPIC,
                                                 GoalObjectList, self.__active_tasks_cb)
        self.locations_sub = rospy.Subscriber(
            "robots_locations", RobotLocations, self.__locations_callback)

        if wrapper_active:
            new_wrapper_dirt_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
                "/" + wrapper_namespace + "/new_dirt_topic")
            self.new_dirt_sub = rospy.Subscriber(new_wrapper_dirt_topic,
                                                 DirtObject, self.__new_dirt_cb)

    def __init_publishers(self):
        self.model_pub = rospy.Publisher(
            NEW_DIRT_TOPIC, DirtModel, queue_size=100)
        self.goal_pub = rospy.Publisher(
            NEW_DIRT_GOALOBJECT_TOPIC, GoalObject, queue_size=100)

        if wrapper_active:
            active_dirt_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
                "/" + wrapper_namespace + "/active_dirt_topic")
            self.active_tasks_pub = rospy.Publisher(
                active_dirt_topic, DirtObjectList, queue_size=100)

    def __active_tasks_cb(self, combined_list):
        # Save the received list with all currently active dirt and goals (from topic all_current_dirt_and_goals)
        self.active_dirt_list = list(combined_list.goal_list)

        if wrapper_active:
            # forward it to the wrapper (as DirtObjectList, not as GoalObjectList)
            translated_list = DirtObjectList()
            translated_list.dirt_list = []
            header = Header(stamp=rospy.get_rostime(), frame_id="map")
            for goal in combined_list.goal_list:
                dirt = DirtObject(header=header)
                dirt.id = goal.id
                dirt.pose = goal.pose
                dirt.trust_value = goal.trust_value
                translated_list.dirt_list.append(dirt)
            self.active_tasks_pub.publish(translated_list)

    def __locations_callback(self, message):
        self.current_robot_locations = message.locations

    def __new_dirt_cb(self, message):
        # spawn and publish the new dirt
        self.__spawn_dirt_from_wrapper(message)

    def __comparing_points(self, point1, point2) -> bool:
        """
        Compares two Points and returns true if they are identical (same position with some tolerance)
        """
        return (abs(point1.x - point2.x) <= self.dirt_pos_tolerance and abs(
            point1.y - point2.y) <= self.dirt_pos_tolerance)

    def __check_for_duplicates(self, point, fp_list) -> bool:
        """
        Goes through the list with all currently active dirt positions and compare their positions with the given
        position. Returns true if there is a duplicate position but false positive of different robot so can create new fp task here, false otherwise
        """
        duplicate = False
        # flag = true if false positive of other robot occupies position
        other_fp_duplicate = False

        # Check all already published (active) dirt objects (stored and received from the goal_list)
        if len(fp_list) == 1:
            for dirt in list(self.active_dirt_list):
                if self.__comparing_points(point, dirt.pose.position):
                    print(
                        'Checking for duplicates in the FP list, robot being tested:, FP of existing task in location:, coordinates of task:')
                    print(fp_list)
                    print(dirt.fp)
                    print(point)
                    # i.e. dirt is ground truth or already fp of that robot.
                    if len(dirt.fp) == 0 or fp_list[0] in dirt.fp:
                        duplicate = True
                        return duplicate, other_fp_duplicate
                    else:
                        other_fp_duplicate = True
                        return duplicate, other_fp_duplicate

        else:
            # Ground Truth
            for dirt in list(self.active_dirt_list):
                if self.__comparing_points(point, dirt.pose.position):
                    duplicate = True
                    return duplicate, other_fp_duplicate

        return duplicate, other_fp_duplicate

    def __get_cell_index(self, x, y) -> int:
        cell_x = int((x - self.occupancy_map.origin.x) /
                     self.occupancy_map.resolution)
        cell_y = int((y - self.occupancy_map.origin.y) /
                     self.occupancy_map.resolution)

        index = cell_x + cell_y * self.occupancy_map.width

        return index

    def __is_occupied(self, x, y) -> bool:
        """
        Check if the cell at position (x, y) is occupied or not (with a static obstacle like a wall)
        """

        cell = self.occupancy_map.world2costmap(
            self.occupancy_map.costmap2world(Cell(x, y)))
        index = self.occupancy_map.to_costmap_index(cell)

        return self.occupancy_map.grid[index] != 0

    def __is_occupied_(self, x, y) -> bool:

        index = self.__get_cell_index(x, y)

        return self.occupancy_map.grid[index] != 0

    def __has_occupied_neighbors(self, x, y) -> bool:
        """
        Checks if the neighbor cells (mask: radius of robot size) of the cell at the given position (x,y) are occupied
        Mask is probably larger than needed, but it is okay (safer)
        """

        # Robot radius in cells (according to OccupancyGrid)
        robot_radius = int(
            math.ceil((self.robot_size / 2) / self.occupancy_map.resolution))

        center_index = self.__get_cell_index(x, y)

        # Only check the cells in the square around the center with edge length of twice the robot_radius (the center
        # cell can be ignored, that is why robot_radius-1)
        for r in range(-(robot_radius - 1), (robot_radius - 1)):

            # Is actually not needed because we assume that the robot is symmetrical (square circle)
            for c in range(-(robot_radius - 1), (robot_radius - 1)):

                # Now refine the initial square with transforming the mask to a circle (nearly)
                if math.floor(math.sqrt(r ** 2 + c ** 2)) <= robot_radius:
                    current_index = center_index + c + r * self.occupancy_map.width

                    # if in this circle one of the cells is occupied (!=0), then the given cell is not possible (
                    # return true)
                    if self.occupancy_map.grid[current_index] != 0:
                        return True

        # Only if all cells in the circle mask are free, then the given cell is possible as dirt center
        return False

    def __get_dirt_candidate_cells(self):
        while True:
            if self.occupancy_map:
                # As soon as map and metadata is received (!=0.0), create a static list with all possible positions

                x_min = self.occupancy_map.origin.x
                y_min = self.occupancy_map.origin.y
                x_max = x_min + self.occupancy_map.height * self.occupancy_map.resolution
                y_max = y_min + self.occupancy_map.width * self.occupancy_map.resolution
                x_step = y_step = self.occupancy_map.resolution

                # Take always the center position of the grid cells
                for x in numpy.arange(x_min + x_step / 2, x_max - x_step / 2, x_step):

                    # Take always the center position of the grid cells
                    for y in numpy.arange(y_min + y_step / 2, y_max - y_step / 2, y_step):

                        # Check if it is inside the movement area of the robots
                        if (boundary_x_min <= x <= boundary_x_max) and (boundary_y_min <= y <= boundary_y_max):

                            if not self.__is_occupied_(x, y):
                                self.position_map.append(
                                    Point(x=x, y=y, z=0.0))

                break

            # Sleep one second until self.position_map can be created (occupancy grid, etc. was received)
            # rospy.sleep(1)

    def __generate_point_based_on_prob(self, fp_list) -> Point:
        """
        Generates a random point, based on probabilities (map/distribution)
        Returns random point of type Point(x, y, z)
        """

        # Use a standard distribution (provided by libs/extern formulas) In this case: Beta distribution
        # with alpha=2 and beta=2 (near to normal/Gaussian): ATTENTION: the distribution is applied to a position
        # list, which goes row by row through the map. That means the hot spot (in case of Gaussian) is not a perfect
        # circle in the middle of the map, but the complete rows in the middle of the map (also their boundary cells)
        # We tested it and the distribution is good for our purpose, but keep in mind: it is not a circle in the
        # center!

        # positions were new task can spawn is empty space (position_map) or
        # if it is a false positive, where a false positive of another robot is located

        # NOTE, self.position_map contains a list of all grid centres which are not a wall and where the robot is not currently positioned. It does not care if a grid is already occupied by a task.

        #all_positions = copy.deepcopy(self.position_map)
        # if len(fp_list) == 1: # fp_list should only be size 0 (ground truth) or 1 i.e. single fp
        #    for task in self.active_dirt_list:
        #        if len(task.fp) > 0:
        #            if fp_list[0] not in task.fp:
        #                all_positions.append(task.pose.position)

        index = 0
        possible = False
        while not possible:
            # index = int(random.betavariate(2, 2) * len(self.position_map))
            index = random.randint(0, len(self.position_map) - 1)  # uniform
            x = self.position_map[index].x
            y = self.position_map[index].y

            # Check for occupied neighbors and only add the position if also the neighboring cells are free (the
            # robot needs some space the reach it), otherwise generate a new one and test it
            if not self.__has_occupied_neighbors(x, y):
                # If actual spawning of dirt is enabled, then it should also be checked while generating objects
                # if another dirt object is already at this position (if so, the generation has to be repeated!)
                duplicate, other_fp_duplicate = self.__check_for_duplicates(
                    Point(x, y, 0.0), fp_list)
                if not duplicate:
                    possible = True
                else:
                    rospy.loginfo(
                        rospy.get_caller_id() + "\n\n\tGenerated dirt at (%f | %f) was refused due to already "
                                                "active dirt at this position (duplicate). Generating next "
                                                "one...\n" % (
                            x, y))
            else:
                rospy.loginfo(
                    rospy.get_caller_id() +
                    "\n\n\tGenerated dirt at (%f | %f) was refused due to occupied neighbor "
                    "cells. Generating next one...\n" % (
                        x, y))

        return self.position_map[index], other_fp_duplicate

    def __spawn_dirt(self, dirt: GoalObject, fp_duplicate_flag):
        """
        Spawns dirt object in the map at the position which was generated for the dirt delivered via the input
        parameter of this function
        fp_duplicate_flag = true if there is already a task/dirt spawned as a false positive from another robot.
        """

        # Init service
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        name = "dirt_" + str(self.spawn_number)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(DIRT_MODEL_PACKAGE)
        path = pkg_path + "/" + DIRT_MODEL_NAME
        with open(path, "r") as data:
            model = data.read()
        robot = ""  # can be left empty
        pose = dirt.pose
        frame = ""  # empty or "world" or "map"

        # save name of model combined with position for future deletion (transmit it to goal_list where this is
        # executed)
        new_dirt_model = DirtModel(header=Header(
            stamp=rospy.get_rostime(), frame_id="map"), name=name, pose=pose)
        self.goal_pub.publish(dirt)

        if not fp_duplicate_flag:
            # only spawn it, when there is no robot at the location (or it would crash):
            rate = rospy.Rate(2)  # Hz
            while not rospy.is_shutdown():
                free = True
                for current_location in self.current_robot_locations:
                    if self.__comparing_points(current_location.pose.position, dirt.pose.position):
                        # as soon as one robot is near, we cannot spawn
                        # rospy.loginfo(rospy.get_caller_id() +
                        #               ": Robot too close to a new spawn position. Robot:\n" + str(current_location.pose.position) + "\nDirt:\n" + str(dirt.pose.position) + "\n")
                        free = False
                        break
                if free:
                    # Spawn it
                    self.model_pub.publish(new_dirt_model)
                    spawn_model(name, model, robot, pose, frame)
                    rospy.loginfo(rospy.get_caller_id() +
                                  "\n\n\tNew dirt was spawned\n")
                    break
                else:
                    rospy.loginfo(rospy.get_caller_id() +
                                  ": Cannot spawn dirt, because a robot is too close to it. Wait until it moves away...\n")
                rate.sleep()
        self.spawn_number += 1

    def __spawn_dirt_from_wrapper(self, dirt: DirtObject):
        # FOR EVALUATION FRAMEWORK!
        """
        Spawns dirt object in the map at the position which was generated for the dirt delivered via the input
        parameter of this function
        """

        # Init service
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        name = "dirt_" + str(dirt.id)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(DIRT_MODEL_PACKAGE)
        path = pkg_path + "/" + DIRT_MODEL_NAME
        with open(path, "r") as data:
            model = data.read()
        robot = ""  # can be left empty
        pose = dirt.pose
        frame = ""  # empty or "world" or "map"

        # save name of model combined with position for future deletion (transmit it to goal_list where this is
        # executed)
        new_dirt_model = DirtModel(header=Header(
            stamp=rospy.get_rostime(), frame_id="map"), name=name, pose=pose)

        dirt_as_goal = GoalObject()
        dirt_as_goal.id = dirt.id
        dirt_as_goal.pose = dirt.pose
        dirt_as_goal.trust_value = dirt.trust_value
        dirt_as_goal.fp = []

        # only spawn it, when there is no robot at the location (or it would crash):
        max_waiting_time = 5  # s
        end_time = rospy.get_time() + max_waiting_time
        rate = rospy.Rate(2)  # Hz
        while not rospy.is_shutdown():
            free = True
            for current_location in self.current_robot_locations:
                if self.__comparing_points(current_location.pose.position, dirt.pose.position):
                    # as soon as one robot is near, we cannot spawn
                    # rospy.loginfo(rospy.get_caller_id() +
                    #               ": Robot too close to a new spawn position. Robot:\n" + str(current_location.pose.position) + "\nDirt:\n" + str(dirt.pose.position) + "\n")
                    free = False
                    break
            if free:
                # publish it
                self.model_pub.publish(new_dirt_model)
                self.goal_pub.publish(dirt_as_goal)
                # Spawn it
                spawn_model(name, model, robot, pose, frame)
                rospy.loginfo(rospy.get_caller_id() +
                              "\n\n\tNew dirt was spawned\n")
                self.spawn_number += 1
                break
            else:
                rospy.loginfo(rospy.get_caller_id() +
                              ": Cannot spawn dirt, because a robot is too close to it. Wait until it moves away...\n")
            if rospy.get_time() > end_time:
                rospy.loginfo(rospy.get_caller_id() +
                              ": Waited long enough for the robot to move away. Dirt will be ignored to not hinder future dirt spawnings.\n")
                break
            rate.sleep()

    def generation_process(self):
        """
        Creates a random dirt until the node is shut down
        """

        # Sleep at the beginning because not everything is set up (nodes, topics)
        rospy.sleep(1)

        if not self.false_positive:
            index = 0
            while not rospy.is_shutdown():
                # Create an (increasing) index, a random trust value and a random position for the new dirt
                index += 1
                trust_value = random.randint(TRUST_MIN, TRUST_MAX)
                r_position, other_fp_duplicate = self.__generate_point_based_on_prob([
                ])
                pose = Pose(position=r_position,
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

                # Position/Point should already be in an empty (not occupied) cell, because it is only randomly searched
                # on already free positions

                # Combine everything in the new object
                # creates no false positives thus empty list
                goal = GoalObject(index, pose, trust_value, [])
                rospy.loginfo("\n\n\t(%d)Dirt/Goal generated: [ID: %d, (%f,%f),trust: %d]\n" % (self.seed,
                                                                                                goal.id, goal.pose.position.x, goal.pose.position.y, goal.trust_value))

                # Spawn the dirt (if it is enabled)
                self.__spawn_dirt(goal, other_fp_duplicate)

                # Sleep rest of the (random defined) time
                # sleep_time = random.randint(TIME_MIN, TIME_MAX)
                sleep_time = random.randint(
                    self.time_interval_min, self.time_interval_max)
                rospy.loginfo(
                    "\n\n\tDirt generation will sleep now for %d seconds.\n" % sleep_time)
                rospy.sleep(sleep_time)
        else:
            # including false positive
            next_spawn = copy.deepcopy(self.spawn_intervals)

            index = 0
            while not rospy.is_shutdown():
                # timers that have run out and spawn task
                spawn_ind = numpy.where(next_spawn == min(next_spawn))[0]
                # TODO: Allow false negatives of multiple robots to spawn in the same location

                for ind in spawn_ind:
                    index += 1

                    # create new task.
                    # pose = Pose(position=self.__generate_point_based_on_prob(),
                    #            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

                    # Combine everything in the new object
                    if ind == 0:
                        # create new task.
                        r_position, other_fp_duplicate = self.__generate_point_based_on_prob([
                        ])
                        pose = Pose(position=r_position,
                                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

                        # Ground Truth
                        goal = GoalObject(index, pose, TRUST_MAX, [])
                        rospy.loginfo("\n\n\t(%d)Dirt/Goal (GT) generated: [ID: %d, (%f,%f),trust: %d]\n" % (
                            self.seed, goal.id, goal.pose.position.x, goal.pose.position.y, goal.trust_value))
                    else:
                        # create new task.
                        r_position, other_fp_duplicate = self.__generate_point_based_on_prob([
                                                                                             ind - 1])
                        pose = Pose(position=r_position,
                                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

                        # False Positive of robot ind - 1
                        goal = GoalObject(index, pose, TRUST_MAX, [ind - 1])
                        rospy.loginfo("\n\n\t(%d)Dirt/Goal (FP) generated: [ID: %d, (%f,%f),trust: %d, robot: %d]\n" % (
                            self.seed, goal.id, goal.pose.position.x, goal.pose.position.y, goal.trust_value, ind - 1))

                    # Spawn the dirt (if it is enabled)
                    self.__spawn_dirt(goal, other_fp_duplicate)

                # Sleep rest of the (random defined) time
                sleep_time = min(next_spawn)
                rospy.loginfo(
                    "\n\n\tDirt generation will sleep now for %d seconds.\n" % sleep_time)
                rospy.sleep(sleep_time)

                # update the clocks and reset the ones that have reached 0
                next_spawn = next_spawn - min(next_spawn)
                next_spawn[spawn_ind] = self.spawn_intervals[spawn_ind]

    def dirt_generator(self):
        global boundary_x_min, boundary_x_max, boundary_y_min, boundary_y_max
        if wrapper_active:
            # We only listen to the global generator and spawn its dirt (the subscriber is already in place)
            pass
        else:
            rospy.loginfo(f"[{NODE_NAME}] \n\tWaiting for occupancy map.")
            self.occupancy_map = OccupancyMap.from_message(
                rospy.wait_for_message(BASE_MAP_TOPIC, OccupancyGrid))
            # self.dirt_pos_tolerance = self.occupancy_map.resolution

            # The received map is already transformed and maybe cut.
            # So, we can directly take the corner points provided by it:
            boundary_x_min = self.occupancy_map.origin.x  # normally 0.0
            boundary_x_max = self.occupancy_map.origin.x + \
                round(self.occupancy_map.width *
                      self.occupancy_map.resolution, 6)
            boundary_y_min = self.occupancy_map.origin.y  # normally 0.0
            boundary_y_max = self.occupancy_map.origin.y + \
                round(self.occupancy_map.height *
                      self.occupancy_map.resolution, 6)

            self.__get_dirt_candidate_cells()

            # Seed can be set with a parameter
            # random.seed(SEED)
            random.seed(self.seed)

            # Start generating+spawning dirt
            thread1 = threading.Thread(target=self.generation_process)
            thread1.start()

        rospy.spin()


if __name__ == '__main__':
    # check if wrapper/framework is active or not
    if rospy.has_param("/wrapper_namespace"):
        wrapper_namespace = rospy.get_param("/wrapper_namespace")
        # the simulator was started with the wrapper, but it could be that
        # the wrapper is not active (does not provide dirt etc.)
        wrapper_active = rospy.get_param(
            "/" + wrapper_namespace + "/active_wrapper")

    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    seed_ = rospy.get_param(f'~seed', 100)
    spawn_interval_ = rospy.get_param(f'~spawn_interval', 10)
    dg = DirtGenerator(seed_, spawn_interval_)

    dg.dirt_generator()
