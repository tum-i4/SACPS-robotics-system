#!/usr/bin/env python

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "07/2019"


"""
IMPORTANT:
During the implementation of a wrapper for the evaluation framework, this node was updated and
changed significantly. 
Since the map_provider of the wrapper worked better and exactly as wanted, major parts
of it were just copy and pasted into this node. This is why both nodes are very similar.

TASK:
---------------------------------------------
Node takes the OccupancyGrid of the map_server and transforms (especially scales) this into a new map with less cells (better performance for our own algorithms)

HELPFUL INFOMRATION:
---------------------------------------------
If you are familiar with the map layout provided by the official ROS map_server: this is exactly the same layout!

If "map" is the received object (OccupancyGrid taken from the PROVIDED_MAP_TOPIC):
The startpoint of the map (first element of the array) is the bottom left corner of the inital image of the map (identical with the layout of the map from map_server).
This bottom left corner (first array element) maps to the real world position given by map.info.origin.
From this origin / start point the x direction is horizontal (to the right) and the y direction is vertical (to the top).
Each cell has the size map.info.resolution x map.info.resolution (resolution is in m/cell).
The map is then a rectangle (or normally square) which has from the start point <map.info.width> cells in x direction and <map.info.height> in y direction.
If you want to have the real world distances in m: length/width = map.info.width*map.info.resolution and height = map.info.height*map.info.resolution

If you have a real world position (x,y) and want to get the cell containing this position:
  cell_x = min(int((x - map.info.origin.position.x) / map.info.resolution), map.info.width-1)
  cell_y = min(int((y - map.info.origin.position.y) / map.info.resolution), map.info.height-1)
  index = cell_x + cell_y * map.info.width
  map.data[index]

If you have a cell index of the map/grid array and want to know the real world position (x, y) in m:
(The position will be the bottom left corner of the cell. To get the whole area of the cell, expand the position by map.info.resolution in x and in y direction)
  cell_x = int(index % ros_map.info.width)    #[number of cells in x direction]
  cell_y = int(index / ros_map.info.width)    #[number of cells in y direction]
  x = map.info.origin.position.x + cell_x * map.info.resolution
  y = map.info.origin.position.y + cell_y * map.info.resolution

For path planning and dirt generation I recommend using the center of the cells:
The resulting center of cell map.data[index] is in real world:
  cell_x = int(index % ros_map.info.width)    #[number of cells in x direction]
  cell_y = int(index / ros_map.info.width)    #[number of cells in y direction]
  x = map.info.origin.position.x + (cell_x + 0.5) * map.info.resolution
  y = map.info.origin.position.y + (cell_y + 0.5) * map.info.resolution
"""

# IMPORTS
# ---------------------------------------------

import rospy
import numpy
import math
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------

# CONSTANTS:

# Enables more ROS prints (for testing/debugging)
PRINT_ON = False

BASE_MAP_TOPIC = "robot_0/map"
NEW_MAP_TOPIC = "modified_occupancy_grid"

NEW_RES = 0.2  # m/cell (resolution of the new transformed map)

# VARIABLES:
# (they will be overridden -> do not adjust them)
base_res = 0.0  # m/cell
base_width = 0.0  # cells
base_height = 0.0  # cells
base_origin = Pose()
real_width = 0.0  # m (= base_width * base_res)
real_height = 0.0  # m (= base_height * base_res)

boundary_x_min = 0.0  # m (will be updated in the beginning)
boundary_x_max = 0.0  # m (will be updated in the beginning)
boundary_y_min = 0.0  # m (will be updated in the beginning)
boundary_y_max = 0.0  # m (will be updated in the beginning)

map_publisher = None


# CODE
# ---------------------------------------------


def print_map(ros_map, map_name):
    # Only for testing issues: prints ros_map
    # ros_map is of type OccupancyGrid
    map_array = list(ros_map.data)

    # local parameters:
    occupied_sign = "X"
    occupied_color = '\033[93m'  # '\033[93m' = yellow , '\033[91m' = red
    free_sign = ":"
    free_color = ''  # none/white
    end_color = '\033[0m'  # indicates end of color

    rospy.loginfo(rospy.get_caller_id() + ": " + map_name + " is seen by the wrapper as the following one:\n\t(free = '%s' [0], occupied = '%s' [100 or -1]).\n\n\tSize w: %d cells x h: %d cells (%.2f m x %.2f m)\n\tResolution: %.2f m/cell\n\tOrigin: (%.2f , %.2f)" % (
        free_sign, occupied_sign, ros_map.info.width, ros_map.info.height, real_width, real_height, ros_map.info.resolution, ros_map.info.origin.position.x, ros_map.info.origin.position.y))

    array_string = ""
    image_string = ""
    for index in range(0, len(map_array)):
        # Check new line for both strings:
        if index % ros_map.info.width == 0:
            array_string += "\n\t"
            image_string += "\n\t"
        # Indicate symbol for array element:
        if map_array[index] == 0:
            array_string += free_color + ":" + end_color + " "
        else:
            array_string += occupied_color + "X" + end_color + " "
        # Indicate symbol for image perspective (translate the index):
        x = int(index % ros_map.info.width)  # x = x_old
        y = (ros_map.info.height - 1) - int(index /
                                            ros_map.info.width)  # y = (height-1) - y_old
        image_index = x + y * ros_map.info.width
        if map_array[image_index] == 0:
            image_string += free_color + ":" + end_color + " "
        else:
            image_string += occupied_color + "X" + end_color + " "

    print(
        "\n\tARRAY perspective (based on the actual order of the map/grid array):\n\t[first array element (origin) is top left and then row-major]" + array_string + "\n")
    print(
        "\n\tMAP IMAGE perspective (based on the initial image of the map):\n\t[origin is bottom left and then row-major]" + image_string + "\n")


def publish_map(new_map, res, height, width, origin, enable_print, first_time):
    # new_map is an array of the map/grid
    ros_map = OccupancyGrid()
    if new_map:  # Only when a new map exists:
        ros_map.data = new_map
        # Edit meta data of new_map
        ros_map.info = MapMetaData()
        ros_map.info.map_load_time = rospy.Time.now()
        ros_map.info.resolution = res
        ros_map.info.height = height
        ros_map.info.width = width
        ros_map.info.origin = origin
        # Edit header
        ros_map.header = Header()
        ros_map.header.stamp = rospy.Time.now()

        # Publish new generated map
        map_publisher.publish(ros_map)
        if PRINT_ON:
            rospy.loginfo(rospy.get_caller_id() +
                          "\tModified occupancy map was published")

        # Print map (if wanted), but only once in the beginning
        if enable_print and first_time:
            map_name = "The transformed map"
            print_map(ros_map, map_name)
            first_time = False


def is_occupied(_map, x, y):
    # check if cell at real world position (x,y) is occupied or not (with a static obstacle like a wall)

    # rounding to solve more or less the general float problem (e.g. a 2.0 after division will be represented as 1.9999999...)
    # cell_x = int(round(round(x, 6) / round(_map.info.resolution, 6), 6))
    # cell_y = int(round(round(y, 6) / round(_map.info.resolution, 6), 6))
    cell_x = min(int(round(round(x - _map.info.origin.position.x, 6) /
                           round(_map.info.resolution, 6), 6)), _map.info.width - 1)
    cell_y = min(int(round(round(y - _map.info.origin.position.y, 6) /
                           round(_map.info.resolution, 6), 6)), _map.info.height - 1)
    # since width should be always an integer, the index will be integer, too
    index = cell_x + cell_y * _map.info.width  # because array is in row-major order

    '''
    print('THE SIZE O S::::::::::::::::::::::::')
    print(len(received_map))
    array = numpy.array(received_map).reshape([640,640])
    array_litle = numpy.zeros([45,20])
    q = 0
    print(numpy.shape(array))
    print(numpy.shape(array_litle))
    for i in range(216,640-216,4):
        l = 0
        for j in range(216,640-192,12):
            #if array[i,j] == -1:
            # #   try:
            # #       array_litle[q,l] = 1
            # #   except:
            #        pass
            #elif array[i,j] == 100:
            #    try:
            #        array_litle[q,l] = 2
            #    except:
            #        pass
            #else:
            #    try:
            #        array_litle[q,l] = array[i,j]
            #    except:
            #        pass
            
            try:
                value = numpy.max(array[i-2:i+2,j-6:j+6])
            except:
                pass

            if value == -1:
                try:
                    array_litle[q,l] = 1
                except:
                    pass
            elif value == 100:
                try:
                    array_litle[q,l] = 2
                except:
                    pass
            else:
                try:
                    array_litle[q,l] = value
                except:
                    pass

            l = l + 1
        q = q + 1
    print(array_litle)
    '''
    return _map.data[index] != 0


def transform(base_map, new_res, new_width, new_height):
    # Create a new map on basis of the received map (but with other sizes)
    new_map = []  # will be row-major

    rospy.loginfo(rospy.get_caller_id() +
                  "\n\n\t\tTrying to transform the occupancy grid from cell size %.2f to %.2f (map size: from %dx%d to %dx%d)\n" % (base_res, new_res, base_width, base_height, new_width, new_height))

    # 2dim for-loop for indexing the resized cell of the new map (row-major order!)
    # The rows are indexed by y values [y is real distance (in m)]
    for y in numpy.arange(boundary_y_min, boundary_y_max, new_res):
        # The columns are indexed by x values [x is real distance (in m)]
        for x in numpy.arange(boundary_x_min, boundary_x_max, new_res):
            # The interesting/selected cell reaches now from x to x+new_res and y to y+new_res
            # The question is now if all "old" cells inside this new cell are free.
            # Then we can also state the new cell as free (0), otherwise we state it as occupied (100), even if only one old cell is occupied
            free = True
            # 2dim for-loop for indexing all cells of the old OccupancyGrid which are inside the new (selected) cell of the new map
            for j in numpy.arange(y, y + new_res, base_res):
                for i in numpy.arange(x, x + new_res, base_res):
                    if is_occupied(base_map, i, j):
                        free = False
                        break  # When there is already one occupied old cell, then the checks of the others can be skipped
                if not free:  # Same goes for the other axis loops: When there is already one occupied old cell, then the checks of the others can be skipped
                    break
            # Select the right number base on the state
            if free:
                new_map.append(0)
            else:
                new_map.append(100)

    # New map is completely created
    rospy.loginfo(rospy.get_caller_id() +
                  "\n\n\t\tOccupancy grid transformation is finished (cell: from %.2f to %.2f --> map: from %dx%d to %dx%d)\n" % (base_res, new_res, base_width, base_height, new_width, new_height))

    return new_map


def callback_forward_map(wrapper_map):
    # just forward it on the internal topic
    map_publisher.publish(wrapper_map)


def map_transformer():
    global map_publisher, base_res, base_width, base_height, base_origin, real_width, \
        real_height, boundary_x_min, boundary_x_max, boundary_y_min, boundary_y_max
    # Node init
    rospy.init_node('map_transformer', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + "\tMap transformer node initialized")

    # Init publisher (to topic modified_occupancy_grid)
    map_publisher = rospy.Publisher(NEW_MAP_TOPIC,
                                    OccupancyGrid, queue_size=100)
    rospy.loginfo(rospy.get_caller_id(
    ) + "\tMap transformer publisher initialized. Will publish to " + NEW_MAP_TOPIC)

    # Check if wrapper is active:
    active_wrapper = False
    if rospy.has_param("/wrapper_namespace"):
        wrapper_namespace = rospy.get_param("/wrapper_namespace")
        active_wrapper = rospy.get_param(
            "/" + wrapper_namespace + "/active_wrapper")

    # If the wrapper is actually providing the map, we can just publish the transformed map from the map_provider:
    if active_wrapper:
        provided_map_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/provided_map_topic")
        rospy.Subscriber(provided_map_topic, OccupancyGrid,
                         callback_forward_map)
        rospy.loginfo(rospy.get_caller_id() +
                      "\tMap transformer subscribed to " + provided_map_topic + " and will just forward it")
    else:
        # No wrapper is active --> do it "alone":
        # get base map (provided by map_server directly from the provided map image)
        base_map = rospy.wait_for_message(BASE_MAP_TOPIC, OccupancyGrid)

        base_res = round(base_map.info.resolution, 6)  # m/cell
        base_width = base_map.info.width  # cells
        base_height = base_map.info.height  # cells
        base_origin = base_map.info.origin  # Pose

        # should be same as new_height*new_res
        real_width = round(base_width * base_res, 6)  # m
        # should be same as new_width*new_res
        real_height = round(base_height * base_res, 6)  # m

        # Normally, the whole image is the map, but the default image from the
        # practical course needs some cutting:
        expander = 100000  # because of the general float problem
        if ((base_res * expander) == (0.05 * expander)
            and base_width == 640
            and base_height == 640
            and base_origin.position.x == -16.2
                and base_origin.position.y == -16.2):
            # with these parameters, it should be the default image
            # use only the selected section of the image
            boundary_x_min = -5.0
            boundary_x_max = 5.0
            boundary_y_min = -5.0
            boundary_y_max = 5.0
        else:
            # otherwise use complete image
            boundary_x_min = base_origin.position.x  # normally 0.0
            boundary_x_max = base_origin.position.x + real_width
            boundary_y_min = base_origin.position.y  # normally 0.0
            boundary_y_max = base_origin.position.y + real_height

        # the new resolution leads to new cell numbers:
        new_real_width = round(boundary_x_max - boundary_x_min, 6)
        new_real_height = round(boundary_y_max - boundary_y_min, 6)
        new_width = int(round(new_real_width / NEW_RES, 6))  # cells
        new_height = int(round(new_real_height / NEW_RES, 6))  # cells

        # if a map section was selected, the origin will be a new one:
        new_origin = Pose(position=Point(x=boundary_x_min, y=boundary_y_min, z=0.0),
                          orientation=base_origin.orientation)

        # create/transform new map
        new_map = transform(base_map, NEW_RES, new_width, new_height)

        print_transformed_map = True

        rate = rospy.Rate(1)  # Hz
        first_time = True
        while not rospy.is_shutdown():
            # publish map (and maybe print it the first time)
            publish_map(new_map, NEW_RES,
                        new_height, new_width, new_origin, print_transformed_map, first_time)

            if first_time:
                first_time = False
            rate.sleep()

    # Transforming or forwarding process is triggered by the incoming messages

    rospy.spin()


if __name__ == '__main__':
    try:
        map_transformer()
    except rospy.ROSInterruptException:
        pass
