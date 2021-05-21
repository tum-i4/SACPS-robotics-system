#!/usr/bin/env python

import rosbag
import os
import copy
import pickle
import re
import numpy as np
from datetime import datetime, timedelta
from collections import defaultdict
from matplotlib import pyplot as plt

class DataAnalysis:
    """
    DataAnalysis is a class that extracts the data from the rosbag files and analysis it
    """
    def __init__(self, bag_path, bag_file):
        self.bag = rosbag.Bag(os.path.join(bag_path, bag_file + ".bag"))
        # filename
        self.filename = bag_file

        # Data containers
        self.task_data = defaultdict(dict) # task spawning/completion
        self.task_distr = defaultdict(dict) # spawned task distriution
        self.opinion_map = defaultdict(dict) # knowledge aggregated map from logics
        self.observations_map = defaultdict(dict) # knowledge aggregated map from logics
        self.observations_data = defaultdict(dict) # saves list of opinioins with key = [counter]
        #self.robot_map = np.zeros([20,20])
        self.robot_pos = defaultdict(dict)
        self.occupancy_map = None

        # Plotting
        self.fig_counter = 1
        plt.rcParams["figure.figsize"] = [20,6]

        # Get the map resolution/width etc.
        saved_map_info = True
        if saved_map_info:
            self.extract_map_info()
        else:
            # manually saved. Preferred for long tests as saving modified_occupancy_grid
            # requires a lot of memory and is always the same
            self.grid_map_origin_x = -5.0
            self.grid_map_origin_y = -5.0
            self.grid_map_resolution = 0.5
            self.grid_map_width = 20
            self.grid_map_height = 20


    def get_cell_index(self, x, y):
        cell_x = int((x - self.grid_map_origin_x) / self.grid_map_resolution)
        cell_y = int((y - self.grid_map_origin_y) / self.grid_map_resolution)

        index = cell_x + cell_y * self.grid_map_width
        return index

    def extract_map_info(self):
        """
        This function extracts the information from the topic "/modified_occupancy_grid"
        to get the basic information about the map
        """
        for topic, msg, time in self.bag.read_messages(topics='/modified_occupancy_grid'):
            #self.occupancy_map = msg
            self.grid_map_origin_x = msg.info.origin.position.x
            self.grid_map_origin_y = msg.info.origin.position.y
            self.grid_map_resolution = msg.info.resolution
            self.grid_map_width = msg.info.width
            self.grid_map_height = msg.info.height
            
        data = np.array(msg.data)
        data = data.reshape([20,20])
        #plotting grid
        #print(data[:,0:18])
        #data2 = np.rot90(data)
        #print(data2[:,1:19])
        #plt.pcolormesh(np.transpose(data))
        #plt.show()
    #def extract_robot_map(self):
    #    
    #    for topic, msg, time in self.bag.read_messages(topics='/robot_0/map'):
    #        counter_row = 0
    #        counter_col = 0
    #        for index in range(len(msg.data)):
    #            self.robot_map[counter_row][counter_col] = msg.data[index]
    #            counter_row += 1
    #            counter_col += 1
    #            if counter_row == np.shape(self.robot_map)[0]:
    #                counter_row = 0
    #            if counter_col == np.shape(self.robot_map)[1]:
    #                counter_col = 0
    #        #print(self.robot_map)
    #        print(msg.info.resolution)
    #        print(msg.info.width)
    #        print(msg.info.height)
    #        print(msg.info.origin)
    #        print('done')
    
    def extract_observation_info(self):
        """
        This function extracts the information from the topic "/modified_occupancy_grid"
        to get the basic information about the map
        """
        msg_num = 1
        for topic, msg, time in self.bag.read_messages(topics='/partial_observation'):
            time_sec = time.to_nsec() / float(10**9)
            frequency = []
            net_believe = [] # avg (belief - disbelief)
            index_list = []
            so_opinions = [] # list of lists of format [index, belief, disbelief, uncertainty, baserate]
            for so in msg.partial_observation:
                occ_index = self.get_cell_index(so.pose.position.x, so.pose.position.y)
                net_bel = so.belief - so.disbelief
                #print(so.pose.position.x, " ", so.pose.position.y)
                #print(so.belief, " ", so.disbelief)
                so_opinions.append([occ_index, so.belief, so.disbelief, so.uncertainty,so.base_rate])
                try:
                    # if already inside index_list
                    tmp_index = index_list.index(occ_index)
                    net_believe[tmp_index] += net_bel
                    frequency[tmp_index] += 1.

                except ValueError:
                    index_list.append(occ_index)
                    frequency.append(1)
                    net_believe.append(net_bel)

            #print(frequency)
            #print(net_believe)
            #print(index_list)
            for i in range(len(index_list)):
                # divide by respective frequencies to get avg.
                self.observations_map[datetime.fromtimestamp((float(time_sec)))][index_list[i]] = net_believe[i] / frequency[i]
            #print(self.observations_map[datetime.fromtimestamp((float(time_sec)))])
            
            # store opinoins list in observations_data
            self.observations_data[msg_num] = so_opinions
            msg_num += 1
                
    def extract_robot_pos(self):
        """
        This function extracts the position of the robots as a function of time
        """
        for topic, msg, time in self.bag.read_messages(topics='/robot_0/amcl_pose'):
            time_sec = time.to_nsec() / float(10**9)
            
            # x,y position of robot at this time
            pos_list = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.robot_pos['robot_0'][datetime.fromtimestamp(time_sec)] = pos_list
        
        for topic, msg, time in self.bag.read_messages(topics='/robot_1/amcl_pose'):
            time_sec = time.to_nsec() / float(10**9)
            
            # x,y position of robot at this time
            pos_list = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.robot_pos['robot_1'][datetime.fromtimestamp(time_sec)] = pos_list

    def extract_opinion_map(self):
        """
        This function extracts the information from the topic "/opinion_map" to retrieve 
        the aggregated knowledge map craeted by the different logic types.
        """
        for topic, msg, time in self.bag.read_messages(topics='/opinion_map'):
            for index in range(len(msg.so_list)):
                time_sec = time.to_nsec() / float(10**9)
                self.opinion_map[datetime.fromtimestamp((float(time_sec)))][index] = msg.so_list[index]
   
        # Add initial grid of vacuous opinions.
        # To add it at the beginning give it time of 5min before the first published opinion.
        time_delta = timedelta(seconds = 5 * 60)
        time_keys = self.opinion_map.keys()
        init_time = min(time_keys) - time_delta
        vac_opinion = copy.deepcopy(self.opinion_map[time_keys[0]][self.opinion_map[time_keys[0]].keys()[0]])
        vac_opinion.belief = 0.
        vac_opinion.disbelief = 0.
        vac_opinion.uncertainty = 1.
        vac_opinion.base_rate = 0.5

        for i in self.opinion_map[time_keys[0]].keys():

            if self.opinion_map[time_keys[0]][i].belief != -1:
                # not a wall
                self.opinion_map[init_time][i] = vac_opinion
            else:
                self.opinion_map[init_time][i] = self.opinion_map[time_keys[0]][i]


    def extract_active_tasks(self):
        """
        This function extracts the information from the topic "/active_tasks" and puts it
        into the dictionary self.task_data to calculate the average cleaning times.

        Secondly it also creates a dictionary self.task_distr with [times][position index]
        as keys and contains the false positive list (empty for ground truth otherwise robot id) 
        """
        prev_msg_flag = False
        for topic, msg, time in self.bag.read_messages(topics='/active_tasks'):
            time_sec = time.to_nsec() / float(10**9)
            for task in msg.goal_list:
                ID = task.id
                if ID not in self.task_data.keys() or list(task.fp) != self.task_data[ID]['false_positive']:
                    self.task_data[ID]['spawned'] = datetime.fromtimestamp(time_sec)
                    self.task_data[ID]['false_positive'] = list(task.fp)
            
            # check if a goal is achieved, i.e. task in previous msgs but not in this one
            if prev_msg_flag:
                for task in prev_msg.goal_list:
                    ID = task.id
                    msg_ids = [msg_task.id for msg_task in msg.goal_list]
                    if ID not in msg_ids:
                        self.task_data[ID]['completed'] = datetime.fromtimestamp(time_sec)

            # add to self.task_distr
            if len(msg.goal_list) != 0:
                for task in msg.goal_list:
                    # convert position into index
                    pos_index = self.get_cell_index(task.pose.position.x, task.pose.position.y)
                    self.task_distr[datetime.fromtimestamp(time_sec)][pos_index] = list(task.fp)
            else:
                # empty map, so add unoccpied space value (-1) to beginning
                self.task_distr[datetime.fromtimestamp(time_sec)][0] = [-1] 
            
            prev_msg = msg
            prev_msg_flag = True


    def ground_truth_completion_time(self):
        """
        This function calculates the average and variance time needed for the robots to
        complete a task after it is spawned.
        """
        completion_times = []
        for task_id in self.task_data.keys():
            # check that it is ground truth
            if len(self.task_data[task_id]['false_positive']) == 0:
                if 'completed' in self.task_data[task_id].keys():
                    duration = self.task_data[task_id]['completed'] - self.task_data[task_id]['spawned']
                    completion_times.append(duration.total_seconds())
        #print(completion_times)

        # calculate average
        avg_compl_time = np.mean(completion_times)

        # calculate the variance, ddof = 1 for unbiased estiamte.
        compl_times_stdev = np.std(completion_times, ddof=1) 

        print("mean = {} and stdev = {}".format(avg_compl_time, compl_times_stdev))
        return avg_compl_time, compl_times_stdev

    def completion_time_history(self, plotting_flag):
        """
        This function plots the time history of when ground truth and false positive tasks
        were completed
        """
        gt_completion_times = []
        fp_completion_times = []

        first_spawn = self.task_data[1]['spawned']
        for task_id in self.task_data.keys():
            # check that it is ground truth
            if len(self.task_data[task_id]['false_positive']) == 0:
                if 'completed' in self.task_data[task_id].keys():
                    completed_time = self.task_data[task_id]['completed'] - first_spawn 
                    gt_completion_times.append(completed_time.total_seconds())
            else:
                if 'completed' in self.task_data[task_id].keys():
                    completed_time = self.task_data[task_id]['completed'] - first_spawn 
                    fp_completion_times.append(completed_time.total_seconds())


        # sort list from lowest to highest completion time
        #sorted_indexes = numpy.argsort(gt_completion_times)
        gt_completion_times = sorted(gt_completion_times)
        fp_completion_times = sorted(fp_completion_times)
        if plotting_flag:
            # Ground Thruths
            fig = plt.figure(self.fig_counter)
            gt_completion_number = range(1,len(gt_completion_times) + 1)
            plt.plot(gt_completion_times, gt_completion_number, label='GT completion times')
            plt.legend()
            plt.savefig('plots/gt_completion_history' + self.filename + '_{}.png'.format(self.fig_counter), bbox_inches='tight')
            self.fig_counter += 1  

            # False positives
            fig = plt.figure(self.fig_counter)
            fp_completion_number = range(1,len(fp_completion_times) + 1)
            plt.plot(fp_completion_times, fp_completion_number, label='fp completion times')
            plt.savefig('plots/fp_completion_history' + self.filename + '_{}.png'.format(self.fig_counter), bbox_inches='tight')
            self.fig_counter += 1

            print(gt_completion_number)
            print(gt_completion_times)
        return gt_completion_times, fp_completion_times

    def task_type_counter(self):
        """
        This function counts the number of ground truhts, false positives that have been cleaned 
        as well as the total number of ground truths and false positives that have been spawned. 
        """
        gt_cleaned = 0
        total_gt_spawned = 0
        fp_cleaned = 0
        total_fp_spawned = 0
        for task_id in self.task_data.keys():
            # check that it is ground truth
            if len(self.task_data[task_id]['false_positive']) == 0:
                total_gt_spawned += 1
                if 'completed' in self.task_data[task_id].keys():
                    gt_cleaned += 1
            else:
                total_fp_spawned += 1
                if 'completed' in self.task_data[task_id].keys():
                    fp_cleaned += 1
        
        return gt_cleaned, total_gt_spawned, fp_cleaned, total_fp_spawned
    
    def save_data(self):
        """
        This function saves the data in form of a list in a pickle dump. To do this, the datetime
        information mus be discareded
        """
        tmp_opinion_map_dict = defaultdict(dict)
        sorted_keys = sorted(self.opinion_map.keys())
        
        print('LEN = ', len(self.opinion_map.keys()))
        for i in range(len(self.opinion_map.keys())):
            data_lists = []
            for ind in self.opinion_map[sorted_keys[i]].keys():
                tmp_so = self.opinion_map[sorted_keys[i]][ind]
                data_lists.append([ind, tmp_so.belief, tmp_so.disbelief, tmp_so.uncertainty, tmp_so.base_rate])         
                        
            tmp_opinion_map_dict[i] = data_lists


        save_file = open('./data/' + self.filename + '.pkl', 'wb')
        pickle.dump([self.observations_data, tmp_opinion_map_dict], save_file)
        #pickle.dump([data_analysis.task_data, data_analysis.task_distr, data_analysis.opinion_map, data_analysis.observations_map, data_analysis.robot_pos], save_file)
        save_file.close()


    def plot_opinion_maps(self):
        """
        This function plots both the opinion map and the task distribution side by side on a 
        2D grid, so that they can be visually compared.
        """
        # check which has smaller temporal resolution
        #if len(self.task_distr.keys()) < len(self.opinion_map.keys()):
        #    time_list = sorted(self.opinion_map.keys())
        #else:
        #    time_list = sorted(self.task_distr.keys())
      
        #previous_believe_array = np.zeros([self.grid_map_height, self.grid_map_width])
 
        # alternatively use robot positioning as timing
        if True:
            #  use robots as time
            time_list = sorted(self.robot_pos['robot_0'].keys())
        else:
            # use opinion map as time
            time_list = sorted(self.opinion_map.keys())
            time_list = [tmp_t for tmp_t in time_list if tmp_t > datetime(year = 2020, month = 9, day = 8, hour = 16, minute=3, second = 19) and tmp_t < datetime(year = 2020, month = 8, day = 8, hour = 16, minute=7, second = 9)]
 
        for time in time_list:
            task_distr_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            
            # determine the times closest to 'time' in both arrays.
            # find closest time that is older i.e. smaller thatn time. (so dont see future)
            
            arr_td = np.array([(i - time).total_seconds() for i in self.task_distr.keys()])
            if np.all(arr_td >= 0):
                # take closest one
                closest_time_td = min(self.task_distr.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_td_index =np.where(arr_td== -1*min(np.absolute(arr_td[arr_td<0])))[0][0]
                closest_time_td = self.task_distr.keys()[closest_time_td_index]
            
            arr_op = np.array([(i - time).total_seconds() for i in self.opinion_map.keys()])
            if np.all(arr_op >= 0):
                # take closest one
                closest_time_op = min(self.opinion_map.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_op_index = np.where(arr_op==-1*min(np.absolute(arr_op[arr_op<0])))[0][0]
                closest_time_op = self.opinion_map.keys()[closest_time_op_index]
            
            # convert data to 2D array. The values are assigned as follows:
            # -1: unoccupied space
            #  0: false positive of robot 0
            #  1: fasle positive of robobt 1
            #  2: false positive of robot 0 and 1
            #  3: ground truth
            for pos_index in self.task_distr[closest_time_td].keys():
                fp_list = self.task_distr[closest_time_td][pos_index]
                if len(fp_list) == 0:
                    task_distr_array[0,pos_index] = 3
                elif len(fp_list) == 1:
                    task_distr_array[0, pos_index] = fp_list[0]
                elif len(fp_list) == 2:
                    task_distr_array[0, pos_index] = 2

            # believe array stores the belive that it is occupied
            # whilst uncertainty array stores the uncertainty
            believe_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            uncertainty_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            expected_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            base_rate_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            for pos_index in self.opinion_map[closest_time_op].keys():
                tmp_op = self.opinion_map[closest_time_op][pos_index]
                believe_array[0, pos_index] = tmp_op.belief
                uncertainty_array[0, pos_index] = tmp_op.uncertainty 
                base_rate_array[0, pos_index] = tmp_op.base_rate
                expected_array[0, pos_index] = tmp_op.belief + tmp_op.base_rate * tmp_op.uncertainty
                
            #print(max(expected_array[0,:]))
            # add walls to the task_distr for reference
            wall_mask = [believe_array[0,:] == -1]
            task_distr_array[0,wall_mask[0]] = -2
               
            # reshape arrays into actual dimensions
            task_distr_array = np.reshape(task_distr_array, [self.grid_map_height, self.grid_map_width])
            believe_array = np.reshape(believe_array, [self.grid_map_height, self.grid_map_width])
            uncertainty_array = np.reshape(uncertainty_array, [self.grid_map_height, self.grid_map_width])
            expected_array = np.reshape(expected_array, [self.grid_map_height, self.grid_map_width])
            base_rate_array = np.reshape(base_rate_array, [self.grid_map_height, self.grid_map_width])
            #print(time)
            #print(believe_array)
            
            # Determine the robot positions at this time
            closest_time_r_0 = min(self.robot_pos['robot_0'], key=lambda x: np.absolute(x-time))
            closest_time_r_1 = min(self.robot_pos['robot_1'], key=lambda x: np.absolute(x-time))
            robot_0_x, robot_0_y = self.robot_pos['robot_0'][closest_time_r_0]
            robot_1_x, robot_1_y = self.robot_pos['robot_1'][closest_time_r_1]
            
            # convert the robot pos into the correct indexes
            robot_0_x = (robot_0_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_0_y = (robot_0_y - self.grid_map_origin_y) / self.grid_map_resolution
            robot_1_x = (robot_1_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_1_y = (robot_1_y - self.grid_map_origin_y) / self.grid_map_resolution
                        
            # plot the arrays side by side
            # Note, pcolormesh automatically flips vertical axis when plotting. i.e. array[0,0] is
            # plotted in the lower left hand corner but top left corner when print(array).
            # Also first transpose the array (i.e. mirror along diagonal) so that combination of 
            # mirror along diagonal and vertical flip alligns the x axis to the right and y axis to
            # the top. This is needed so that robots are in correct position in map when plotted.
            fig, (ax0, ax1, ax2) = plt.subplots(1,3)
            

            #print(believe_array[:,1:19])
            test_list =[]
            for i in range(20):
                for j in range(20):
                    if believe_array[i,j] == -1:
                        test_list.append(100)
                    else:
                        test_list.append(0)
            #print(np.array(test_list).reshape([20,20])[:,1:19])



            # ####### When plotting difference between believes to see result of aggregation
            #diff_b_array =believe_array - previous_believe_array
            #previous_believe_array = believe_array


            im0 = ax0.pcolormesh(task_distr_array)
            ax0.set_title('Task Distribution')
            ax0.set_xlabel('min={}, sec = {}'.format(closest_time_td.minute, closest_time_td.second))
            fig.colorbar(im0, ax=ax0)
            
            im1 =  ax1.pcolormesh(uncertainty_array)
            #im1 =  ax1.pcolormesh(believe_array)
            #im1 =  ax1.pcolormesh(diff_b_array) 
            ax1.set_title('Uncertainty Distributon, robot time = min={}, sec = {}'.format(time.minute, time.second))
            fig.colorbar(im1, ax=ax1)
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax1.scatter(robot_0_x, robot_0_y, c='b')
                ax1.scatter(robot_1_x, robot_1_y, c='r')
            ax1.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))
    

            im2 = ax2.pcolormesh(expected_array)
            ax2.set_title('Expected Distribution')
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax2.scatter(robot_0_x, robot_0_y, c='b')
                ax2.scatter(robot_1_x, robot_1_y, c='r')
            fig.colorbar(im2, ax=ax2)
            ax2.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))

            plt.savefig('plots/opinion_map_' + self.filename + '_op_{}.png'.format(self.fig_counter), bbox_inches='tight')
            plt.close(fig)
            #plt.show()
            self.fig_counter += 1
            

    def plot_belief_maps(self):
        """
        This function plots the individual opinions published in the observations data and their
        aggregation.
        """
      
        # alternatively use robot positioning as timing
        if False:
            #  use robots as time
            time_list = sorted(self.robot_pos['robot_0'].keys())
        else:
            # use opinion map as time
            time_list = sorted(self.observations_map.keys())
            #print(time_list)
            #time_list = [tmp_t for tmp_t in time_list if tmp_t > datetime(year = 2020, month = 9, day = 8, hour = 16, minute=3, second = 19) and tmp_t < datetime(year = 2020, month = 9, day = 8, hour = 16, minute=7, second = 9)]
            #print(time_list)
 
        for time in time_list:
            task_distr_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            
            # determine the times closest to 'time' in both arrays.
            # find closest time that is older i.e. smaller thatn time. (so dont see future)
            
            arr_td = np.array([(i - time).total_seconds() for i in self.task_distr.keys()])
            if np.all(arr_td >= 0):
                # take closest one
                closest_time_td = min(self.task_distr.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_td_index =np.where(arr_td== -1*min(np.absolute(arr_td[arr_td<0])))[0][0]
                closest_time_td = self.task_distr.keys()[closest_time_td_index]
            
            arr_op = np.array([(i - time).total_seconds() for i in self.opinion_map.keys()])
            if np.all(arr_op >= 0):
                # take closest one
                closest_time_op = min(self.opinion_map.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_op_index = np.where(arr_op==-1*min(np.absolute(arr_op[arr_op<0])))[0][0]
                closest_time_op = self.opinion_map.keys()[closest_time_op_index]
            


            arr_obs = np.array([(i - time).total_seconds() for i in self.observations_map.keys()])
            if np.all(arr_obs >= 0):
                # take closest one
                closest_time_obs = min(self.observations_map.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_obs_index = np.where(arr_obs==-1*min(np.absolute(arr_obs[arr_obs<0])))[0][0]
                closest_time_obs = self.observations_map.keys()[closest_time_obs_index]



            # convert data to 2D array. The values are assigned as follows:
            # -1: unoccupied space
            #  0: false positive of robot 0
            #  1: fasle positive of robobt 1
            #  2: false positive of robot 0 and 1
            #  3: ground truth
            for pos_index in self.task_distr[closest_time_td].keys():
                fp_list = self.task_distr[closest_time_td][pos_index]
                if len(fp_list) == 0:
                    task_distr_array[0,pos_index] = 3
                elif len(fp_list) == 1:
                    task_distr_array[0, pos_index] = fp_list[0]
                elif len(fp_list) == 2:
                    task_distr_array[0, pos_index] = 2

            # believe array stores the belive that it is occupied
            # whilst uncertainty array stores the uncertainty
            believe_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            uncertainty_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            expected_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            base_rate_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            for pos_index in self.opinion_map[closest_time_op].keys():
                tmp_op = self.opinion_map[closest_time_op][pos_index]
                believe_array[0, pos_index] = tmp_op.belief
                uncertainty_array[0, pos_index] = tmp_op.uncertainty 
                base_rate_array[0, pos_index] = tmp_op.base_rate
                expected_array[0, pos_index] = tmp_op.belief + tmp_op.base_rate * tmp_op.uncertainty
                
            
            net_belief_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            for pos_index in self.observations_map[closest_time_obs].keys():
                net_belief_array[0,pos_index] = self.observations_map[closest_time_obs][pos_index]
            
            
            #print(max(expected_array[0,:]))
            # add walls to the task_distr for reference
            wall_mask = [believe_array[0,:] == -1]
            task_distr_array[0,wall_mask[0]] = -2
               
            # reshape arrays into actual dimensions
            task_distr_array = np.reshape(task_distr_array, [self.grid_map_height, self.grid_map_width])
            believe_array = np.reshape(believe_array, [self.grid_map_height, self.grid_map_width])
            uncertainty_array = np.reshape(uncertainty_array, [self.grid_map_height, self.grid_map_width])
            expected_array = np.reshape(expected_array, [self.grid_map_height, self.grid_map_width])
            base_rate_array = np.reshape(base_rate_array, [self.grid_map_height, self.grid_map_width])
           

            net_belief_array = np.reshape(net_belief_array, [self.grid_map_height, self.grid_map_width])

            # Determine the robot positions at this time
            closest_time_r_0 = min(self.robot_pos['robot_0'], key=lambda x: np.absolute(x-time))
            closest_time_r_1 = min(self.robot_pos['robot_1'], key=lambda x: np.absolute(x-time))
            robot_0_x, robot_0_y = self.robot_pos['robot_0'][closest_time_r_0]
            robot_1_x, robot_1_y = self.robot_pos['robot_1'][closest_time_r_1]
            
            # convert the robot pos into the correct indexes
            robot_0_x = (robot_0_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_0_y = (robot_0_y - self.grid_map_origin_y) / self.grid_map_resolution
            robot_1_x = (robot_1_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_1_y = (robot_1_y - self.grid_map_origin_y) / self.grid_map_resolution
            
            # plot the arrays side by side
            # Note, pcolormesh automatically flips vertical axis when plotting. i.e. array[0,0] is
            # plotted in the lower left hand corner but top left corner when print(array).
            # Also first transpose the array (i.e. mirror along diagonal) so that combination of 
            # mirror along diagonal and vertical flip alligns the x axis to the right and y axis to
            # the top. This is needed so that robots are in correct position in map when plotted.
            fig, (ax0, ax1, ax2, ax3) = plt.subplots(1,4)            

            im0 = ax0.pcolormesh(task_distr_array)
            ax0.set_title('Task Distribution')
            ax0.set_xlabel('min={}, sec = {}'.format(closest_time_td.minute, closest_time_td.second))
            fig.colorbar(im0, ax=ax0)
            
            im1 =  ax1.pcolormesh(net_belief_array)
            ax1.set_title('Net Belief Distr, robot time = min={}, sec = {}'.format(time.minute, time.second))
            fig.colorbar(im1, ax=ax1)
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax1.scatter(robot_0_x, robot_0_y, c='b')
                ax1.scatter(robot_1_x, robot_1_y, c='r')
            ax1.set_xlabel('min={}, sec = {}'.format(closest_time_obs.minute, closest_time_obs.second))

            im2 =  ax2.pcolormesh(believe_array)
            ax2.set_title('Believe Distributon, robot time = min={}, sec = {}'.format(time.minute, time.second))
            fig.colorbar(im2, ax=ax2)
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax2.scatter(robot_0_x, robot_0_y, c='b')
                ax2.scatter(robot_1_x, robot_1_y, c='r')
            ax2.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))
    

            im3 = ax3.pcolormesh(expected_array)
            ax3.set_title('Expected Distribution')
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax3.scatter(robot_0_x, robot_0_y, c='b')
                ax3.scatter(robot_1_x, robot_1_y, c='r')
            fig.colorbar(im3, ax=ax3)
            ax3.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))

            plt.savefig('plots/opinion_map_' + self.filename + '_op_{}.png'.format(self.fig_counter), bbox_inches='tight')
            plt.close(fig)
            #plt.show()
            self.fig_counter += 1

if __name__ == '__main__':
    bag_path = "./data"
    
    # Flag, if want to anylise a single test and make detailed plots or compare multiple tests
    single_analysis = False

    if single_analysis == False:
        # either give specific filenames here
        #filename_list = ["r2_sl1_fp1_fn1_spi80_comp"] # without .bag ending
        
        # or all in /data directory
        filename_list= []
        dirListing = os.listdir('./data')
        for fname in dirListing:
            # remove .bag ending
            filename_list.append(fname[:fname.rindex('.bag')])
        
        # store dictinary of format dict[interval][use_sl] = [avg_time, stdev] in dict list and 
        # the seed with which the test was done in the seed list.
        seed_list = [] 
        dict_list = []
        counter_dict_list = []
        history_dict_list = []

        for filename in filename_list:
            # get test info from name
            seperated_str= re.split('_+', filename)
            use_sl_str = re.split('sl', seperated_str[1])[1]
            spawn_interval = int(re.findall('\d+', seperated_str[4])[0])# _spi# is 5th entry of name 

            seed = int(re.findall('\d+', seperated_str[5])[0]) # _seed# is 6th entry of name 
            fpp0_num = float(re.findall('\d+\.\d+', seperated_str[6])[0])
            fpp1_num = float(re.findall('\d+\.\d+', seperated_str[8])[0])

            # only select the ones with specific fp uncertainties:
            if fpp0_num == 0.4 and fpp1_num == 0.4 and seperated_str[-1] != 'type0.99':
                
                # extract information
                data_analysis = DataAnalysis(bag_path, filename)
                data_analysis.extract_active_tasks()
                #data_analysis.extract_opinion_map()
                #data_analysis.extract_observation_info()
                #data_analysis.extract_robot_pos()
                
                # saving data in pickle file for offline CBF calculations
                #data_analysis.save_data()
                
                # analysis
                print('File tested: SL = ' + use_sl_str + ' seed = {} interval = {}'.format(seed, spawn_interval))
                duration_results = data_analysis.ground_truth_completion_time()
                counter_results = data_analysis.task_type_counter()
                history_results = data_analysis.completion_time_history(False)

                # store duration_results: first check if already dictionary with seed 
                # if exists then add to that.
                if seed in seed_list:
                    index = seed_list.index(seed)
                    
                    existing_dict = dict_list[index]
                    existing_counter_dict = counter_dict_list[index]
                    existing_history_dict = history_dict_list[index]   
                    
                    # check if keys exist
                    if spawn_interval in existing_dict.keys() and use_sl_str in existing_dict[spawn_interval].keys():
                        print('Keys already exist, should not happen, replace previous value with new one')

                    # replace previous value -- maybe should take average instead...
                    existing_dict[spawn_interval][use_sl_str] = duration_results
                    dict_list[index] = existing_dict

                    existing_counter_dict[spawn_interval][use_sl_str] = counter_results
                    counter_dict_list[index] = existing_counter_dict
                    
                    existing_history_dict[spawn_interval][use_sl_str] = history_results
                    history_dict_list[index] = existing_history_dict

                else:
                    seed_list.append(seed)    
                    
                    seed_dict = defaultdict(dict)
                    seed_dict[spawn_interval][use_sl_str] = duration_results
                    dict_list.append(seed_dict)

                    counter_dict = defaultdict(dict)
                    counter_dict[spawn_interval][use_sl_str] = counter_results
                    counter_dict_list.append(counter_dict)

                    history_dict = defaultdict(dict)
                    history_dict[spawn_interval][use_sl_str] = history_results
                    history_dict_list.append(history_dict)
                # duration_results = [avg time, stdev]

                # Printing tasks who are a false positive for both robots
                #for key in data_analysis.task_data.keys():
                #    if len(data_analysis.task_data[key]['false_positive']) == 2:
                #        print(data_analysis.data[key])
                
                
                # Detailed Plotting: belief maps plots aggregation of individual opinions (debugging)
                #                    opinons maps plots belief and expected distribution (prefered)
                #data_analysis.plot_belief_maps()
                #data_analysis.plot_opinion_maps()

        # plotting avg completion times and their stdev. against spi
        fig_counter = 1
        for i in range(len(seed_list)):
            # Plotting durations
            fig = plt.figure(fig_counter)
            seed = seed_list[i]
            seed_dict = dict_list[i]
            intervals = sorted(seed_dict.keys()) # from smallest fo largest
            print([seed_dict[j]['true'][0]  - seed_dict[j]['false'][0]for j in intervals])
            plt.scatter(intervals, [seed_dict[j]['true'][0] for j in intervals],label='avg time with KA')
            plt.scatter(intervals, [seed_dict[j]['false'][0] for j in intervals],label='avg time no KA')
            plt.scatter(intervals, [seed_dict[j]['true'][1] for j in intervals], label='stdev for KA')
            plt.scatter(intervals, [seed_dict[j]['false'][1] for j in intervals], label='stdev no KA')
            plt.legend()

            plt.savefig('plots/cleaning_times_seed{}_45min.png'.format(seed), bbox_inches='tight')
            fig_counter += 1

            # Plotting counters
            fig = plt.figure(fig_counter)
            counter_dict = counter_dict_list[i]
            print(counter_dict)
            plt.scatter(intervals, [counter_dict[j]['true'][0] for j in intervals],label='gt cleaned wth KA')
            plt.scatter(intervals, [counter_dict[j]['false'][0] for j in intervals],label='gt cleaned no KA')
            plt.scatter(intervals, [counter_dict[j]['true'][2] for j in intervals], label='fp cleaned with KA')
            plt.scatter(intervals, [counter_dict[j]['false'][2] for j in intervals], label='fp cleaned no KA')
            plt.legend()

            plt.savefig('plots/cleaning_numbers_seed{}_45min.png'.format(seed), bbox_inches='tight')
            fig_counter += 1

        # plotting the completion times
        for i in range(len(seed_list)):
            seed = seed_list[i]
            seed_dict = history_dict_list[i]
            counter_d = counter_dict_list[i]
            intervals = sorted(seed_dict.keys()) # from smallest fo largest
            for it in intervals:
                # GT time history
                fig = plt.figure(fig_counter)
                plt.plot(seed_dict[it]['true'][0], [j / float(counter_d[it]['true'][1]) for j in range(1, 1 + len(seed_dict[it]['true'][0]))], label='gt cleaned with KA')
                plt.plot(seed_dict[it]['false'][0], [j / float(counter_d[it]['false'][1]) for j in range(1, 1 + len(seed_dict[it]['false'][0]))] , label='gt cleaned no KA')
                plt.legend()
                plt.savefig('plots/gt_history_seed{}_interval{}_45min.png'.format(seed, it), bbox_inches='tight')

                fig_counter += 1
                
                # plotting fp time history
                fig = plt.figure(fig_counter)
                plt.plot(seed_dict[it]['true'][1], [j / float(counter_d[it]['true'][3]) for j in range(1, 1 + len(seed_dict[it]['true'][1]))], label='FP cleaned with KA')
                plt.plot(seed_dict[it]['false'][1],[j/ float(counter_d[it]['false'][3]) for j in range(1, 1 + len(seed_dict[it]['false'][1]))], label='FP cleaned no KA')
                plt.legend()
                plt.savefig('plots/fp_history_seed{}_interval{}_45min.png'.format(seed, it), bbox_inches='tight')

                fig_counter += 1
            

    else:
        # single analysis

        # note filename without .bag ending
        filename = "r2_sltrue_fptrue_fntrue_spi30_seed131_r0fpp0.4_r0fnp0.2_r1fpp0.4_r1fnp0.2_type0.99"

        # extract data
        data_analysis = DataAnalysis(bag_path, filename)
        data_analysis.extract_active_tasks()
        data_analysis.extract_opinion_map()
        #data_analysis.extract_observation_info()
        data_analysis.extract_robot_pos()
        
        
        # Plotting the aggregated opinoin maps
        data_analysis.plot_opinion_maps()
        
        # Plotting time history of completing ground truths
        #data_analysis.completion_time_history(True)
        
        # Plotting the individual opinions that creat aggregated opinion map (many plots!)
        #data_analysis.plot_opinion_maps()


