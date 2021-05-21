#!/usr/bin/env python

import rosbag
import sys
import os
import pickle
import matplotlib.pyplot as plt
from matplotlib import rcParams
from datetime import datetime, timedelta
import glob
import seaborn as sns
import pandas as pd
import numpy as np
import distutils.dir_util
import csv
from collections import defaultdict


def make_task_key(x, y):
    """
    Creates a key from the coordinates of a given task.

    The key is used to identify the tasks within a dictionary

    :param x: x coordinate
    :param y: y coordinate
    :return: the created key
    """
    key = (x, y)
    return key

def from_task_key(key):
    """
    Retrieves the coordinates of a tasks given its corresponding key

    :param key: the key to use
    :return: x and y coordinates of the task
    """
    return key[0], key[1]


class BagAnalysis:
    """
    BagAnalysis is a helper class to extract data from rosbag files and store such data in csv files
    """
    def __init__(self, bag_path, bag_file):
        self.data = defaultdict(dict)
        self.csv_path = os.path.join(bag_path, bag_file[:-4] + ".csv")

        self.freq_plot_file = os.path.join(bag_path, "freq_tasks_" + bag_file[-23:-4] + ".png")
        self.rolling_plot_file = os.path.join(bag_path, "rolling_tasks_" + bag_file[-23:-4] + ".png")

        self.all_freq_plot_file = "freq_tasks" + ".png"
        self.all_rolling_plot_file = "rolling_tasks" + ".png"

        self.ma_plot_file = os.path.join(bag_path, "cum_tasks_" + bag_file[-23:-4] + ".png")
        self.bag = rosbag.Bag(os.path.join(bag_path, bag_file))
        self.keys = []

        self.spawned_count = []
        self.spawned_time_count = []

        self.detected_count = []
        self.detected_time_count = []

        self.attained_count = []
        self.attained_time_count = []

        # Create csv file and add header values
        with open(self.csv_path, 'w') as csv_file:
            fieldnames = ['x', 'y', 'spawned', 'detected', 'attained', 'discovery', 'attainment',
                          'discover_completion']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

    def extract_spawned_tasks(self, topic):
        """
        Extracts the data recorded about spawned tasks
        :param topic: ROS topic to which the data was publish
        :return: None
        """
        spawned = 0
        for topic, msg, t in self.bag.read_messages(topics=topic):
            # print(msg)
            spawned_dirt = make_task_key(msg.pose.position.x, msg.pose.position.y)
            spawned = spawned + 1
            self.keys.append(spawned_dirt)
            self.spawned_count.append(spawned)
            self.data[spawned_dirt]["spawned"] = datetime.fromtimestamp(float(t.secs))
            self.spawned_time_count.append(datetime.fromtimestamp(float(t.secs)))

    def extract_detected_tasks(self, topic):
        """
        Extracts the data recorded about detected tasks
        :param topic: ROS topic to which the data was publish
        :return: None
        """
        detected = 0
        for topic, msg, t in self.bag.read_messages(topics=topic):
            # print(msg)
            if not msg.Bid.task.is_virtual:
                detected_dirt = make_task_key(msg.Bid.task.x, msg.Bid.task.y)
                detected = detected + 1
                self.detected_count.append(detected)
                self.data[detected_dirt]["detected"] = datetime.fromtimestamp(float(t.secs))
                self.detected_time_count.append(datetime.fromtimestamp(float(t.secs)))

    def extract_attained_tasks(self, topic):
        """
        Extracts the data recorded about attained tasks
        :param topic: ROS topic to which the data was publish
        :return: None
        """
        attained = 0
        for topic, msg, t in self.bag.read_messages(topics=topic):
            # print(msg)
            if not msg.is_virtual:
                attained_dirt = make_task_key(msg.x, msg.y)
                attained = attained + 1
                self.attained_count.append(attained)
                self.data[attained_dirt]["attained"] = datetime.fromtimestamp(float(t.secs))
                self.attained_time_count.append(datetime.fromtimestamp(float(t.secs)))

    def write_task_data(self):
        """
        Writes the extracted rosbag data into the corresponding csv file
        :return: None
        """
        start_date = datetime(1970, 1, 1, 1, 00, 00)

        with open(self.csv_path, 'a') as csv_file:
            writer = csv.writer(csv_file)
            for key in self.keys:
                diff = self.data[key]["spawned"] - start_date
                if (diff.total_seconds() % 3600) // 60 <= 10:

                    time_to_discover = None
                    if self.data[key].get("detected", None):
                        time_to_discover = (self.data[key]["detected"] - self.data[key]["spawned"]).total_seconds()

                    time_to_completion = None
                    if self.data[key].get("attained", None):
                        time_to_completion = (self.data[key]["attained"] - self.data[key]["detected"]).total_seconds()

                    total_time = None
                    if self.data[key].get("attained", None) and self.data[key].get("detected", None):
                        # total_time = self.data[key]["attained"] - self.data[key]["spawned"]
                        total_time = time_to_discover + time_to_completion

                    entry = [from_task_key(key)[0], from_task_key(key)[1], self.data[key]["spawned"],
                             self.data[key].get("detected", None),
                             self.data[key].get("attained", None),
                             time_to_discover,
                             time_to_completion,
                             total_time]

                    writer.writerow(entry)

    def plot_all(self):
        """
        Plots detected and attained trends based on the frequency of tasks
        :return:
        """
        sns.set()
        fig2, ax2 = plt.subplots(figsize=(14, 10))
        # spawned = ax2.plot(self.spawned_time_count, self.spawned_count, linewidth=2.5,
        #                    label='spawned')

        detected = ax2.plot(self.detected_time_count, self.detected_count, linewidth=2.5,
                            label='detected')

        attained = ax2.plot(self.attained_time_count, self.attained_count, linewidth=2.5,
                            label='attained')

        ax2.legend()
        plt.xlabel("Simulation time (min)", fontsize=14)
        plt.ylabel("Number of tasks", fontsize=14)
        ax2.tick_params('y', labelsize=14)
        ax2.tick_params('x', labelsize=14)
        plt.tight_layout()
        plt.savefig(self.all_plot_file, bbox_inches='tight', dpi=300)

    def plot_all2(self):
        """
        Plots spawnwd, detected and attained trends based on the frequency of tasks using a rolling window
        :return:
        """
        df = pd.read_csv(self.csv_path)

        df["spawned"] = df["spawned"].astype("datetime64")
        df["detected"] = df["detected"].astype("datetime64")
        df["attained"] = df["attained"].astype("datetime64")

        df1 = df[['detected', 'attained']]
        df1.groupby(df1["detected"].dt.minute).count().plot(kind="line", use_index=False)
        plt.xlabel("Simulation time (min)", fontsize=14)
        plt.ylabel("# tasks", fontsize=14)
        plt.savefig(self.freq_plot_file, bbox_inches='tight', dpi=300)

        # Frequency using rolling window
        df3 = pd.DataFrame({'detected': list(df1.groupby(df1["detected"].dt.minute).count()["detected"])},
                           index=list(df1.groupby(df1["detected"].dt.minute)["detected"]))

        df4 = pd.DataFrame({'attained': list(df1.groupby(df1["attained"].dt.minute).count()["attained"])},
                           index=list(df1.groupby(df1["attained"].dt.minute)["attained"]))

        ax = df3.rolling(1).sum().plot(use_index=False)
        df4.rolling(1).sum().plot(ax=ax, use_index=False)
        ax.legend()
        plt.xlabel("Simulation time(min)", fontsize=14)
        plt.ylabel("# tasks", fontsize=14)
        ax.tick_params('y', labelsize=14)
        ax.tick_params('x', labelsize=14)
        plt.tight_layout()
        plt.savefig(self.rolling_plot_file, bbox_inches='tight', dpi=300)

    def moving_avg(self, elements):
        """
        Computes the moving average of the given elements
        :param elements: elements to work with
        :return: the calculated moving average
        """
        cumsum, ma = [0], []
        n = 3

        for i, x in enumerate(elements, 1):
            cumsum.append(cumsum[i - 1] + x)
            if i >= n:
                m_avg = (cumsum[i] - cumsum[i - n]) / n
                ma.append(m_avg)
        return ma

    def plot_moving_average(self):
        """
        Plots spawned, detected and attained trend using a moving average
        :return:
        """
        ma_spawned = self.moving_avg(self.spawned_count)
        ma_detected = self.moving_avg(self.detected_count)
        ma_attained = self.moving_avg(self.attained_count)

        sns.set()
        fig2, ax2 = plt.subplots(figsize=(14, 10))
        spawned = ax2.plot(self.spawned_time_count[1:-1], ma_spawned, linewidth=2.5,
                           label='spawned')

        detected = ax2.plot(self.detected_time_count[1:-1], ma_detected, linewidth=2.5,
                            label='detected')

        attained = ax2.plot(self.attained_time_count[1:-1], ma_attained, linewidth=2.5,
                            label='attained')

        ax2.legend()
        plt.xlabel("Simulation time (min)", fontsize=14)
        plt.ylabel("# tasks", fontsize=14)
        ax2.tick_params('y', labelsize=14)
        ax2.tick_params('x', labelsize=14)
        plt.tight_layout()
        plt.savefig(self.ma_plot_file, bbox_inches='tight', dpi=300)

    def combine_data(self, bag_path):
        """
        Consolidates all the csv data from each run, into a single csv file
        :param bag_path: path to the diretory where the csv files are located
        :return: None
        """
        os.chdir(bag_path)
        extension = 'csv'
        all_filenames = [i for i in glob.glob('*.{}'.format(extension))]
        # combine all files in the list
        combined_csv = pd.concat([pd.read_csv(f) for f in all_filenames])
        # export to csv
        combined_csv.to_csv("combined_data.csv", index=False, encoding='utf-8-sig')

        df = pd.read_csv('combined_data.csv')

        df["spawned"] = df["spawned"].astype("datetime64")
        df["detected"] = df["detected"].astype("datetime64")
        df["attained"] = df["attained"].astype("datetime64")

        df1 = df[['detected', 'attained']]
        df1.groupby(df1["detected"].dt.minute).count().plot(kind="line", use_index=False)
        plt.xlabel("Simulation time (min)", fontsize=14)
        plt.ylabel("# of tasks", fontsize=14)
        plt.savefig(self.all_freq_plot_file, bbox_inches='tight', dpi=300)

        # Plot detected and attained trends based on the frequency of tasks using a rolling window
        df3 = pd.DataFrame({'detected': list(df1.groupby(df1["detected"].dt.minute).count()["detected"])},
                           index=list(df1.groupby(df1["detected"].dt.minute)["detected"]))

        df4 = pd.DataFrame({'attained': list(df1.groupby(df1["attained"].dt.minute).count()["attained"])},
                           index=list(df1.groupby(df1["attained"].dt.minute)["attained"]))

        ax = df3.rolling(1).sum().plot(use_index=False)
        df4.rolling(1).sum().plot(ax=ax, use_index=False)
        ax.legend()
        plt.xlabel("Simulation time(min)", fontsize=14)
        plt.ylabel("# tasks", fontsize=14)
        ax.tick_params('y', labelsize=14)
        ax.tick_params('x', labelsize=14)
        plt.tight_layout()
        plt.savefig(self.all_rolling_plot_file, bbox_inches='tight', dpi=300)


if __name__ == '__main__':
    # Get path where the rosbag data is stored. This will be used as the working directory
    bag_path_ = sys.argv[1]
    cwd = os.getcwd()

    # Create csv files for each rosbag file found in the given directory
    for filename in os.listdir(bag_path_):
        os.chdir(cwd)
        if filename.endswith(".bag"):
            print(filename)
            analysis = BagAnalysis(bag_path_, filename)
            analysis.extract_spawned_tasks("/new_dirt")
            analysis.extract_detected_tasks("/confirmation")
            analysis.extract_attained_tasks("/goal_attained")
            analysis.write_task_data()

            # Enable to plot frequency of tasks for each run (rosbag file)
            # analysis.plot_all()
            # analysis.plot_all2()
            # analysis.plot_moving_average()
        else:
            continue

    # Combine all csv files into a single file, and plot the task trends of the consolidated data
    analysis.combine_data(bag_path_)
