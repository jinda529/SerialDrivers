# encoding: utf-8
#!/usr/bin/env python

import argparse
import copy
import csv
from dbw_mkz_msgs.msg import *
import logging
import math
import matplotlib
import matplotlib.pyplot as plt
import munch
import numpy as np
import os
import re
import rosbag
from scipy import interpolate
from scipy.optimize import leastsq
from std_msgs.msg import String
import time
import traceback
import sys
from control import dbw_reports_pb2
from control import control_command_pb2
from planning import planning_trajectory_pb2
from prediction import prediction_obstacle_pb2
from localization import localization_pb2


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

linear_regression_k = -0.231081
linear_regression_b = 29.404476


class KMeansClusteringForFloatTwoGroups:
    def __init__(self):
        self.point_1 = None
        self.point_2 = None
        self.data_points = []
        self.data_points_belong = []
        self.setup_success = False
        self.clustering_done = False
        self.max_step = 100000
        self.counter = 0

    def setup_data_points(self, data_point_list):
        for data_point in data_point_list:
            self.data_points.append(data_point)
            self.data_points_belong.append(1)
        if len(self.data_points) >= 2:
            self.setup_success = True
            self.point_1 = min(self.data_points)
            self.point_2 = max(self.data_points)
    
    def do_step(self):
        self.counter += 1
        old_data_points_belong = [belong for belong in self.data_points_belong]
        for index, value in enumerate(self.data_points):
            if abs(value - self.point_1) <= abs(value - self.point_2):
                self.data_points_belong[index] = 1
            else:
                self.data_points_belong[index] = 2
        self.point_1 = np.mean([self.data_points[index] for index in range(len(self.data_points)) if self.data_points_belong[index] == 1])
        self.point_2 = np.mean([self.data_points[index] for index in range(len(self.data_points)) if self.data_points_belong[index] == 2])
        if any([old_belong != belong for old_belong, belong in zip(old_data_points_belong, self.data_points_belong)]):
            return False
        return True

    def do_clustering(self):
        if not self.setup_success:
            return (np.nan, np.nan)
        while self.counter < self.max_step:
            self.clustering_done = self.do_step()
            if self.clustering_done:
                break
        return (self.point_1, self.point_2)
        



class LinearRegression:
    #   Theta^T * X = y
    # Where: X is N*1 matrix,
    #        Theta is N*1 matrix,
    #        y is 1*1 number.
    # For linear regression:
    #   Theta = sum(X * X^T)^-1 * sum(X*y)
    def __init__(self, N):
        self.reset(N)

    def reset(self, N):
        self.N = N
        self.sum_of_X_XT = np.zeros((N, N))
        self.sum_of_X_y = np.zeros((N, 1))
        self.sum_of_y2 = 0.0
        self.result = None
        self.cost = None
        self.input_counter = 0

    def add_a_input(self, X_input, y_input):
        # Check if input is valid and has correct length
        if len(X_input) != self.N:
            print("invalid input for LinearRegression! " +
                  "Input X has incorrect length: {}, ".format(len(X_input)) +
                  "which should be: {}!".format(self.N))
            return
        try:
            y = float(y_input)
        except ValueError as e: 
            print("invalid input for LinearRegression: {}".format(e))
            return
        X = np.zeros((self.N, 1))
        # Update collection variables
        for index, value in enumerate(X_input):
            X[index] = value
        self.sum_of_X_XT = self.sum_of_X_XT + X.dot(X.T)
        self.sum_of_X_y = self.sum_of_X_y + X.dot(y)
        self.sum_of_y2 = self.sum_of_y2 + y * y
        self.input_counter += 1
        self.result = None
        self.cost = None

    def add_input(self, X_list, y_list):
        if len(X_list) != len(y_list) or len(y_list) < 1:
            print("invalid input for LinearRegression! " +
                  "Input X and y list have different length.")
            return
        for X, y in zip(X_list, y_list):
            self.add_a_input(X, y)

    def execute(self):
        rank = np.linalg.matrix_rank(self.sum_of_X_XT)
        if rank != self.N:
            print("data input not yet full rank! " +
                  "Current rank is: {}".format(rank))
            return
        self.result = np.linalg.inv(self.sum_of_X_XT).dot(self.sum_of_X_y)
        self.cost = np.power(((self.result.T.dot(
            self.sum_of_X_XT.dot(self.result)) -
            self.result.T.dot(self.sum_of_X_y) +
            self.sum_of_y2)[0][0] / self.input_counter), 0.5)

def linear_regressor_filter(input_list, Num4Filter=0, Order=0):
    output_list = copy.deepcopy(input_list)
    for index in range(len(input_list)):
        if index < Num4Filter:
            output_list[index] = np.nan
            continue
        if index > len(input_list)-Num4Filter-1:
            output_list[index] = np.nan
            continue
        snip = input_list[index - Num4Filter: index + Num4Filter + 1]
        if any(np.isnan(snip)):
            output_list[index] = np.nan
            continue
        regressor = LinearRegression(Order+1)
        for snip_index in range(len(snip)):
            x_input = [1.0]
            for i in range(Order):
                x_input.append(x_input[-1]*snip_index)
            regressor.add_a_input(x_input, snip[snip_index])
        regressor.execute()
        result = 0.0
        for i in range(Order+1):
            result += regressor.result[i] * np.power(Num4Filter, i)
        output_list[index] = result
    return output_list


def sine_curve_fit(data_tuple, preset_w=None):
    # Fit a curve in the form of:
    #     y = amp * sin(freq * t + phase) + mean
    # Step 1: Fast Fourier Tranform to identify most possible freq.
    # Step 2: do optimization to find these params

    # Step 1:
    timestamp_list = list(data_tuple[0])
    data_list = list(data_tuple[1])
    if len(timestamp_list) != len(data_list):
        print("Invalid input! Data length has to be the same!")
        return None
    if preset_w is None:
        # Number of sample points
        N = len(timestamp_list)
        # sample spacing
        T = timestamp_list[1] - timestamp_list[0]
        yf = 2.0 / N * np.abs(np.fft.rfft(data_list)[0:N //2])
        xf = np.fft.rfftfreq(N, T)[:N //2]
        guess_freq = 2.0 * math.pi * xf[np.where(yf == max(yf))[0][0]]
    else:
        guess_freq = preset_w
    guess_mean = (max(data_list) + min(data_list)) / 2.0
    guess_amp = (max(data_list) - min(data_list)) / 2.0
    guess_phase = math.pi / 2.0 - \
        guess_freq * timestamp_list[data_list.index(max(data_list))]
    guess_phase = guess_phase % (2.0 * np.pi)

    # Step 2:
    t = np.array(timestamp_list)
    data = np.array(data_list)
    # Define the function to optimize
    def optimize_func(x):
        return x[0] * np.sin(x[1] * t + x[2]) + x[3] - data
    est_amp, est_freq, est_phase, est_mean = leastsq(
        optimize_func, [guess_amp, guess_freq, guess_phase, guess_mean])[0]
    return (est_amp, est_freq, est_phase, est_mean)


def data_interpolation(x_original, y_original, x_new):
    '''
    Interpolation function for data, where x_original and x_new need to be "[topic].times".
    '''
    dist_for_interp = 0.1
    if len(list(set([len(x_original), len(y_original)]))) != 1:
        sys.exit("ERROR: First Two Of data_interpolation() Input Have To Have Same Length!")
    x_original = np.insert(x_original, 0, min(min(x_new), x_original[0]) - dist_for_interp)
    x_original = np.append(x_original, max(max(x_new), x_original[-1]) + dist_for_interp)
    y_original = np.insert(y_original, 0, y_original[0] + (y_original[1] - y_original[0]) / (x_original[1] - x_original[0]) * (min(min(x_new) - x_original[0], 0.0) - dist_for_interp))
    y_original = np.append(y_original, y_original[-1] + (y_original[-1] - y_original[-2]) / (x_original[-1] - x_original[-2]) * (max(max(x_new) - x_original[-1], 0.0) + dist_for_interp))
    data_interpolate = interpolate.interp1d(x_original, y_original)
    if type(x_new) == list:
        x_new = np.array(x_new)
    return data_interpolate(x_new)


class BagFinder:
    def __init__(self):
        self.baglist = []

    def append_filename_to_baglist(self, path, file_name):
        if (file_name.split('.'))[-1] == 'bag' or (file_name.split('.'))[-1] == 'db':
            self.baglist.append(os.path.abspath(os.path.join(path,file_name)))

    def find_bags_in_(self, dir_name_list):
        for dir_name in dir_name_list:
            if os.path.isfile(dir_name):
                self.append_filename_to_baglist('', os.path.abspath(dir_name))
            elif os.path.isdir(dir_name):
                for path, dirs, files in os.walk(dir_name):
                    for file_name in files:
                        self.append_filename_to_baglist(path,file_name)
        self.baglist = list(set(self.baglist))
        return self.baglist


class BagReader():
    def __init__(self, file_name):
        self.file_name = file_name
        self.last_time_sec = None
        self.start_time_sec = None
        self.dbw_reports_data = []
        self.miran_sensor_data = []
        self.range_set = args.tmin is not None and args.tmax is not None
    def read_bag(self):
        # Load data
        print('   ')
        logger.info('Opening bag file: {}'.format(self.file_name))
        try:
            bag = rosbag.Bag(self.file_name)
        except UnicodeDecodeError as e:
            logger.error('Invalid bag file: {}, {}'.format(self.file_name, e))
            return False
        except IOError as e:
            logger.error('Unable to read bag: {}, {}'.format(self.file_name, e))
            return False
        bag_start_time = bag.get_start_time()
        bag_end_time = bag.get_end_time()
        logger.info('Reading {} messages (duration: {:.1f} sec)...'.format(
            bag.get_message_count(), bag_end_time - bag_start_time))
        logger.info('Starting time {:.0f}'.format(bag_start_time))
        logger.info('Ending time {:.0f}'.format(bag_end_time))

        # Record data
        for topic, msg, t in bag.read_messages():
            if topic == '/vehicle/dbw_reports':
                self.last_time_sec = t.to_sec()
                if self.start_time_sec is None:
                    self.start_time_sec = t.to_sec()
                pb = dbw_reports_pb2.DbwReports()
                pb.ParseFromString(msg.data)
                try:
                    if (self.range_set
                        and (args.tmin > t.to_sec() - self.start_time_sec 
                             or t.to_sec() - self.start_time_sec > args.tmax)):
                        continue
                    self.dbw_reports_data.append(munch.Munch(
                        bag_time=t.to_sec() - self.start_time_sec,
                        msg_time=pb.header.timestamp_msec / 1000.0 - self.start_time_sec,
                        real_steering_angle=(pb.steering_report.steering_wheel_angle
                                             + pb.steering_report.steering_wheel_angle_compensation),
                        speed=pb.steering_report.speed,
                    ))
                except AttributeError:
                    pass
            elif topic == '/MiranSensorInfo':
                self.last_time_sec = t.to_sec()
                if self.start_time_sec is None:
                    self.start_time_sec = t.to_sec()
                try:
                    if (self.range_set
                        and (args.tmin > t.to_sec() - self.start_time_sec 
                             or t.to_sec() - self.start_time_sec > args.tmax)):
                        continue
                    self.miran_sensor_data.append(munch.Munch(
                        bag_time=t.to_sec() - self.start_time_sec,
                        msg_time=msg.timestamp - self.start_time_sec,
                        length=msg.length
                    ))
                except AttributeError:
                    pass
        if not self.dbw_reports_data or not self.miran_sensor_data:
            logger.error('Not enough valid data in bag: {}'.format(self.file_name))
            return False
        return True
    
    def plot_data(self):
        steering_wheel_angle = [msg.real_steering_angle * 57.3 for msg in self.dbw_reports_data]
        dbw_timestamps = [msg.bag_time for msg in self.dbw_reports_data]
        miran_timestamps = [msg.bag_time for msg in self.miran_sensor_data]
        # derive equivalent front wheel angle
        front_wheel_angle = [1.22 * (msg.length * linear_regression_k + linear_regression_b) for msg in self.miran_sensor_data]
        front_wheel_angle_filtered = linear_regressor_filter(front_wheel_angle, Num4Filter=7, Order=3)
        front_wheel_angle_filtered_in_dbw_timestamp = data_interpolation(miran_timestamps,
                                                                         front_wheel_angle_filtered,
                                                                         dbw_timestamps)
        # do sine curve fit for steering_wheel_angle for freq identification
        sw_angle_freq = np.nan
        if self.range_set:
            amp_sw, sw_angle_freq, pha, sw_angle_offset = sine_curve_fit((dbw_timestamps, steering_wheel_angle))
        # find average corsspoint on x axis
        x_crosspoint_list = []
        N_ = 15
        for index in range(len(steering_wheel_angle)-N_+1):
            x_ = np.array(steering_wheel_angle[index:index + N_])
            y_ = np.array(front_wheel_angle_filtered_in_dbw_timestamp[index:index + N_])
            k_ = (sum(x_*y_) - sum(x_) * sum(y_) / N_) / (sum(x_*x_) - sum(x_) * sum(x_) / N_)
            b_ = sum(y_) / N_ - k_ * sum(x_) / N_
            error_ = np.sqrt(np.mean(np.power(k_ * x_ + b_ - y_, 2)))
            if error_ < 0.09:
                x_crosspoint_list.append(-b_/k_)

        cluster_agent = KMeansClusteringForFloatTwoGroups()
        cluster_agent.setup_data_points(x_crosspoint_list)
        corss_points = cluster_agent.do_clustering()

        # Do the plotting!
        matplotlib.rcParams.update({'font.size': 12})
        fig = plt.figure(num=0, figsize=(20, 7.5), dpi=80)
        fig.canvas.set_window_title(os.path.basename(self.file_name))
        plt.subplots_adjust(left=0.08, bottom=0.06, right=0.92, top=0.95, wspace=0.15, hspace=0.20)

        graph = fig.add_subplot(321)
        graph.set_title("steering  report")
        graph.plot(dbw_timestamps,
                   steering_wheel_angle,
                   label="real_steering_angle / deg, freq == {:.4f} Hz".format(sw_angle_freq/2.0/math.pi))
        plt.legend()
        plt.grid()

        graph = fig.add_subplot(323)
        graph.set_title("front_wheel_angle")
        graph.plot([msg.bag_time for msg in self.miran_sensor_data],
                   front_wheel_angle,
                   label="front_wheel_angle / deg")
        graph.plot([msg.bag_time for msg in self.miran_sensor_data],
                   front_wheel_angle_filtered,
                   label="front_wheel_angle_filtered / deg")
        plt.legend()
        plt.grid()

        graph = fig.add_subplot(325)
        graph.set_title("speed")
        graph.plot(dbw_timestamps,
                   [msg.speed for msg in self.dbw_reports_data],
                   label="speed / m/s")
        plt.legend()
        plt.grid()

        
        graph = fig.add_subplot(122)
        graph.set_title("phase_plot")
        graph.plot(steering_wheel_angle,
                   front_wheel_angle_filtered_in_dbw_timestamp,
                   label="estimated hysteresis: {:.2f} deg".format(abs(corss_points[0]-corss_points[1])))
        # graph.scatter([corss_points[0], corss_points[1]], [0.0, 0.0], color='r', zorder=100, s=30.0)
        plt.legend()
        plt.grid()
        
        if not args.save:
            plt.show()
        else:
            time_range_string = '_' + str(args.tmin) + '_to_' + str(args.tmax) if self.range_set else ''
            savefig_name = '.'.join(self.file_name.split('.')[:-1]) + '_savefig' + time_range_string + '.png'
            plt.savefig(savefig_name, dpi=80, 
                        facecolor='w', 
                        edgecolor='w', 
                        orientation='portrait', 
                        papertype=None, 
                        format='png')
        plt.close()


def main(args):
    dir_name_list = [item for argument in args.bagdir for item in argument.split(',') if item != '']
    baglist = BagFinder().find_bags_in_(dir_name_list)
    for bag_file in baglist:
        bag_reader = BagReader(bag_file)
        if bag_reader.read_bag():
            bag_reader.plot_data()

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Run trucksim on bag')
    parser.add_argument('bagdir', nargs='+', type=str, help="path to bag or db (can be more than one, separated by commas/spaces)")
    parser.add_argument('-v', '--verbose', help='Enable debug logging', action='store_true')
    parser.add_argument('--save', action='store_true', default=False,
                    dest='save',
                    help='Enable saving figures')
    parser.add_argument('--tmin', action='store', default=0.0,
                    dest='tmin', type=float,
                    help='Setting steer offset')
    parser.add_argument('--tmax', action='store', default=None,
                    dest='tmax', type=float,
                    help='xlim maximum')
    args = parser.parse_args()
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format='%(asctime)s|%(lev  elname)s|%(name)s(%(lineno)d): %(message)s')
    main(args)

