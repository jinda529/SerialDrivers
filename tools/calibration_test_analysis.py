# encoding: utf-8
#!/usr/bin/env python

import sys
import traceback
import argparse
import rosbag
import csv
import numpy as np
from std_msgs.msg import String
import logging
import math
import os
import re
import copy
import time
import munch
import matplotlib
import matplotlib.pyplot as plt
from scipy import interpolate
from dbw_mkz_msgs.msg import *
from control import dbw_reports_pb2
from control import control_command_pb2
from planning import planning_trajectory_pb2
from prediction import prediction_obstacle_pb2
from localization import localization_pb2


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


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
        self.imu_data = []
        self.dbw_reports_data = []
        self.steering_angle_sensor_data = []
        self.miran_sensor_data = []
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
            if topic == '/imu/data':
                self.last_time_sec = t.to_sec()
                if self.start_time_sec is None:
                    self.start_time_sec = t.to_sec()
                try:
                    self.imu_data.append(munch.Munch(
                        bag_time=t.to_sec() - self.start_time_sec,
                        msg_time=msg.header.timestamp_msec,
                        yawrate=msg.angular_velocity.z
                    ))
                except AttributeError:
                    pass
            if topic == '/vehicle/dbw_reports':
                self.last_time_sec = t.to_sec()
                if self.start_time_sec is None:
                    self.start_time_sec = t.to_sec()
                pb = dbw_reports_pb2.DbwReports()
                pb.ParseFromString(msg.data)
                try:
                    self.dbw_reports_data.append(munch.Munch(
                        bag_time=t.to_sec() - self.start_time_sec,
                        msg_time=pb.header.timestamp_msec,
                        steering_wheel_angle=pb.steering_report.steering_wheel_angle,
                        steering_wheel_angle_compensation=pb.steering_report.steering_wheel_angle_compensation,
                        speed=pb.steering_report.speed,
                        gear=pb.gear_report.state,
                        yawrate=pb.vehicle_dynamic.angular_velocity.z
                    ))
                except AttributeError:
                    pass
            if topic == '/SteeringWheelSensorInfo':
                self.last_time_sec = t.to_sec()
                if self.start_time_sec is None:
                    self.start_time_sec = t.to_sec()
                try:
                    self.steering_angle_sensor_data.append(munch.Munch(
                        bag_time=t.to_sec() - self.start_time_sec,
                        msg_time=msg.timestamp,
                        torque=msg.torque,
                        angle=msg.angle
                    ))
                except AttributeError:
                    pass
            if topic == '/MiranSensorInfo':
                self.last_time_sec = t.to_sec()
                if self.start_time_sec is None:
                    self.start_time_sec = t.to_sec()
                try:
                    self.miran_sensor_data.append(munch.Munch(
                        bag_time=t.to_sec() - self.start_time_sec,
                        msg_time=msg.timestamp,
                        length=msg.length
                    ))
                except AttributeError:
                    pass
        if not self.miran_sensor_data or not self.dbw_reports_data:
            logger.error('Not enough valid data in bag: {}'.format(self.file_name))
            return False
        return True


    def linear_regression_calculation(self):
        miran_sensor_interp = interpolate.interp1d([msg.bag_time for msg in self.miran_sensor_data],
                                                   [msg.length for msg in self.miran_sensor_data])
        miran_range_min = min([msg.bag_time for msg in self.miran_sensor_data])
        miran_range_max = max([msg.bag_time for msg in self.miran_sensor_data])
        # imu_interp = interpolate.interp1d([msg.bag_time for msg in self.imu_data],
        #                                   [msg.yawrate for msg in self.imu_data])
        # imu_range_min = min([msg.bag_time for msg in self.imu_data])
        # imu_range_max = max([msg.bag_time for msg in self.imu_data])
        for msg in self.dbw_reports_data:
            msg.miran_sensor_value = miran_sensor_interp(msg.bag_time) if miran_range_min < msg.bag_time < miran_range_max else np.nan
            # msg.yawrate = imu_interp(msg.bag_time) if imu_range_min < msg.bag_time < imu_range_max else np.nan
            msg.road_wheel_angle = 57.3 * 3.9 / msg.speed * msg.yawrate if msg.speed > 0.5 else np.nan

        road_wheel_angle = [msg.road_wheel_angle for msg in self.dbw_reports_data if not np.isnan(msg.road_wheel_angle) and not np.isnan(msg.miran_sensor_value)]
        miran_sensor_data = [msg.miran_sensor_value for msg in self.dbw_reports_data if not np.isnan(msg.road_wheel_angle) and not np.isnan(msg.miran_sensor_value)]
        print(len(miran_sensor_data), len(road_wheel_angle))

        x_ = np.array(miran_sensor_data)
        y_ = np.array(road_wheel_angle)
        N_ = len(y_)
        linear_regression_k = (sum(x_*y_) - sum(x_) * sum(y_) / N_) / (sum(x_*x_) - sum(x_) * sum(x_) / N_)
        linear_regression_b = sum(y_) / N_ - linear_regression_k * sum(x_) / N_
        
        print("linear_regression_k = {0:.6f}".format(linear_regression_k))
        print("linear_regression_b = {0:.6f}".format(linear_regression_b))

        fig = plt.figure(num=0, figsize=(45, 30), dpi=80)
        fig.canvas.set_window_title(os.path.basename(self.file_name))
        plt.subplots_adjust(left=0.08, bottom=0.06, right=0.92, top=0.95, wspace=0.15, hspace=0.20)

        graph = fig.add_subplot(122)
        graph.set_title('phase plot')
        graph.plot([msg.steering_wheel_angle + msg.steering_wheel_angle_compensation for msg in self.dbw_reports_data], [msg.miran_sensor_value for msg in self.dbw_reports_data], color='blue', label='phase', linewidth=1.0, marker='*')
        plt.xlabel('steering_wheel_angle')
        plt.ylabel('MiranSensorInfo')
        plt.legend()
        plt.grid()

        graph = fig.add_subplot(121)
        graph.set_title('phase plot')
        graph.plot([(-5.0 - linear_regression_b) / linear_regression_k, (5.0 - linear_regression_b) / linear_regression_k], [-5.0, 5.0], color='blue', label='y = {:.4f} * x + {:.4f}'.format(linear_regression_k, linear_regression_b), linewidth=2.0, marker='*', zorder=200)
        graph.scatter(miran_sensor_data, road_wheel_angle, color='red', label='phase', linewidth=1.0, marker='*', zorder=100)
        # graph.plot(miran_sensor_data_tmp, road_wheel_angle_tmp, color='black', label='phase', linewidth=1.0, marker='*', zorder=0)
        plt.xlabel('miran_sensor_data')
        plt.ylabel('road_wheel_angle')
        plt.legend()
        plt.grid()
        
        if not args.save:
            plt.show()
        else:
            savefig_name = '.'.join(self.file_name.split('.')[:-1]) + '_calib.png'
            plt.savefig(savefig_name, dpi=80, 
                        facecolor='w', 
                        edgecolor='w', 
                        orientation='portrait', 
                        papertype=None, 
                        format='png')
        plt.close()

    
    def plot_data(self):
        matplotlib.rcParams.update({'font.size': 12})
        fig = plt.figure(num=0, figsize=(45, 30), dpi=80)
        fig.canvas.set_window_title(os.path.basename(self.file_name))
        plt.subplots_adjust(left=0.08, bottom=0.06, right=0.92, top=0.95, wspace=0.15, hspace=0.20)

        graph = fig.add_subplot(211)
        graph.set_title("phase plot")
        graph.plot([msg.bag_time for msg in self.dbw_reports_data],
                   [msg.steering_wheel_angle * 57.3 for msg in self.dbw_reports_data],
                   label="steering_wheel_angle")
        graph.plot([msg.bag_time for msg in self.dbw_reports_data],
                   [msg.steering_wheel_angle * 57.3 + msg.steering_wheel_angle_compensation * 57.3 for msg in self.dbw_reports_data],
                   label="steering_wheel_angle + compensation")
        graph.plot([msg.bag_time for msg in self.steering_angle_sensor_data],
                   [-msg.angle / 57.3 * 57.3 for msg in self.steering_angle_sensor_data],
                   label="steeringwheel")
        plt.legend()
        plt.grid()

        data_interp_no_comp = interpolate.interp1d([msg.bag_time for msg in self.dbw_reports_data],
                                                   [msg.steering_wheel_angle for msg in self.dbw_reports_data])
        data_interp_with_comp = interpolate.interp1d([msg.bag_time for msg in self.dbw_reports_data],
                                                     [msg.steering_wheel_angle
                                                      + msg.steering_wheel_angle_compensation for msg in self.dbw_reports_data])
        max_bagtime = max([msg.bag_time for msg in self.dbw_reports_data])
        min_bagtime = min([msg.bag_time for msg in self.dbw_reports_data])
        for msg in self.steering_angle_sensor_data:
            msg.interpolated_no_comp_angle = data_interp_no_comp(msg.bag_time) if min_bagtime < msg.bag_time < max_bagtime else np.nan
            msg.interpolated_with_comp_angle = data_interp_with_comp(msg.bag_time) if min_bagtime < msg.bag_time < max_bagtime else np.nan

        graph = fig.add_subplot(212)
        graph.set_title("phase plot")
        graph.plot([msg.bag_time for msg in self.dbw_reports_data],
                   [msg.steering_wheel_angle_compensation * 57.3 for msg in self.dbw_reports_data],
                   label="steering_wheel_angle")
        graph.plot([msg.bag_time for msg in self.steering_angle_sensor_data],
                   [msg.interpolated_no_comp_angle * 57.3 - (-msg.angle / 57.3 * 57.3) for msg in self.steering_angle_sensor_data],
                   label="interpolated_no_comp_angle")
        graph.plot([msg.bag_time for msg in self.steering_angle_sensor_data],
                   [msg.interpolated_with_comp_angle * 57.3 - (-msg.angle / 57.3 * 57.3) for msg in self.steering_angle_sensor_data],
                   label="interpolated_with_comp_angle")
        plt.legend()
        plt.grid()
        
        if not args.save:
            plt.show()
        else:
            savefig_name = '.'.join(self.file_name.split('.')[:-1]) + '_savefig.png'
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
            bag_reader.linear_regression_calculation()

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Run trucksim on bag')
    parser.add_argument('bagdir', nargs='+', type=str, help="path to bag or db (can be more than one, separated by commas/spaces)")
    parser.add_argument('-v', '--verbose', help='Enable debug logging', action='store_true')
    parser.add_argument('--save', action='store_true', default=False,
                    dest='save',
                    help='Enable saving figures')
    args = parser.parse_args()
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format='%(asctime)s|%(levelname)s|%(name)s(%(lineno)d): %(message)s')
    main(args)

