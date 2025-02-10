#!/usr/bin/env python3
 
# import required libraries
# pip install rosbags
# pip install -U bagpy
import bagpy
from bagpy import bagreader
import pandas as pd

import math

def plot_trajectory(**kwargs):
    # add your code here
    # read the data from bag file
    # compute the trajectory
    # plot the trajectory
    pass

# define other functions if needed
def ticks_to_radians(df_ticks, resolution):
    df_ticks['radians'] = df_ticks["ticks"].apply(lambda x: (x / resolution) * 2 * math.pi)

def wheel_position_to_speed(df_ticks):
    '''
    needs the radians column.
    creates a radians/ms column.
    applies to the previous time interval.
    first speed is 0.
    '''

if __name__ == '__main__':
    # access the bag file
    # call the plot_trajectory function

    b = bagreader('2025-02-07-23-16-48.bag')

    # replace the topic name as per your need
    wheels_cmd = b.message_by_topic('/csc22946/wheels_driver_node/wheels_cmd')
    left_tick = b.message_by_topic('/csc22946/left_wheel_encoder_node/tick')
    right_tick = b.message_by_topic('/csc22946/right_wheel_encoder_node/tick')

    # probably something like time (ms), data.resolution, data.data (the ticks)
    df_wheels_cmd = pd.read_csv(wheels_cmd)
    df_left_tick = pd.read_csv(left_tick)
    df_right_tick = pd.read_csv(right_tick)

    # get the resolution
    resolution = df_left_tick['resolution'][0]


