#!/usr/bin/env python3
 
# import required libraries
# pip install rosbags
# pip install -U bagpy
import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np

import math

import matplotlib.pyplot as plt
import json

# the radius of the wheels on the duckiebot
# from rosparam /csc22946/kinematics_node/radius
WHEEL_RADIUS = 0.0318 # meters
# the distance between the wheels and the center of the robot
# TODO: estimation
WHEEL_DIST = 0.2 # meters

def read_bagfile(bf, topic):
    # read the topic we're interested in
    topic_bag = bf.message_by_topic(topic)
    # convert to dataframe
    df_topic = pd.read_csv(topic_bag)
    # print df
    print(df_topic.columns.tolist())
    print(df_topic.head())
    return df_topic

def ticks_to_radians(df_ticks):
    '''
    duckiebot has a wheel encoder that measures the number of ticks.
    the resolution is the number of ticks per revolution.
    this function converts the ticks to radians.
    '''
    # get the resolution
    resolution = df_ticks['resolution'][0]
    df_ticks['radians'] = df_ticks["data"].apply(lambda x: (x / resolution) * 2 * math.pi)

def wheel_position_to_wheel_speed(df_rads):
    '''
    needs the radians column.
    creates a radians/ms column.
    applies to the previous time interval.
    first speed is 0.
    '''
    # Compute the average speed (angular velocity) in the last interval
    # the diff function computes the difference between the current and previous value
    df_rads['speed'] = df_rads['radians'].diff().fillna(0) / df_rads['Time'].diff().fillna(0)

def merge_wheel_dfs(df_left, df_right):
    '''
    merges the left and right wheel dataframes.
    should have: time, speed_left, speed_right, interval
    '''
    # Merge using outer join (to include all unique timestamps)
    df_combined = pd.merge(df_left, df_right, on="Time", how="outer", suffixes=('_left', '_right'))
    # Sort by time
    df_combined = df_combined.sort_values(by="Time")
    # Forward fill missing values to keep the latest speed
    df_combined.fillna(method='ffill', inplace=True)
    # add a new column for the time interval (amount of time between each row)
    df_combined['interval'] = df_combined['Time'].diff().fillna(0)
    # select only the necessary columns
    df_combined = df_combined[['Time', 'interval', 'speed_left', 'speed_right']]
    return df_combined

def wheel_speed_to_positional_and_rotational_velocity(df_bot):
    '''
    with a dataframe that has speed columns for both wheels,
    computes the robot's positional velocity, dpos,
    and rotational velocity, drot.
    pos_speed = (wheel_radius_l * wheel_speed_l) / 2 + (wheel_radius_r * wheel_speed_r) / 2
    angular_speed  = (wheel_radius_l * wheel_speed_l) / (2 * wheel_dist) + (wheel_radius_r * wheel_speed_r) / (2 * wheel_dist)
    '''
    df_bot["dpos"] = ((WHEEL_RADIUS * df_bot["speed_left"]) + (WHEEL_RADIUS * df_bot["speed_right"])) / 2
    df_bot["drot"] = ((WHEEL_RADIUS * df_bot["speed_left"]) + (WHEEL_RADIUS * df_bot["speed_right"])) / (2 * WHEEL_DIST)

def positional_velocity_to_position(df_bot):
    '''
    with a dataframe that has positional and rotational velocity columns,
    computes the robot's position, (x/y)rpos,
    and the robot's rotation, rrot.
    rrot_{t+1} = rrot_t + drot * interval
    xrpos_{t+1} = xrpos_t + dpos * interval * cos(rrot_t)
    yrpos_{t+1} = yrpos_t + dpos * interval * sin(rrot_t)
    '''
    # Compute rotation (cumulative sum of rotational change in each interval)
    df_bot['rrot'] = ((df_bot['drot'] / 2) * df_bot['interval']).cumsum()
    # Compute displacement (distance traveled in each interval)
    df_bot['distance'] = df_bot['dpos'] * df_bot['interval']
    # compute the x and y positions
    # cumulative sum of the x/y distance traveled in each interval * cos/sin of the rotation
    # need to shift the rotation column to get the previous rotation
    #df_bot['xrpos'] = (df_bot['distance'] * np.cos(df_bot['rrot'].shift(fill_value=0))).cumsum()
    #df_bot['yrpos'] = (df_bot['distance'] * np.sin(df_bot['rrot'].shift(fill_value=0))).cumsum()
    df_bot['xrpos'] = (df_bot['distance'] * np.cos(df_bot['rrot'])).cumsum()
    df_bot['yrpos'] = (df_bot['distance'] * np.sin(df_bot['rrot'])).cumsum()

def robot_to_arbitrary_frame(df_bot, theta, x, y):
    '''
    with a dataframe that has position and rotation columns,
    computes the robot's position and rotation in an arbitrary frame.
    first performs clockwise rotation by theta, then translates by x, y.
    xipos = (xrpos * cos(theta) + yrpos * sin(theta)) + x
    yipos = (xrpos * -sin(theta) + yrpos * cos(theta)) + y
    '''
    df_bot['xipos'] = (df_bot['xrpos'] * np.cos(theta) + df_bot['yrpos'] * np.sin(theta)) + x
    df_bot['yipos'] = (df_bot['xrpos'] * -np.sin(theta) + df_bot['yrpos'] * np.cos(theta)) + y

def convert_velocity_df(df):
    #['Time', 'header.seq', 'header.stamp.secs', 'header.stamp.nsecs', 'header.frame_id', 'v', 'omega']
    df_bot = df[['Time', 'v', 'omega']]
    df_bot['interval'] = df_bot['Time'].diff().fillna(0)
    # v -> dpos, omega -> drot
    df_bot.rename(columns={'v': 'dpos', 'omega': 'drot'}, inplace=True)
    return df_bot

def convert_odometry_df(df):
    # should have columns time, interval, xpos, ypos, theta, cpos, ctheta
    df_expanded = df["data"].apply(json.loads).apply(pd.Series)
    # rename columns for plotting
    df_expanded.rename(columns={'xpos': 'xipos', 'ypos': 'yipos'}, inplace=True)
    return df_expanded

def for_ticks(df_left, df_right):
    # first do pos/rot in robot frame (ie relative to starting position)
    # then do pos/rot in world frame (ie absolute position)
    # so wheel rotation (ticks -> radians)
    #   -> wheel speed (radians/ms)
    #       -> robot position speed (m/ms)
    #       -> robot rotation speed (rad/ms)
    #           -> position relative to initial (m)
    #           -> rotation relative to initial (rad)
    #               -> + convert to arbitrary frame
    ticks_to_radians(df_left)
    ticks_to_radians(df_right)
    wheel_position_to_wheel_speed(df_left)
    wheel_position_to_wheel_speed(df_right)
    df_bot = merge_wheel_dfs(df_left, df_right)
    wheel_speed_to_positional_and_rotational_velocity(df_bot)
    positional_velocity_to_position(df_bot)
    robot_to_arbitrary_frame(df_bot, 0, 0, 0)
    plot_trajectory(df_bot)

def for_velocity(df_velocity):
    df_bot = convert_velocity_df(df_velocity)
    #df_bot = df_bot.iloc[::3]
    positional_velocity_to_position(df_bot)
    robot_to_arbitrary_frame(df_bot, 0, 0, 0)
    plot_trajectory(df_bot)
    plot(df_bot, 'dpos')
    plot(df_bot, 'drot')
    plot(df_bot, 'rrot')

def plot_trajectory(df_bot, connect=True):
    # add your code here
    # read the data from bag file
    # compute the trajectory
    # plot the trajectory
    plt.figure(figsize=(6, 6))  # Set figure size

    if connect:
        plt.plot(df_bot['xipos'], df_bot['yipos'], marker="o", linestyle="-", label="Path")  # Lines + markers
    else:
        plt.scatter(df_bot['xipos'], df_bot['yipos'], label="Positions", color="red")  # Only points

    # Labels & Title
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Robot's Journey")
    plt.legend()
    plt.grid(True)  # Show grid
    plt.show()

def plot(df, col):
    plt.figure(figsize=(8, 5))
    plt.plot(df['Time'], df[col], marker='o', linestyle='-', color='b', label=f'{col} over time')

    # Labels and title
    plt.xlabel('Time')
    plt.ylabel(f'{col}')
    plt.title(f'Time vs {col}')
    plt.legend()

    # Set equal aspect ratio
    plt.gca().set_aspect('equal')

    # Show plot
    plt.show()
if __name__ == '__main__':
    bf = bagreader('2025-02-13straighttest4.bag')

    #df_left = read_bagfile(bf, '/csc22946/left_wheel_encoder_node/tick')
    #df_right = read_bagfile(bf, '/csc22946/right_wheel_encoder_node/tick')
    #df_velocity = read_bagfile(bf, '/csc22946/kinematics_node/velocity')
    df_odometry = read_bagfile(bf, '/csc22946/exercise2/odometry')

    df_odometry = convert_odometry_df(df_odometry)

    plot_trajectory(df_odometry)


