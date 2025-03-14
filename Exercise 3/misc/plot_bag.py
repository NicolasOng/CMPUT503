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

def read_bagfile(bf, topic):
    # read the topic we're interested in
    topic_bag = bf.message_by_topic(topic)
    # convert to dataframe
    df_topic = pd.read_csv(topic_bag)
    # print df
    print(len(df_topic))
    print(df_topic.columns.tolist())
    print(df_topic.head())
    return df_topic

# Odometry Functions

def positions_to_positional_velocity(df, theta_fix=1):
    '''
    given a dataframe with x and y positions over time,
    calculates the 'dpos' - change in distance between each row.
    also dtheta.
    Also 'vpos' - the speed
    '''
    df['dpos'] = np.sqrt(np.diff(df['xpos'], prepend=np.nan)**2 + np.diff(df['ypos'], prepend=np.nan)**2)
    df['drot'] = np.diff(df['theta'], prepend=np.nan) * theta_fix
    df['vpos'] = df['dpos'] / df['interval']
    df['vrot'] = df['drot'] / df['interval']
    df.fillna(0, inplace=True)

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
    df_bot['theta'] = (df_bot['drot'] * df_bot['interval']).cumsum()
    # Compute displacement (distance traveled in each interval)
    df_bot['distance'] = df_bot['dpos'] * df_bot['interval']
    # compute the x and y positions
    # cumulative sum of the x/y distance traveled in each interval * cos/sin of the rotation
    df_bot['xpos'] = (df_bot['distance'] * np.cos(df_bot['theta'])).cumsum()
    df_bot['ypos'] = (df_bot['distance'] * np.sin(df_bot['theta'])).cumsum()
    df_bot.drop(columns=['distance'], inplace=True)

def robot_to_arbitrary_frame(df_bot, theta, x, y):
    '''
    with a dataframe that has position and rotation columns,
    computes the robot's position and rotation in an arbitrary frame.
    first performs clockwise rotation by theta, then translates by x, y.
    xipos = (xrpos * cos(theta) + yrpos * sin(theta)) + x
    yipos = (xrpos * -sin(theta) + yrpos * cos(theta)) + y
    '''
    df_bot['xipos'] = (df_bot['xpos'] * np.cos(theta) + df_bot['ypos'] * np.sin(theta)) + x
    df_bot['yipos'] = (df_bot['xpos'] * -np.sin(theta) + df_bot['ypos'] * np.cos(theta)) + y
    df_bot['xpos'] = df_bot['xipos']
    df_bot['ypos'] = df_bot['yipos']
    df_bot['theta'] = df_bot['theta'] - theta
    df_bot.drop(columns=['xipos', 'yipos'], inplace=True)

# Functions for specific bag types

def convert_odometry_df(df):
    # should have columns time, interval, xpos, ypos, theta, cpos, ctheta
    df_expanded = df["data"].apply(json.loads).apply(pd.Series)
    return df_expanded

def for_odometry(df_odometry, fix=-2.25):
    df_odometry = convert_odometry_df(df_odometry)

    positions_to_positional_velocity(df_odometry, fix) # 3.875, 2.125
    positional_velocity_to_position(df_odometry)
    robot_to_arbitrary_frame(df_odometry, -math.pi/2 * 0.9, 0, 0)

    plot_trajectory(df_odometry)
    #plot(df_odometry, 'ctheta')
    #plot(df_odometry, 'cpos')

def for_pose(df_pose):
    #['Time', 'header.seq', 'header.stamp.secs', 'header.stamp.nsecs', 'header.frame_id', 'x', 'y', 'theta']
    df_pose['time'] = df_pose['Time']
    df_pose["xpos"] = df_pose["x"]
    df_pose["ypos"] = df_pose["y"]
    plot_trajectory(df_pose, color="theta")

def for_velocity(df_velocity):
    #['Time', 'header.seq', 'header.stamp.secs', 'header.stamp.nsecs', 'header.frame_id', 'v', 'omega']
    df_velocity['time'] = df_velocity['Time']
    df_velocity['drot'] = df_velocity['omega']
    df_velocity['dpos'] = df_velocity['v']
    df_velocity['interval'] = df_velocity['time'].diff().fillna(0)
    positional_velocity_to_position(df_velocity)
    plot_trajectory(df_velocity, color="dpos")
    #plot(df_velocity, "v")
    #plot(df_velocity, "omega")

def for_ticks(df_ticks):
    # ['Time', 'header.seq', 'header.stamp.secs', 'header.stamp.nsecs', 'header.frame_id', 'data', 'resolution', 'type']
    df_ticks['time'] = df_ticks['Time']
    plot(df_ticks, "data")

def for_wheels_cmd_executed(df_wce):
    #['Time', 'header.seq', 'header.stamp.secs', 'header.stamp.nsecs', 'header.frame_id', 'vel_left', 'vel_right']
    df_wce['time'] = df_wce['Time']
    plot(df_wce, "vel_left")
    plot(df_wce, "vel_right")

# Plotting functions

def plot_trajectory(df_bot, color="vpos", arrows=True, arrow_size=0.0125):
    plt.figure(figsize=(6, 6))  # Set figure size

    # Scatter plot with color based on speed
    scatter = plt.scatter(df_bot['xpos'], df_bot['ypos'], c=df_bot[color], cmap='viridis', label="Positions")

    # Plot lines connecting the points
    plt.plot(df_bot['xpos'], df_bot['ypos'], linestyle="-", color="gray", alpha=0.5, label="Path")

    # Add rotated markers based on theta
    if arrows:
        scale = 0.0001
        for i in range(len(df_bot)):
            x, y, theta = df_bot['xpos'].iloc[i], df_bot['ypos'].iloc[i], df_bot['theta'].iloc[i]
            dx, dy = np.cos(theta) * scale, np.sin(theta) * scale  # Scale for visibility
            plt.arrow(x, y, dx, dy, head_width=arrow_size, head_length=arrow_size, color=scatter.to_rgba(df_bot[color].iloc[i]))

    # Labels & Title
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Robot's Journey")
    plt.legend()
    plt.grid(True)
    plt.colorbar(scatter, label=color)  # Add colorbar for speed
    plt.gca().set_aspect('equal')
    plt.show()

def plot(df, col):
    plt.figure(figsize=(8, 5))
    plt.plot(df['time'], df[col], marker='o', linestyle='-', color='b', label=f'{col} over time')

    # Labels and title
    plt.xlabel('time')
    plt.ylabel(f'{col}')
    plt.title(f'time vs {col}')
    plt.legend()

    # Show plot
    plt.show()

if __name__ == '__main__':
    #bf = bagreader('bags/2025-03-11Plap.bag')
    #bf = bagreader('bags/2025-03-11PDlap2.bag')
    bf = bagreader('bags/2025-03-11 PIDlap.bag')

    #for_odometry(read_bagfile(bf, '/csc22946/exercise3/odometry'), fix=8)
    #'''
    for i in range(16):
        i = i / 3
        for_odometry(read_bagfile(bf, '/csc22946/exercise3/odometry'), fix=i)
    #'''
    #for_pose(read_bagfile(bf, '/csc22946/velocity_to_pose_node/pose'))
    #for_velocity(read_bagfile(bf, '/csc22946/kinematics_node/velocity'))
    #for_ticks(read_bagfile(bf, '/csc22946/left_wheel_encoder_node/tick'))
    #for_wheels_cmd_executed(read_bagfile(bf, '/csc22946/wheels_driver_node/wheels_cmd_executed'))
