import math
import pandas as pd

import plot_bag

def point_on_line(length, speed, time, start_x=0, start_y=0, start_theta=0):
    cur_distance = speed * time
    x = start_x + cur_distance * math.cos(start_theta)
    y = start_y + cur_distance * math.sin(start_theta)
    return x, y, start_theta

def rotation_in_place(total_rotation, speed, time, start_x=0, start_y=0, start_theta=0):
    return start_x, start_y, start_theta + (speed * time)

def point_on_circle(radius, distance, speed, time, start_x=0, start_y=0, start_theta=0):
    """
    Calculate the point on a circle.

    radius: The radius of the circle (m). Make it negative to curve the other way.
    distance: The distance along the circumference of the circle to travel (m).
    speed: The speed to travel the circle in (m/s)
    time: The current time, determines the current ideal point along the circle

    Useful: to get the final x/y/theta, use time = distance / speed
    """
    # Calculate the angle in radians
    angle = start_theta + (speed * time) / radius

    # Calculate the start of the circle
    cstart_x = radius * math.cos(start_theta)
    cstart_y = radius * math.sin(start_theta)

    # Calculate x and y coordinates
    x = (start_x - cstart_x) + radius * math.cos(angle)
    y = (start_y - cstart_y) + radius * math.sin(angle)

    return (x, y, angle + math.pi/2)

def plot_circle(radius, distance, speed):
    points = []
    time = 0
    distance_travelled = speed * time
    while distance_travelled < distance:
        (x, y, angle) = point_on_circle(radius, distance, speed, time, 5, 10, math.pi/2)
        points.append((time, x, y, angle))
        # each loop is 1s
        time += 1
        distance_travelled = speed * time
    
    (x, y, angle) = point_on_circle(radius, distance, speed, distance/speed, 5, 10, math.pi/2)
    points.append((distance/speed, x, y, angle))

    # Create a DataFrame
    df = pd.DataFrame(points, columns=['time', 'xpos', 'ypos', 'theta'])

    plot_bag.plot_trajectory(df, color="time", arrows=True, arrow_size=1)

def plot_line(distance, speed):
    points = []
    time = 0
    distance_travelled = speed * time
    while distance_travelled < distance:
        x, y, angle = point_on_line(distance, speed, time, 5, 10, math.pi/2)
        points.append((time, x, y, angle))
        # each loop is 1s
        time += 1
        distance_travelled = speed * time
    
    (x, y, angle) = point_on_line(distance, speed, distance/speed, 5, 10, math.pi/2)
    points.append((distance/speed, x, y, angle))

    # Create a DataFrame
    df = pd.DataFrame(points, columns=['time', 'xpos', 'ypos', 'theta'])

    plot_bag.plot_trajectory(df, color="time", arrows=True, arrow_size=1)

#plot_circle(5, 5 * 2 * math.pi * 0.75, (5 * 2 * math.pi * 0.75) / 20)
plot_line(20, 20 / 100)