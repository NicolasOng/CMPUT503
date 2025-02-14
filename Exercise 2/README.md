# Exercise 2

This folder contains the code for exercise 2.

## Duckietown Use ROS Tutorial

[`ros-dtproject-tutorial`](/Exercise%202/ros-dtproject-tutorial/) is the ROS Duckietown Project template described [here](https://docs.duckietown.com/daffy/devmanual-software/beginner/ros/create-project.html). We've followed that tutorial to make the:
- ROS Publisher node
- ROS Subscriber node
- Camera Reader node
- Wheel Encoder Reader node
- Wheel Control node.

## Exercise 2 Tasks

[`exercise-2-template`](/Exercise%202/exercise-2-template/) is the template for exercise 2 where we implemented the main tasks we were given (straight line task, rotation task, D task, and the bonus tasks). For all these tasks, we calculate the odometry from the wheel ticks to keep track of bot rotation/position. We also have the code to plot the bot trajectories. All the duckiebot movement tasks are in [`template_move.py`](/Exercise%202/exercise-2-template/exercise-2/packages/odometry/src/template_move.py) and the plotting is in [`template_plot.py`](/Exercise%202/exercise-2-template/exercise-2/packages/odometry/src/template_plot.py).

To run the node, use
```sh
dts devel build -f
dts devel run -R csc22946 -L template_move -X
```
To decide which task the bot performs, change the method that runs in the code: `straight_line_task`, `rotation_task`, `d_task`, `reverse_park_task`, `eight_shape`.

The node calculates the bot's odometry and publishes it to the `/{self.vehicle_name}/exercise2/odometry` topic. To record this, use the following commands:
```sh
dts start_gui_tools csc22946
rosbag record /csc22946/kinematics_node/velocity /csc22946/left_wheel_encoder_node/tick /csc22946/right_wheel_encoder_node/tick /csc22946/exercise2/odometry
docker ps -a
docker cp [docker identifier]:/code/catkin_ws/src/dt-gui-tools/[bag filename] /home/
```
Then to plot, use:
```sh
python3 template_plot.py
```
