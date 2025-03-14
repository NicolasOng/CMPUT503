# Exercise 3: Computer Vision & Controllers for Robotics

Group members:  Nicolas Ong, Truong Pham, Martin McLaren

This is the folder that contains all the code for exercise 3 of CMPUT503. In this lab exercise, we used computer vision to enable our robot to perceive features of its environment and adjust its behavior accordingly. In addition, we used a feedback-based controller to correct for errors in our robot's movement, significantly improving its performance in driving along a track compared to our previous dead reckoning-based approach. We gained an understanding of some of the principles of computer vision and control theory, and how to apply these to solve problems related to robot control.

## Dependencies
The following dependencies and systems are required to run the software contained in this exercise repository.

- Ubuntu 22.04
- ROS 2 Humble
- Duckietown Shell
- Docker
- NumPy
- OpenCV

## Code Contents

- `misc` contains some test code, notably
  - `plot_bag.py`: plots the recorded bags - has some functions for plotting our custom odometry topic, along with others the bot produces (like `kinematics_node/velocity` or `velocity_to_pose_node/pose`)
  - `process_camera.py`: mostly testing OpenCV functions for lane detection.
- `exercise-3` is the actual ROS code that runs on the Duckiebot.
  - `lane_detection.py`: has most of the code for Part One - Computer Vision (excluding Lane-Based Behavioral Execution)
  - `camera_detection.py`: the camera detection code for the rest of the exercise. Important functions:
    - detects red/green/blue markings on the ground, and calculates their x and y distances from the bot. Used in the Lane-Based Behavioral Execution task.
    - detects white and yellow lines defining lanes, and related errors. Used in the Straight Line and Lane Following tasks.
    - detects how far to the side a white marking is from the bot. A faster and more effective method used in the Straight Line and Lane Following tasks.
    - the above results are published to topics for other nodes to use.
  - `move.py`: adapted from exercise 2 - calculates the bot's odometry
    - odometry is used by movement functions that are exposed as service calls for other nodes
    - odometry is also published to a topic that can be recorded by a ROSbag and plotted
  - `color_based_movement.py`: Primary node for Lane-Based Behavioral Execution. Uses:
    - `camera_detection.py` to look for the R/G/B markings
    - `move.py` to do the 90 degree turns
  - `controller.py` Primary node for the Straight Line and Lane Following Tasks.
    - Uses `camera_detection.py` to get line following errors.

## How to Run

Running locally (on laptop or PC) - better for faster development/build times and more compute
```sh
dts devel build -f

dts devel run -R csc22946 -L lane-detection -X
dts devel run -R csc22946 -L move -X -n "odometry"
dts devel run -R csc22946 -L camera-detection -X -n "camera"
dts devel run -R csc22946 -L controller -X -n "controller"
dts devel run -R csc22946 -L color-move -X -n "color-move"
```

Running directly on the Duckiebot - lower latency over the network, but less compute
```sh
dts devel build -H csc22946 -f

dts devel run -H csc22946 -L move -n "odometry"
dts devel run -H csc22946 -L camera-detection -n "camera"
dts devel run -H csc22946 -L controller -n "controller"
```

Recording & Saving ROSbags
```sh
dts start_gui_tools csc22946

rosbag record /csc22946/velocity_to_pose_node/pose /csc22946/exercise3/odometry /csc22946/kinematics_node/velocity /csc22946/left_wheel_encoder_node/tick /csc22946/right_wheel_encoder_node/tick /csc22946/wheels_driver_node/wheels_cmd_executed

docker ps -a
docker cp [docker container]:/code/catkin_ws/src/dt-gui-tools/[bag filename] /home/nicolas/
```
