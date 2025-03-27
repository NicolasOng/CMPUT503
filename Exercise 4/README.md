# Exercise 4: AprilTag Detection and Safety with Robots

Group members:  Nicolas Ong, Truong Pham, Martin McLaren

This is the folder that contains all the code for exercise 4 of CMPUT503. In this lab exercise, we continue the use of computer vision for feature detection in our robot's environment, including road features like road signs and crosswalks, as well as pedestrians and other Duckiebots. Our robot reacts to each detection appropriately, navigating the course in a safe manner. We apply the same principles of vision and robot control principles that were introduced in past exercises, extending the behavior of our robot in preparation for the final course project. 

## Dependencies
The following dependencies and systems are required to run the software contained in this exercise repository.

- Ubuntu 22.04
- ROS 2 Humble
- Duckietown Shell
- Docker
- NumPy
- OpenCV
- dt_apriltags

## Code Contents

- `test_scripts` contains some test and experiment code written for this exercise.
  - `apriltag_test.py`: testing apriltag detection loop, adjusting preprocessing, etc.
  - `vehicle_detection.py`: test environment for detecting other Duckiebots.
  - `ground_color_detection.py`: test environment for detecting coloured tape (lane markers, stop lines, etc.)
- `exercise-4` contains the actual ROS code that runs on the Duckiebot. New for this exercise are the following:
  - `camera_detection.py`: the camera detection code for this of the exercise. Expanded from previous exercise to include additional functions:
    - detecting apriltags, finding prominent tags based on their area. Publishes results and image feed with tag bounding box to topic.
    - detecting pedestrians, publishing results for use by other nodes.
    - detecting other duckiebots, publishing results for use by other nodes.
  - `tag_loop.py` Primary node for executing the AprilTag detection and lane following loop.
    - Uses `camera_detection.py` to get detect AprilTags and lane features.
    - Uses `odometry.py` to perform 90° rotation (track for this exercise has squared corners, presents some difficulty for our lane following procedure).
  - `pedestrians.py`: Primary node for stopping at crosswalks and waiting for pedestrians to cross.
    - Uses `camera_detection.py` to detect crosswalks and pedestrians.
  - `vehicle_detection.py`: Primary node for safely maneuvering around stopped Duckiebots.
    - Uses `camera_detection.py` to detect duckiebots.

## How to Run

Running locally (on laptop or PC) - better for faster development/build times and more compute
```sh
dts devel build -f

dts devel run -R csc22946 -L camera-detection -X -n "camera"
dts devel run -R csc22946 -L odometry -X -n "odometry"
dts devel run -R csc22946 -L tag-loop -X -n "tag"
dts devel run -R csc22946 -L pedestrians -X -n "pedestrian"
dts devel run -R csc22946 -L vehicle-detection -X -n "vehicle"
```

Running directly on the Duckiebot - lower latency over the network, but less compute (same
commands as above, but with -H flag):
```sh
dts devel build -H csc22946 -f

dts devel run -H csc22946 -L camera-detection -n "camera"
dts devel run -H csc22946 -L odometry -n "odometry"
dts devel run -H csc22946 -L tag-loop -n "tag"
dts devel run -H csc22946 -L pedestrians -n "pedestrian"
dts devel run -H csc22946 -L vehicle-detection -n "vehicle"
```