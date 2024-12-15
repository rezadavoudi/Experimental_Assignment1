# Experimental_Assignment1

Reza Davoudi Beni s6115320

# Marker Detection and Tracking

This ROS package enables robots to detect and align with markers using a camera. Markers are placed in a circular arrangement around the robot. Two approaches are implemented: rotating the robot base (`fix_controller.py`) or rotating the camera mount (`rotating_controller.py`).

---

## Table of Contents

1. [Requirements](#requirements)  
2. [Overview](#overview)  
3. [Instructions](#instructions)  
4. [ROS Topics](#ros-topics)  
5. [Workflow](#workflow)  

---

## Requirements

### Software
- **Python 3.x**
- **ROS Noetic**

### Python Libraries
Install the required libraries:
```bash
pip install numpy scipy imutils opencv-python
```

---

## Overview

- Detects markers by their IDs.  
- Tracks marker positions via the camera feed.  
- Publishes an annotated feed highlighting detected markers.  
- Offers two detection methods: robot base rotation or camera mount rotation.  

---

## Instructions

1. Clone the Aruco ROS repository:
```bash
git clone https://github.com/CarmineD8/aruco_ros
```
2. Add `robot_urdf` to your workspace.
3. Replace the `.cpp` file in the `src` folder of the Aruco ROS repository.
4. Build the workspace:
```bash
catkin_make
```
5. To rotate the robot base, launch:
```bash
roslaunch robot_urdf assignment1_1.launch
```
6. Check the following topics are active:

- `/robot4/camera1/image_raw/compressed`
- `/robot4/camera1/camera_info`
- `/marker/id_number`
- `/marker/center_loc`

7. For camera mount rotation, launch:
```bash
roslaunch robot_urdf assignment1_2.launch
```

---

## ROS Topics

### Published Topics

`/output/image_raw/compressed`
- **Type**: `sensor_msgs/CompressedImage`
- **Description**: Publishes the processed camera feed.

`/cmd_vel`
- **Type**: `geometry_msgs/Twist`
- **Description**: Publishes velocity commands for alignment.

### Subscribed Topics

`/robot4/camera1/image_raw/compressed`
- **Type**: `sensor_msgs/CompressedImage`
- **Description**: Processes raw camera data.

`/robot4/camera1/camera_info`
- **Type**: `sensor_msgs/CameraInfo`
- **Description**: Provides camera parameters.

`/marker/id_number`
- **Type**: `std_msgs/Int32`
- **Description**: Supplies the marker ID.

`/marker/center_loc`
- **Type**: `geometry_msgs/Point`
- **Description**: Specifies the marker’s center position.

---

## Workflow

### Key Components

#### `Camera_callback(self, msg)`
- Extracts camera data for calibration.

#### `Id_callback(self, msg)`
- Updates detected marker ID.

#### `Center_callback(self, msg)`
- Logs the marker’s position.

#### `Controller_callback(self, msg)`
- Executes detection and alignment.

#### `main()`
- Initializes the ROS node and starts the controller.

---

