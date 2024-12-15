# Experimental_Assignment1
# Marker Tracker Project

A ROS package for marker detection, tracking, and alignment using a robot's camera. The markers are placed on a circle around the robot. This package enables the robot to gather marker information, navigate to them, and visually highlight detected markers on the camera feed. Two different codes are written. In the first Python code `fix_controller.py`, the whole chassis of the car rotates and detects the markers placed around it. In the second code `rotating_controller.py`, only the link that carries the camera rotates while the car chassis stays still.

---

## Table of Contents

1. [Features](#features)  
2. [Dependencies](#dependencies)  
3. [Usage](#usage)  
4. [Topics](#topics)  
5. [Code Overview](#code-overview)  
6. [Author](#author)  

---

## Features

- Identifies markers based on their unique IDs.  
- Tracks marker positions and aligns the robot's camera center with detected markers.  
- Draws a circle around detected markers in the camera feed and publishes the processed image.  
- Provides a mechanism for the robot to rotate and gather data on multiple markers.  
- Stops rotation and alignment once all markers are processed.  

---

## Dependencies

Ensure the following software and libraries are installed:

### Required Software
- **Python 3.x**
- **ROS Noetic**

### Python Libraries
- `numpy`
- `scipy`
- `opencv-python`
- `imutils`

### Install Python dependencies:
```bash
pip install numpy scipy imutils opencv-python
```

---

## Usage

1. Clone the following Aruco ROS Repository in your workspace:
```bash
git clone https://github.com/CarmineD8/aruco_ros
```
2. Clone the `robot_urdf` folder into your workspace.
3. From the `aruco_ros` folder in this repository, substitute the `.cpp` file in the `src` folder with the one in the `src` folder of the cloned `aruco_ros` repository.
4. Build your workspace:
```bash
catkin_make
```
5. Run the `fix_controller` node:
```bash
roslaunch robot_urdf assignment1_1.launch
```
6. Ensure the following topics are active and functioning:

- `/robot4/camera1/image_raw/compressed`
- `/robot4/camera1/camera_info`
- `/marker/id_number`
- `/marker/center_loc`

7. The robot will rotate to gather marker information and align its camera with the markers one by one.
8. For the case where only the camera link rotates while the car stays fixed, run the following launch file:
```bash
roslaunch robot_urdf assignment1_2.launch
```

---

## Topics

### Published Topics
`/output/image_raw/compressed`

- **Message Type**: `sensor_msgs/CompressedImage`
- **Description**: Publishes the processed image with detected markers highlighted.

`/cmd_vel`

- **Message Type**: `geometry_msgs/Twist`
- **Description**: Publishes velocity commands to navigate the robot towards markers.

### Subscribed Topics
`/robot4/camera1/image_raw/compressed`

- **Message Type**: `sensor_msgs/CompressedImage`
- **Description**: Subscribes to the raw camera feed for marker detection.

`/robot4/camera1/camera_info`

- **Message Type**: `sensor_msgs/CameraInfo`
- **Description**: Subscribes to the camera's resolution and focal properties.

`/marker/id_number`

- **Message Type**: `std_msgs/Int32`
- **Description**: Subscribes to the ID of the currently detected marker.

`/marker/center_loc`

- **Message Type**: `geometry_msgs/Point`
- **Description**: Subscribes to the center coordinates of the detected marker.

---

## Code Overview

The `fix_controller` class is the core of this package. Below are the key methods and their functions:

### `Camera_callback(self, msg)`

- Extracts the camera's center coordinates from the `CameraInfo` message.

### `Id_callback(self, msg)`

- Updates the detected marker's ID (`id_number`).

### `Center_callback(self, msg)`

- Tracks the detected marker's center coordinates and stores them in a dictionary.

### `Controller_callback(self, msg)`

Implements:
  - Marker detection and alignment.
  - Rotational strategy to gather all marker information.
  - Drawing circles on the processed image for visual feedback.

### `main()`

- Initializes the ROS node and starts the `fix_controller`.

