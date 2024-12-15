# Experimental_Assignment1

Reza Davoudi Beni s6115320

# Marker Detection and Alignment

This ROS package is designed to detect and track markers using a robot-mounted camera. Markers are arranged in a circular pattern around the robot, and the package facilitates the collection of marker data and alignment of the camera with the detected markers. Two distinct approaches are implemented: in the first, `fix_controller.py`, the entire robot base rotates to identify the markers. In the second, `rotating_controller.py`, only the camera mount rotates while the robot base remains stationary.

---

## Table of Contents

1. [Highlights](#highlights)  
2. [Prerequisites](#prerequisites)  
3. [Instructions](#instructions)  
4. [ROS Topics](#ros-topics)  
5. [Code Details](#code-details)    

---

## Highlights

- Recognizes markers using their unique identifiers.  
- Tracks marker positions and aligns the camera view with detected markers.  
- Highlights detected markers on the camera feed and publishes the annotated image.  
- Rotates the robot or the camera mount to capture data from all markers.  
- Automatically halts movement after processing all markers.  

---

## Prerequisites

Ensure the following are installed:

### Software Requirements
- **Python 3.x**
- **ROS Noetic**

### Python Libraries
- `numpy`
- `scipy`
- `opencv-python`
- `imutils`

### Install Python Dependencies:
```bash
pip install numpy scipy imutils opencv-python
```

---

## Instructions

1. Clone the Aruco ROS repository into your workspace:
```bash
git clone https://github.com/CarmineD8/aruco_ros
```
2. Add the `robot_urdf` folder to your workspace.
3. Replace the `.cpp` file in the `src` directory of the cloned `aruco_ros` repository with the one provided in this package.
4. Build the workspace:
```bash
catkin_make
```
5. Launch the `fix_controller` node for rotating the robot base:
```bash
roslaunch robot_urdf assignment1_1.launch
```
6. Confirm the following topics are active:

- `/robot4/camera1/image_raw/compressed`
- `/robot4/camera1/camera_info`
- `/marker/id_number`
- `/marker/center_loc`

7. To rotate only the camera mount while keeping the robot base stationary, use this command:
```bash
roslaunch robot_urdf assignment1_2.launch
```

---

## ROS Topics

### Published Topics

`/output/image_raw/compressed`
- **Type**: `sensor_msgs/CompressedImage`
- **Purpose**: Shares the annotated camera feed with detected markers.

`/cmd_vel`
- **Type**: `geometry_msgs/Twist`
- **Purpose**: Sends velocity commands for robot navigation.

### Subscribed Topics

`/robot4/camera1/image_raw/compressed`
- **Type**: `sensor_msgs/CompressedImage`
- **Purpose**: Receives the raw camera feed for processing.

`/robot4/camera1/camera_info`
- **Type**: `sensor_msgs/CameraInfo`
- **Purpose**: Retrieves camera specifications such as resolution and focal length.

`/marker/id_number`
- **Type**: `std_msgs/Int32`
- **Purpose**: Gets the ID of the currently detected marker.

`/marker/center_loc`
- **Type**: `geometry_msgs/Point`
- **Purpose**: Provides the coordinates of the detected marker's center.

---

## Code Details

### Key Functions

#### `Camera_callback(self, msg)`
- Extracts camera center details from `CameraInfo` messages.

#### `Id_callback(self, msg)`
- Updates the ID of the detected marker.

#### `Center_callback(self, msg)`
- Records the position of the marker's center and stores it in a dictionary.

#### `Controller_callback(self, msg)`
Implements:
  - Marker recognition and camera alignment.
  - Strategies for rotation to capture all markers.
  - Annotating the camera feed for visual feedback.

#### `main()`
- Sets up the ROS node and starts the `fix_controller`.

---

