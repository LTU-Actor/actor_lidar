# actor_lidar

## Overview

ROS package designed for processing 2D Lidar data. It includes functionality for smoothing laser scan data and detecting the closest object within a specified field of view (FOV).

## Features

- **Dynamic Reconfiguration**: Adjust parameters like center offset, FOV, and range limits on the fly
- **Smoothed Laser Scan**: Optionally publishes a smoothed version of the incoming laser scan data
- **Closest Object Detection**: Identifies and publishes the distance to the closest object within the specified FOV

## Dependencies

This package depends on the following ROS packages:

- ROS Noetic on Ubuntu 20.04
- [Hokuyo URG Node](https://wiki.ros.org/urg_node)
- Compatible LiDAR (Tested with Hokuyo URG-04LX-UG01 and UTM-30LX)

## Installation

Clone the repository into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone <repository-url> actor_lidar
```

Make sure the Python file is executable:

```bash
chmod +x ~/catkin_ws/src/actor_lidar/scripts/Lidar2D_node.py
```

Build the package:

```bash
cd ~/catkin_ws
catkin build actor_lidar
source devel/setup.bash
```

## Usage

Launch the Lidar processing node using the provided launch file:

```bash
roslaunch actor_lidar Lidar2D.launch
```

### Launch Parameters

- `serial_port` (default: /dev/ttyACM0): Serial port of the Lidar
- `frame_id` (default: laser): Frame ID for the Lidar data
- `output_smoothed_scan` (default: False): Whether to publish the smoothed scan data

### Dynamic Reconfigure Parameters

- `center_offset` Offset (degrees) direction from the front center
- `field_of_view` Field of view (degrees) used for closest object detection
- `min_range` Minimum range (m) for detection
- `max_range` Maximum range (m) for detection

### Lidar2D_node

#### Subscribed Topics

- `scan` (sensor_msgs/LaserScan): The incoming laser scan data. (in the same namespace as the node)

#### Published Topics

- `closest_object` (std_msgs/Float64): The distance to the closest detected object within the FOV
- `smoothed_scan` (sensor_msgs/LaserScan, optional): The smoothed laser scan data

## Author

Devson Butani - Package Updated: 2024

For older versions, checkout an older branch
