# Realsense Camera

This repository is designed to be a ROS Wrapper for the Intel Realsense D435i camera

## Camera Scripts

### Camera:

```bash
ros2 run realsense_camera camera
```

#### Script: realsense_publisher.py

#### Published Topics:

/camera/Image_raw
/depth/PointCloud2_raw
/depth/depth_raw

#### Published Messages:

sensor_msgs/Image
sensor_msgs/PointCloud2
std_msgs/UInt16MultiArray

#### Description:

This script is designed to publish all aspects which you want from the camera.  

It implements multi-threading to give each publisher its own process while sharing the same pipeline for the camera

### Image Server:

```bash
ros2 run realsense_camera image_server
```

#### Script: image_server.py

#### Subscribed Topics:

/camera/Image_raw

#### Subscribed Messages:

sensor_msgs/Image

#### Description:

This script implements a UDP server for 2 devices to send live image_feedback from the camera to the ground station.

It must be used with the image_client executable to work properly.

### Image Client:

```bash
ros2 run realsense_camera image_client
```

#### Script: camera_client.py

#### Published Messages:

sensor_msgs/Image

#### Arguments:

Topic: Name of the topic you want the image to be published on - Default is /camera1

Port: Name of the socket you want to connect to - Default is 12345

```bash
ros2 run realsense_camera image_client --ros-args -p topic:='/camera2' -p port:=12346
```

#### Description:

This script is the other half of image_server which publishes the image topic to the ground station for viewing

### Depth Server

```bash
ros2 run realsense_camera depth_server
```

#### Scripts: depth_server.py

#### Subscribed Topics:

/depth/depth_raw

#### Subscribed Messages:

std_msgs/UInt16MultiArray

#### Description:

This script implements a UDP server for 2 devices to send live pointcloud_feedback from the camera to the ground station.

It must be used with the depth_client executable to work properly

### Depth Client

```bash
ros2 run realsense_camera depth_client
```

#### Script: depth_client.py

#### Published Topics:

/depth/PointCloud2

#### Published Messages:

sensor_msgs/PointCloud2

#### Description:

This script is the other half of depth_server which publishes the PointCloud2 topic to the ground station for viewing
