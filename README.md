# ROS Node: OpenCV Camera

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** TBD

Simple generic camera driver using OpenCV to grab images.

## Command Line

	rosrun camera_node opencv_cam _source:=# _size:=#x# _debug:=true/false
 
*debug: true or false
 *source: > 0 selects a camera other than default
 *size: my MacBook Pro can do 160x120, 320x240, 640x480
 *fps: my MacBook Pro seems to ignore fps and always gives me ~15 fps

### Published Topics: 
**Image:** "/opencv_cam/camera"
 
 ### Example:
 	rosrun camera_node opencv_cam _debug:=true _size:=160x120 _fps:=30

## To Do

* finalize published image 
* handle real-time setting camera parameters 
* handle reading camera parameters from a file

# ROS Node: OpenCV Camera Calibration Tool

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** TBD

Camera calibration code from OpenCV ported to ROS.

## Command Line

	rosrun camera_node calibrate
 
### Example:
 	rosrun camera_node calibrate

## To Do

* Clean-up code
* Handle saving camera parameters better


