# Catkin workspace with CV test packages 

Workspace source contains nodes :

### [test_package](https://github.com/vfdev-5/ros_test_package) 
Package contains two tutorial nodes (talker & listener) and a marker detection node (code taken from ar_track_alvar individual marker detection node)

### test_camera_senz3d 
Package contains a node to display and broadcast rgb and depth images with opencv highgui windows

### [softkinetic_camera](https://github.com/vfdev-5/softkinetic) 
It is a fork of ROS driver package for an interactive gesture camera [softkinetic](https://github.com/ipa320/softkinetic). This node create useful topics : rgb, depth, camera_info, etc

```
> rosnode info /softkinetic_camera
--------------------------------------------------------------------------------
Node [/softkinetic_camera]
Publications:
 * /softkinetic_camera/rgb/camera_info [sensor_msgs/CameraInfo]
 * /softkinetic_camera/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /rosout [rosgraph_msgs/Log]
 * /softkinetic_camera/rgb/image_color [sensor_msgs/Image]
 * /softkinetic_camera/rgb/image_mono [sensor_msgs/Image]
 * /softkinetic_camera/depth/camera_info [sensor_msgs/CameraInfo]
 * /softkinetic_camera/depth/points [sensor_msgs/PointCloud2]
 * /softkinetic_camera/parameter_updates [dynamic_reconfigure/Config]
 * /softkinetic_camera/depth/image_raw [sensor_msgs/Image]

Subscriptions: None

Services:
 * /softkinetic_camera/set_parameters
 * /softkinetic_camera/set_logger_level
 * /softkinetic_camera/get_loggers
```
 
### marker_navigation
Test package to work with data from ar_track_alvar and broadcast location information with respect to markers 
