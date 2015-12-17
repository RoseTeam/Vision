
// ROS
#include <ros/ros.h>

// Local
#include "test_camera_senz3d/ImageConverter.h"

int main(int argc, char ** argv) 
{

    ros::init(argc, argv, "test_camera_senz3d_node");
	
    ROS_INFO("Test camera Senz3D node started");

    // Initialize image converter
    ImageConverter ic("/softkinetic_camera/depth/image_raw",
                      "/softkinetic_camera/rgb/image_color",
                      "/test_camera_senz3d/image_processed");

	// Enter a loop
	ros::spin();
	
	return 0;

}
