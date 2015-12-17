
// ROS
#include <ros/ros.h>

// Local


int main(int argc, char ** argv)
{

    ros::init(argc, argv, "marker_navigation_node");

    ROS_INFO("Start marker_navigation_node");

    // Initialize image converter
//    ImageConverter ic("/softkinetic_camera/depth/image_raw",
//                      "/softkinetic_camera/rgb/image_color",
//                      "/test_camera_senz3d/image_processed");

    // Enter a loop
    ros::spin();

    return 0;

}
