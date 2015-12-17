
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Local
#include "test_camera_senz3d/ImageConverter.h"

//*****************************************************************************


ImageConverter::ImageConverter(const std::string &input_depth_topic, const std::string &input_color_topic, const std::string &output_topic)
    : it_(nh_), fps_(30.0)
{
    // Subscrive to input video feed and publish output video feed
    image_depth_sub_ = it_.subscribe(input_depth_topic, 1,
                                     &ImageConverter::imageDepthCallback, this);
    image_color_sub_ = it_.subscribe(input_color_topic, 1,
                                     &ImageConverter::imageColorCallback, this);

    image_pub_ = it_.advertise(output_topic, 1);

    cv::namedWindow("Depth");
    cv::namedWindow("Color");
}

//*****************************************************************************

ImageConverter::~ImageConverter()
{
    cv::destroyWindow("Depth");
    cv::destroyWindow("Color");
}

//*****************************************************************************

void ImageConverter::imageDepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::imshow("Depth", cv_ptr->image);
    cv::waitKey(1000.0 / fps_);

    // Output modified video stream
//    image_pub_.publish(cv_ptr->toImageMsg());
}

//*****************************************************************************

void ImageConverter::imageColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::imshow("Color", cv_ptr->image);
    cv::waitKey(1000.0 / fps_);

    // Output modified video stream
//    image_pub_.publish(cv_ptr->toImageMsg());
}

//*****************************************************************************
