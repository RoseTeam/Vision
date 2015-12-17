#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>

//*****************************************************************************

class ImageConverter
{

public:
    ImageConverter(const std::string & input_depth_topic,
                   const std::string & input_color_topic,
                   const std::string & output_topic);

    ~ImageConverter();

    void setFps(double fps) { fps_ = fps; }

    void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageColorCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_depth_sub_;
    image_transport::Subscriber image_color_sub_;
    image_transport::Publisher image_pub_;

    double fps_;


};

//*****************************************************************************

#endif // IMAGECONVERTER_H
