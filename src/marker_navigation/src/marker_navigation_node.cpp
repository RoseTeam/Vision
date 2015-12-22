
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

// AR_TRACK_ALVAR
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>


//*****************************************************************************************************************
// Global Variables and Methods

ros::Subscriber ar_pose_markers_sub_;

bool VERBOSE = false;

void arPoseMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers &poseMarkers);

//*****************************************************************************************************************
//*****************************************************************************************************************

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "marker_navigation_node");

    ROS_INFO("Start marker_navigation_node");

    ros::NodeHandle nh("~");

    // Subcribe to ar_track_alvar pose markers topic
    if (!nh.hasParam("pose_markers"))
    {
        ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'pose_markers' is missing.");
        ros::shutdown();
        return 1;
    }

    std::string pose_markers_topic;
    nh.getParam("pose_markers", pose_markers_topic);
    ROS_INFO_STREAM("Subscribing to '" << pose_markers_topic << "' topic");
    ar_pose_markers_sub_ = nh.subscribe(pose_markers_topic, 0, &arPoseMarkersCallback);


    // setup verbose:
    nh.param<bool>("verbose", VERBOSE, false);

    // Enter a loop
    ros::spin();

    return 0;

}

//*****************************************************************************************************************
//*****************************************************************************************************************

void arPoseMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers & poseMarkers)
{
    // This method is called even if no markers detected:
    if (poseMarkers.markers.size() == 0) return;

    if (VERBOSE) printf("Detected %d markers \n", (int) poseMarkers.markers.size());

    // There is { Header, Markers } in ar_track_alvar_msgs::AlvarMarkers
    //ar_track_alvar_msgs::AlvarMarkers_::_markers_type & markers = poseMarkers->markers;
    ar_track_alvar_msgs::AlvarMarkers::_markers_type::const_iterator it = poseMarkers.markers.begin();
    for (; it != poseMarkers.markers.end(); ++it)
    {
        const ar_track_alvar_msgs::AlvarMarker & marker = *it;
        const geometry_msgs::PoseStamped::_pose_type & pose = marker.pose.pose;

        double dist = pose.position.x*pose.position.x;
        dist += pose.position.y*pose.position.y;
        dist += pose.position.z*pose.position.z;
        dist = sqrt(dist);

        double nvect_x = pose.orientation.x;
        double nvect_y = pose.orientation.y;
        double nvect_z = pose.orientation.z;
        double theta = (180.0 / M_PI) * 2.0*atan(sqrt(nvect_x*nvect_x + nvect_y*nvect_y + nvect_z*nvect_z) / pose.orientation.w);

        if (VERBOSE) printf("Marker : id=%d, dist=%f, nvect=(%f, %f, %f), theta=%f\n", (int) marker.id, dist, nvect_x, nvect_y, nvect_z, theta);
    }
}

//*****************************************************************************************************************




