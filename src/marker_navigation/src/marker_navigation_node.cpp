
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

// AR_TRACK_ALVAR
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar/Camera.h>

//*****************************************************************************************************************
// Global Variables and Methods

ros::Subscriber ar_pose_markers_sub_;
ar_track_alvar_msgs::AlvarMarkers::_markers_type markers_;

int rate(10); // in Hz

bool VERBOSE = false;
ros::Subscriber verbose_image_sub_;
ros::Publisher verbose_image_pub_;
alvar::Camera * verbose_camera_;

void arPoseMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers &poseMarkers);
void verbose_imageCallback(const sensor_msgs::ImageConstPtr &msg);


//*****************************************************************************************************************
//*****************************************************************************************************************

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "marker_navigation_node");

    ROS_INFO("Start marker_navigation_node");

    ros::NodeHandle nh, pn("~");

    // setup verbose:
    pn.param<bool>("verbose", VERBOSE, false);

    // setup image topic if verbose is true
    if (VERBOSE)
    {
        std::string verbose_camera_info;
        pn.getParam("verbose_camera_info_topic", verbose_camera_info);
        verbose_camera_ = new alvar::Camera(nh, verbose_camera_info);

        // Give tf a chance to catch up before the camera callback starts asking for transforms
        // It will also reconfigure parameters for the first time, setting the default values
        ros::Duration(1.0).sleep();
        ros::spinOnce();


        std::string verbose_image_topic;
        pn.getParam("verbose_input_image_topic", verbose_image_topic);
        ROS_INFO_STREAM("VERBOSE : Subscribing to verbose input image topic '" << verbose_image_topic << "'");
        verbose_image_sub_ = nh.subscribe(verbose_image_topic, 1, &verbose_imageCallback);

        pn.getParam("verbose_output_image_topic", verbose_image_topic);
        ROS_INFO_STREAM("VERBOSE : Advertise to verbose output image topic '" << verbose_image_topic << "'");
        verbose_image_pub_ = nh.advertise<sensor_msgs::Image>(verbose_image_topic, 1);

    }



    // Subcribe to ar_track_alvar pose markers topic
    if (!pn.hasParam("pose_markers"))
    {
        ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'pose_markers' is missing.");
        ros::shutdown();
        return 1;
    }

    std::string pose_markers_topic;
    pn.getParam("pose_markers", pose_markers_topic);
    ROS_INFO_STREAM("Subscribing to '" << pose_markers_topic << "' topic");
    ar_pose_markers_sub_ = nh.subscribe(pose_markers_topic, 0, &arPoseMarkersCallback);


    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;

}

//*****************************************************************************************************************
//*****************************************************************************************************************

void arPoseMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers & poseMarkers)
{

    markers_ = poseMarkers.markers;

    // This method is called even if no markers detected:
    if (poseMarkers.markers.size() == 0) return;

    if (VERBOSE) printf("VERBOSE : Detected %d markers \n", (int) poseMarkers.markers.size());

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

        double roll, pitch, yaw;
        tf::Matrix3x3 m(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
        m.getRPY(roll, pitch, yaw);

        if (VERBOSE) printf("VERBOSE : Marker : id=%d, dist=%f, roll=%f, pitch=%f, yaw=%f | position=(%f, %f, %f)\n",
                            (int) marker.id,
                            dist,
                            180.0*roll/M_PI,
                            180*pitch/M_PI,
                            180*yaw/M_PI,
                            pose.position.x,
                            pose.position.y,
                            pose.position.z);



        const geometry_msgs::Pose::_position_type & position = pose.position;
        double pitch_angle = pitch;
        {
            // Compute camera position in 2D Marker Coordinate System
            // 2D Marker Coordinate System is defined as
            // - zero is in the marker center,
            // - OX axis is normal to the marker surface
            // - OY axis is orthogonal to the OX, belong to the marker surface and is directed 'horizontally'

            // Camera position in 2D Marker Coordinate System is denoted as x_cam, y_cam
            // and is computed using {position.x, position.z, pitch_angle}
            //      x_cam = dist2D*cos(alpha)
            //      y_cam = dist2D*sin(alpha)
            //      dist2D = sqrt(position.x**2 + position.z**2)
            // where alpha is computed as
            //      alpha = beta + pitch_angle, if pitch_angle >= 0
            //      alpha = beta - pitch_angle, if pitch_angle < 0
            // with beta = arctan(position.x, position.z)
            //
            // Camera orientation in 2D Marker Coordinate System (nx, ny) is computed as
            //      nx = -sin(gamma), ny = cos(gamma)
            //  which is obtained from the following taking position = (0, 1)
            //      (x_n - x_cam) = R_matx(gamma) * (position.x)
            //      (y_n - y_cam)                   (position.z)

            double x_cam, y_cam, alpha, beta, dist2D;

            beta = atan2(position.x, position.z);
            alpha = pitch_angle - beta;

            dist2D = sqrt(position.x*position.x + position.z*position.z);

            x_cam = dist2D * cos(alpha);
            y_cam = dist2D * sin(alpha);

            if (VERBOSE) printf("VERBOSE : dist2D=%f, x=%f, z=%f, alpha=%f, beta=%f\n",
                                dist2D,
                                position.x,
                                position.z,
                                180.0*alpha/M_PI,
                                180.0*beta/M_PI
                                );

            double nx, ny, gamma;
            gamma = M_PI_2 + pitch_angle;
            nx = -sin(gamma);
            ny = cos(gamma);

            if (VERBOSE) printf("VERBOSE : Camera pos/orient 2DMCS meters: X=%f, Y=%f | n=(%f, %f)\n",
                                x_cam,
                                y_cam,
                                nx, ny
                                );




        }

    }


}

//*****************************************************************************************************************

void verbose_imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat & img = cv_ptr->image;

    if (verbose_camera_->getCamInfo_)
    {
        if (!markers_.empty())
        {
            alvar::Pose pose;
            for (int i=0; i<markers_.size(); i++)
            {
                const geometry_msgs::PoseStamped::_pose_type & marker_pose = markers_[i].pose.pose;

                pose.SetTranslation(marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
                double quat[4] = {marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w};
                pose.SetQuaternion(quat);



                CvPoint2D64f imagePoint;
                verbose_camera_->ProjectPoint(cvPoint3D64f(0, 0, 0), &pose, imagePoint);

                //if (VERBOSE) printf("verbose_imageCallback : Image point x=%f, y=%f\n", imagePoint.x, imagePoint.y);

                int rad = 5;
                cv::ellipse(img, cv::Point(imagePoint.x, imagePoint.y), cv::Size(rad, rad), 0.0, 0.0, 360.0, cv::Scalar(255,0,0));
            }
        }

    }
    else
    {
        if (VERBOSE) printf("verbose_imageCallback : Camera Info is not properly set\n");
    }



    // Output modified video stream
    verbose_image_pub_.publish(cv_ptr->toImageMsg());
}

//*****************************************************************************************************************

//void camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
//{
//    if (VERBOSE) {
//        printf("VERBOSE : camInfoCallback : ");

//        printf("VERBOSE : cam_info.K = %f, %f, %f, %f ... %f \n",
//               cam_info->K[0],
//                cam_info->K[1],
//                cam_info->K[2],
//                cam_info->K[3],
//                cam_info->K[8]);

//        if (cam_info->D.size() >= 4) {
//            printf("VERBOSE : cam_info.D = %f, %f, %f, %f \n",
//                   cam_info->D[0],
//                    cam_info->D[1],
//                    cam_info->D[2],
//                    cam_info->D[3]);
//        }

//    }
//}

//*****************************************************************************************************************
// if (VERBOSE)
//                {
//                    double ext_rodriques[3];
//                    double ext_translate[3];
//                    CvMat ext_rodriques_mat = cvMat(3, 1, CV_64F, ext_rodriques);
//                    CvMat ext_translate_mat = cvMat(3, 1, CV_64F, ext_translate);
//                    pose.GetRodriques(&ext_rodriques_mat);
//                    pose.GetTranslation(&ext_translate_mat);

//                    printf("verbose_imageCallback : translation mat : %f, %f, %f \n",
//                           cvmGet(&ext_translate_mat, 0, 0),
//                           cvmGet(&ext_translate_mat, 1, 0),
//                           cvmGet(&ext_translate_mat, 2, 0));
//                    printf("verbose_imageCallback : rotation mat : %f, %f, %f \n",
//                           cvmGet(&ext_rodriques_mat, 0, 0),
//                           cvmGet(&ext_rodriques_mat, 1, 0),
//                           cvmGet(&ext_rodriques_mat, 2, 0));
//                    float object_points_data[3] = {0.0f, 0.0f, 0.0f};
//                    CvMat object_points = cvMat(1, 1, CV_32FC3, object_points_data);
//                    float image_points_data[2] = {0};
//                    CvMat image_points = cvMat(1, 1, CV_32FC2, image_points_data);
//                    printf("verbose_imageCallback : calib_K = %f, %f, %f ... \n",
//                           cvmGet(&verbose_camera_->calib_K, 0, 0),
//                           cvmGet(&verbose_camera_->calib_K, 1, 1),
//                           cvmGet(&verbose_camera_->calib_K, 2, 2));
//                    printf("verbose_imageCallback : calib_D = %f, %f, %f, %f \n",
//                           cvmGet(&verbose_camera_->calib_D, 0, 0),
//                           cvmGet(&verbose_camera_->calib_D, 1, 0),
//                           cvmGet(&verbose_camera_->calib_D, 2, 0),
//                           cvmGet(&verbose_camera_->calib_D, 3, 0));
//                    cvProjectPoints2(&object_points, &ext_rodriques_mat, &ext_translate_mat, &verbose_camera_->calib_K, &verbose_camera_->calib_D, &image_points);
//                }

