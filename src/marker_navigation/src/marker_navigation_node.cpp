
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

// AR_TRACK_ALVAR
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar/Camera.h>

//*****************************************************************************************************************
// Global Variables and Methods


// Camera sensor position in Camera Coordinate System
tf::Vector3 sensor_position_(0, 0, 0);//(-0.0135, 0, 0);

ros::Subscriber ar_pose_markers_sub_;

tf::TransformListener *tf_listener_;

ar_track_alvar_msgs::AlvarMarkers::_markers_type markers_;
geometry_msgs::Pose pose_;
double xcorr_(0.0), ycorr_(0.0), zcorr_(0.0);

int rate(10); // in Hz

struct Pose2D {
    double x, y, theta;
    Pose2D(double px=0, double py=0, double t=12345) :
        x(px), y(py), theta(t)
    {}
};
std::vector<Pose2D> world_markers_;


bool VERBOSE = false;
ros::Subscriber verbose_image_sub_;
ros::Publisher verbose_image_pub_;
alvar::Camera * verbose_camera_(0);

void arPoseMarkersCallback_v2D(const ar_track_alvar_msgs::AlvarMarkers &poseMarkers);
void arPoseMarkersCallback_v3D(const ar_track_alvar_msgs::AlvarMarkers &poseMarkers);


void parseMarkerWorldPose2D(const std::string & pose2DStr, std::vector<Pose2D> & markers);

void displayTransform(int id, const tf::Transform & transform, const std::string & msg="");

void compute2DCameraCoords(const geometry_msgs::Pose::_position_type & position,
                           double pitch_angle,
                           double & x_cam,
                           double & y_cam,
                           double & theta);

void verbose_imageCallback(const sensor_msgs::ImageConstPtr &msg);


//*****************************************************************************************************************
//*****************************************************************************************************************

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "marker_navigation_node");

    ROS_INFO("Start marker_navigation_node");

    ros::NodeHandle nh, pn("~");

    // ---------------- setup verbose:
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
        ROS_ERROR_STREAM("Node '" << ros::this_node::getName() << "''. Parameter 'pose_markers' is missing.");
        ros::shutdown();
        return 1;
    }

    bool version_2D = false;
    if (pn.hasParam("version_2D"))
    {
        pn.getParam("version_2D", version_2D);
    }


    std::string pose_markers_topic;
    pn.getParam("pose_markers", pose_markers_topic);
    ROS_INFO_STREAM("Subscribing to '" << pose_markers_topic << "' topic");
    if (version_2D)
    {
        ar_pose_markers_sub_ = nh.subscribe(pose_markers_topic, 0, &arPoseMarkersCallback_v2D);
    }
    else
    {
        ar_pose_markers_sub_ = nh.subscribe(pose_markers_topic, 0, &arPoseMarkersCallback_v3D);
    }

    tf_listener_ = new tf::TransformListener(nh);

    if (pn.hasParam("pose_correction_x"))
    {
        pn.getParam("pose_correction_x", xcorr_);
        ROS_INFO_STREAM("Marker pose X correction : " << xcorr_ << " ");
    }

    if (pn.hasParam("pose_correction_y"))
    {
        pn.getParam("pose_correction_y", ycorr_);
        ROS_INFO_STREAM("Marker pose Y correction : " << ycorr_ << " ");
    }

    if (pn.hasParam("pose_correction_z"))
    {
        pn.getParam("pose_correction_z", zcorr_);
        ROS_INFO_STREAM("Marker pose Z correction : " << zcorr_ << " ");
    }


    if (pn.hasParam("world_markers_count"))
    {
        int count;
        std::string pose2DStr;

        pn.param<int>("world_markers_count", count, 0);
        if (count > 0) {
            ROS_INFO_STREAM("Found " << count << " world markers");
            world_markers_.resize(count);
            for (int i=0; i<count; i++)
            {
                std::stringstream field_name;
                field_name << "world_marker_pose_" << i;
                pn.getParam(field_name.str(), pose2DStr);
                parseMarkerWorldPose2D(pose2DStr, world_markers_);
            }
        }
    }


    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        // Publish camera world pose

    }


    delete tf_listener_;
    if (VERBOSE && verbose_camera_)
    {
        delete verbose_camera_;
    }

    return 0;

}

//*****************************************************************************************************************
//*****************************************************************************************************************

void arPoseMarkersCallback_v2D(const ar_track_alvar_msgs::AlvarMarkers & poseMarkers)
{

    markers_ = poseMarkers.markers;
    int size = markers_.size();

    // This method is called even if no markers detected:
    if (size == 0) return;

    if (VERBOSE) printf("VERBOSE : Detected %d markers \n", (int) poseMarkers.markers.size());

    // There is { Header, Markers } in ar_track_alvar_msgs::AlvarMarkers
    //ar_track_alvar_msgs::AlvarMarkers_::_markers_type & markers = poseMarkers->markers;
    ar_track_alvar_msgs::AlvarMarkers::_markers_type::const_iterator it = poseMarkers.markers.begin();

    double x_world(0), y_world(0), theta_world(0);

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

        // Fix marker position with global parameters:
        geometry_msgs::Pose::_position_type position = pose.position;
        position.x += xcorr_;
        position.y += ycorr_;
        position.z += zcorr_;


        // Compute camera coordinates/orientation in 2D Marker CS
        double x_cam, y_cam, theta_cam;
        compute2DCameraCoords(position, pitch, x_cam, y_cam, theta_cam);

        if (VERBOSE) printf("VERBOSE : Camera pos/orient 2DMCS[%d] meters: X=%f, Y=%f | theta=%f\n",
                            (int) marker.id,
                            x_cam,
                            y_cam,
                            theta_cam*180.0/M_PI
                            );


        // Compute camera position in world coordinates using world marker coordinates
        if (marker.id >= 0 && marker.id < world_markers_.size())
        {
            const Pose2D & wmp = world_markers_[marker.id];
            if (wmp.theta != 12345)
            {
                double x = cos(wmp.theta)*x_cam - sin(wmp.theta)*y_cam + wmp.x;
                double y = sin(wmp.theta)*x_cam + cos(wmp.theta)*y_cam + wmp.y;
                double t = theta_cam - wmp.theta;

                if (VERBOSE) printf("VERBOSE : Camera world pos/orient 2DMCS[%d] meters: X=%f, Y=%f | theta=%f\n",
                                    (int) marker.id,
                                    x,
                                    y,
                                    t*180.0/M_PI
                                    );
                x_world += x*1.0/size;
                y_world += y*1.0/size;
                theta_world += t*1.0/size;
            }
        }
    }

    if (VERBOSE) printf("VERBOSE : MEAN WORLD Camera pos/orient in meters: X=%f, Y=%f | theta=%f\n",
                        x_world,
                        y_world,
                        theta_world*180.0/M_PI
                        );


}

//*****************************************************************************************************************

void arPoseMarkersCallback_v3D(const ar_track_alvar_msgs::AlvarMarkers & poseMarkers)
{

    markers_ = poseMarkers.markers;
    int size = markers_.size();

    // This method is called even if no markers detected:
    if (size == 0) return;

    if (VERBOSE) printf("VERBOSE : Detected %d markers \n", (int) size);

    // There is { Header, Markers } in ar_track_alvar_msgs::AlvarMarkers
    //ar_track_alvar_msgs::AlvarMarkers_::_markers_type & markers = poseMarkers->markers;
    ar_track_alvar_msgs::AlvarMarkers::_markers_type::const_iterator it = poseMarkers.markers.begin();

    for (; it != poseMarkers.markers.end(); ++it)
    {
        const ar_track_alvar_msgs::AlvarMarker & marker = *it;
        const geometry_msgs::PoseStamped::_pose_type & pose = marker.pose.pose;

        // Get tf
        tf::StampedTransform transform;
        try
        {
            //tf_listener_->waitForTransform();
            const std_msgs::Header::_frame_id_type & source_frame = marker.header.frame_id;
            std::stringstream target_frame;
            target_frame << "ar_marker_" << (int) marker.id;
            tf_listener_->lookupTransform(target_frame.str(), source_frame, ros::Time(0), transform);
        }
        catch(tf::TransformException & ex)
        {
            ROS_ERROR_STREAM("%s" << ex.what());
            break;
        }

        if (VERBOSE)
        {
            displayTransform(marker.id, transform);
        }
                                

//        double dist = pose.position.x*pose.position.x;
//        dist += pose.position.y*pose.position.y;
//        dist += pose.position.z*pose.position.z;
//        dist = sqrt(dist);

//        double roll, pitch, yaw;
//        tf::Matrix3x3 m(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
//        m.getRPY(roll, pitch, yaw);

//        if (VERBOSE) printf("VERBOSE : Marker : id=%d, dist=%f, roll=%f, pitch=%f, yaw=%f | position=(%f, %f, %f)\n",
//                            (int) marker.id,
//                            dist,
//                            180.0*roll/M_PI,
//                            180*pitch/M_PI,
//                            180*yaw/M_PI,
//                            pose.position.x,
//                            pose.position.y,
//                            pose.position.z);

//        // Fix marker position with global parameters:
//        geometry_msgs::Pose::_position_type position = pose.position;
//        position.x += xcorr_;
//        position.y += ycorr_;
//        position.z += zcorr_;


        // Transform Marker CS axes :
        tf::Vector3 o(0,0,0);
        tf::Vector3 oz(0,0,1);
        tf::Vector3 oy(0,1,0);
        tf::Vector3 ox(1,0,0);

        tf::Transform itransform = transform.inverse();

        tf::Vector3 o_t = itransform * o;
        tf::Vector3 ox_t = itransform * ox - o_t;
        tf::Vector3 oy_t = itransform * oy - o_t;
        tf::Vector3 oz_t = itransform * oz - o_t;

        printf("Test Marker CS Axes : OX'=(%f, %f, %f) | OY'=(%f, %f, %f) | OZ'=(%f, %f, %f)\n",
               ox_t.x(),
               ox_t.y(),
               ox_t.z(),
               oy_t.x(),
               oy_t.y(),
               oy_t.z(),
               oz_t.x(),
               oz_t.y(),
               oz_t.z());

        // Camera position in ar_marker frame:
        tf::Pose c(tf::Quaternion::getIdentity (), sensor_position_);
        tf::Pose cam_mf = transform * c;

        if (VERBOSE) {
            tf::Vector3 & p = cam_mf.getOrigin();
            tf::Matrix3x3 & o = cam_mf.getBasis();
            double roll, pitch, yaw;
            o.getRPY(roll, pitch, yaw);
            printf("VERBOSE : Camera position in AR_MARKER %d frame : %f, %f, %f | roll=%f, pitch=%f, yaw=%f\n",
                   (int) marker.id,
                   p.x(),
                   p.y(),
                   p.z(),
                   180.0*roll/M_PI,
                   180*pitch/M_PI,
                   180*yaw/M_PI
                   );
        }




    }

    //    if (VERBOSE) printf("VERBOSE : MEAN WORLD Camera pos/orient in meters: X=%f, Y=%f, Z=%f | theta=%f\n",
    //                        x_world,
    //                        y_world,
    //                        z_world,
    //                        theta_world*180.0/M_PI
    //                        );


}

//*****************************************************************************************************************

void parseMarkerWorldPose2D(const std::string &pose2DStr, std::vector<Pose2D> &markers)
{
    if (!pose2DStr.empty())
    {
        int id = -1;
        char * str;
        pose2DStr.copy(str, pose2DStr.size());
        char* pch = strtok(str," ");

        if (pch != 0) {

            int id = atoi(pch);
            if (world_markers_.size() < id)
            {
                world_markers_.resize(id);
            }
            pch = strtok (0, " ");
            world_markers_[id].x = atof(pch);
            pch = strtok (0, " ");
            world_markers_[id].y = atof(pch);
            pch = strtok (0, " ");
            world_markers_[id].theta = M_PI*atof(pch)/180.0;

            if (VERBOSE) printf("VERBOSE : world_marker id=%d, x=%f, y=%f, theta=%f \n",
                                id,
                                world_markers_[id].x,
                                world_markers_[id].y,
                                world_markers_[id].theta*180.0/M_PI);
        }

    }
}

//*****************************************************************************************************************

void displayTransform(int id, const tf::Transform & transform, const std::string & msg)
{
    const tf::Vector3& v = transform.getOrigin();
    double roll, pitch, yaw;
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(roll, pitch, yaw);

    printf("VERBOSE : Marker %s transform : id=%d, origin=(%f,%f,%f), roll=%f, pitch=%f, yaw=%f \n",
           msg.c_str(),
           (int) id,
           v.x(),
           v.y(),
           v.z(),
           180.0*roll/M_PI,
           180.0*pitch/M_PI,
           180.0*yaw/M_PI);
}

//*****************************************************************************************************************

void compute2DCameraCoords(const geometry_msgs::Pose::_position_type & position,
                           double pitch_angle,
                           double & x_cam,
                           double & y_cam,
                           double & theta)
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
    // Camera orientation theta is the angle between z camera axis and OX 2D Marker Coordinate System.
    // It is computed as
    //      theta = 2*pi - gamma - pi/2 = pi - pitch_angle
    // where gamma = pi/2 + pitch_angle is the rotation angle between two coordinate systems

    double alpha, beta, dist2D;

    beta = atan2(position.x, position.z);
    alpha = (pitch_angle >=0) ? pitch_angle - beta : beta - pitch_angle;

    dist2D = sqrt(position.x*position.x + position.z*position.z);

    x_cam = dist2D * cos(alpha);
    y_cam = dist2D * sin(alpha);

    //    if (VERBOSE) printf("VERBOSE : dist2D=%f, x=%f, z=%f, alpha=%f, beta=%f\n",
    //                        dist2D,
    //                        position.x,
    //                        position.z,
    //                        180.0*alpha/M_PI,
    //                        180.0*beta/M_PI
    //                        );

    theta = (pitch_angle > 0) ? pitch_angle - M_PI : M_PI + pitch_angle;

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

