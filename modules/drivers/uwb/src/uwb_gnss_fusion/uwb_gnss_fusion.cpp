// Fuse UWB and GNSS readings
// Jonathan Hoff
// 2018-07-03
#include <mutex>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "uwb_gnss_fusion/coordinate_transform.h"
#include "proto/config.pb.h"
#include "util/utils.h"

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"

/*
// Tests from 2018-07-11

Garage door north: 42.478181, -83.423529
UTM: 300793.45, 4705716.36

Garage door south:
LATLONG: 42.478149, -83.423527
UTM: 300793.51, 4705712.80

Anchor0
(300793.7929, 4705716.8172) = (300793.45, 4705716.36) + (0.3429, 0.4572)
Anchor1
(300796.3294, 4705712.80) = (300793.51, 4705712.80) + (2.8194, 0.0)
*/

/*
// Tests from 2018-07-16
GPS UTM recorded position (garage5 test): 300789.576981 4705712.51248 235.555563478
Anchor0 offset from recorded GPS position: (24'5", 0') --> (7.4422, 0) m
Anchor1 offset from Anchor0: (46'9", -42") --> (14.2494, -1.0668) m

Anchor0
(300797.019181, 4705712.51248) = (300789.576981, 4705712.51248) + (7.4422, 0)
Anchor1
(300811.268581, 4705711.445688) = (300789.576981, 4705712.51248) + (7.4422, 0) + (14.2494, -1.0668)
*/

/*
// Tests from 2018-07-18
GPS UTM recorded position (garage5 test): 300789.576981 4705712.51248 235.555563478
Anchor0 offset from recorded GPS position: (24'5", 0') --> (7.4422, 0) m
Anchor1 offset from Anchor0: (46'9", 0") --> (14.2494, 0) m

Anchor0
(300797.019181, 4705712.51248) = (300789.576981, 4705712.51248) + (7.4422, 0)
Anchor1
(300811.268581, 4705712.51248) = (300789.576981, 4705712.51248) + (7.4422, 0) + (14.2494, 0)
*/

// Note: NovAtel Euler angles are Z-X-Y
// (https://docs.novatel.com/OEM7/Content/SPAN_Operation/SPAN_Translations_Rotations.htm)


class UwbGnssFusion
{
    public:
        // Constructor
        UwbGnssFusion() : _tf_listener(_tf_buffer) {}; // create TF listener

        // Destructor
        ~UwbGnssFusion(){};

        void run(ros::NodeHandle& nh);
        
    private:
        void uwb_callback(const geometry_msgs::PointStamped &uwb_msg);
        void odometry_callback(const apollo::localization::Gps &odometry_msg);
        void gnssbestpose_callback(const apollo::drivers::gnss::GnssBestPose &gnssbestpose_msg);
        void corrimu_callback(const apollo::localization::Imu &imu_msg);
        void robotlocalization_callback(const nav_msgs::Odometry &localizationpose_msg);

        void set_theta(float theta);
        float get_theta();

        UTMCoordinate _tag_uwb; // tag UWB pose (in local UWB frame)
        UTMCoordinate _tag_utm; // tag UTM pose
        UTMCoordinate _gnss_utm; // GNSS UTM pose
        
        UTMCoordinate _anchor0;
        UTMCoordinate _anchor1;
        
        ros::Publisher _gnss_odometry_pub;  // republish odometry message as nav_msgs odometry for robot_localization
        ros::Publisher _gnss_bestpose_pub; // republish BESTGNSSPOS message as sensor message for robot_localization
        ros::Publisher _gnss_imu_msg_pub; // collect data from CORRIMU and odometry (INSPVA) and publish for robot_localization
        ros::Publisher _uwb_odometry_pub; // publish UWB transformed message as odometry
        ros::Publisher _localization_pose_pub; // publish localization pose from robot_localization

        nav_msgs::Odometry _uwb_odometry_msg;
        nav_msgs::Odometry _gnss_odometry_msg;
        sensor_msgs::NavSatFix _gnss_bestpose_msg;
        sensor_msgs::Imu _gnss_imu_msg;

        std::string _utm_frame = "utm";
        std::string _odom_frame = "odom"; // fixed relative to utm_frame
        std::string _base_link_frame = "base_link"; // moves relative to odom_frame
        std::string _indotraq_frame = "indotraq"; // fixed extrinsics relative to base_link_frame

        tf2_ros::Buffer _tf_buffer; // transform buffer for managing coordinate transforms
        tf2_ros::TransformListener _tf_listener; // transform listener for receiving transforms
        
        // TO DO: add tf timeout to conf file
        ros::Duration _tf_timeout = ros::Duration(0.01);

        double _uwb_x_offset;
        double _uwb_y_offset;
        double _uwb_z_offset;

        float _theta; // rotation from UWB frame to UTM frame
        //geometry_msgs::PointStamped _fused_pose; // pose from fusing UWB pose and GNSS odometry
        bool _has_raw_pose; // true if getting actual UWB data, false if getting the 3000001.0 message
        geometry_msgs::PointStamped _raw_pose; // pose from fusing UWB pose and GNSS odometry
        int _uwb_experiment_date; // date of UWB experiment, to be read in from uwb_conf.text file
        // Threshold over which localization should use UWB measurements instead of GNSS
        double _gnss_stddev_threshold; // GNSS measurement std dev threshold, to be read in from uwb_conf.text file
        double _uwb_stddev; // UWB measurement std dev estimate, to be read in from uwb_conf.text file
};

void UwbGnssFusion::set_theta(float theta) {this->_theta = theta;}
float UwbGnssFusion::get_theta() {return this->_theta;}

// Callback function for the UWB pose messages
// Get the XYZ position and convert to UTM
void UwbGnssFusion::uwb_callback(const geometry_msgs::PointStamped &uwb_msg)
{
    // Store tag position w.r.t. UWB local frame and convert to meters
    switch (_uwb_experiment_date)
    {
        case 20180716:
            // For 2018-07-16 tests, anchor0 is located at (2500,0) mm in local UWB frame
            _uwb_x_offset = 0.0; // in meters
            _uwb_y_offset = 0.0;
            _uwb_z_offset = 0.0;
            break;
        default:
            _uwb_x_offset = 0.0;
            _uwb_y_offset = 0.0;
            _uwb_z_offset = 0.0;
            break;
    }
    
    // Check if UWB message is valid: if xyz all have 3000001.0, then it is an invalid message.
    if ( pow((uwb_msg.point.x-3000001.0),2)
        + pow((uwb_msg.point.y-3000001.0),2)
        + pow((uwb_msg.point.z-3000001.0),2) < 1e-10 )
    {
        _has_raw_pose = false;
        _tag_uwb.set_pose(nan(""),nan(""),nan("")); // set to NaN
        _tag_utm.set_pose(nan(""),nan(""),nan("")); // set to NaN
    }
    // Check if UWB message is outside range, i.e. too close to the boundary to give good messages
    // NOTE: This is hard coded. TO DO: adjust this so that it considers the entire IndoTraq box,
    // not just the x coordinate.
    else if (uwb_msg.point.x < -1000)
    {
        _has_raw_pose = false;
        _tag_uwb.set_pose(nan(""),nan(""),nan("")); // set to NaN
        _tag_utm.set_pose(nan(""),nan(""),nan("")); // set to NaN
    }
    else
    {
        _has_raw_pose = true;
    
        _tag_uwb.set_pose(
            (uwb_msg.point.x-_uwb_x_offset)/1000.0,
            (uwb_msg.point.y-_uwb_y_offset)/1000.0,
            (uwb_msg.point.z-_uwb_z_offset)/1000.0); // convert mm to m

        // Convert tag from UWB local frame to UTM frame
        Eigen::Vector3d tag_utm = trans_tag2utm(_tag_uwb.get_pose(),_anchor0.get_pose(),_theta);

        // Set pose and standard deviation
        _tag_utm.set_pose(tag_utm(0),tag_utm(1),tag_utm(2));   
        _tag_utm.set_stddev(1.0,1.0,1.0);

        // Change to base_link frame
        // Frame id's
        std::string targetFrame;
        std::string sourceFrame;

        // TO DO: implement exceptions, as done in ros_filter_utilities.cpp
        ros::Time time = ros::Time(0); // ros::Time::now();

        // UTM to base_link
        tf2::Transform utm2base_trans;
        // IndoTraq to base_link transformation (from extrinsics)
        tf2::Transform base2indotraq_trans;
        
        // Get indotraq to base_link transform
        try
        {
            // u2m to base_link transformation
            targetFrame = _base_link_frame;
            sourceFrame = _utm_frame;
            tf2::fromMsg(_tf_buffer.lookupTransform(targetFrame, sourceFrame, time, _tf_timeout).transform,utm2base_trans);
            // indotraq to base_link transformation
            targetFrame = _indotraq_frame;
            sourceFrame = _base_link_frame;
            tf2::fromMsg(_tf_buffer.lookupTransform(targetFrame, sourceFrame, time, _tf_timeout).transform,base2indotraq_trans);
            
            // Make tag utm position as a tf object
            tf2::Transform tag_utm_trans(tf2::Quaternion(0,0,0,1),tf2::Vector3(
                    _tag_utm.get_x(),_tag_utm.get_y(),_tag_utm.get_z() ));
            
            // Get pose of indotraq tag w.r.t. base_link frame
            tf2::Transform tag_base_trans = utm2base_trans*tag_utm_trans;
            // Subtract indotraq to novatel extrinsics to approximate correct novatel pose
            tf2::Transform novatel_base_trans = base2indotraq_trans*tag_base_trans;
            // Convert back to UTM frame
            tf2::Transform novatel_utm_trans = utm2base_trans.inverse()*novatel_base_trans;
            
            // Translate indotraq measurements to NovAtel receiver using extrinsics
            _uwb_odometry_msg.pose.pose.position.x = novatel_utm_trans.getOrigin().x();
            _uwb_odometry_msg.pose.pose.position.y = novatel_utm_trans.getOrigin().y();
            _uwb_odometry_msg.pose.pose.position.z = novatel_utm_trans.getOrigin().z();
        }
        catch (tf2::TransformException &ex) {ROS_ERROR("%s",ex.what());}

        // Publish as odometry message
        _uwb_odometry_msg.header.stamp = ros::Time::now();
        _uwb_odometry_msg.header.frame_id = _utm_frame;
        _uwb_odometry_msg.child_frame_id = _base_link_frame;
        // _uwb_odometry_msg.pose.pose.position.x = _tag_utm.get_x();
        // _uwb_odometry_msg.pose.pose.position.y = _tag_utm.get_y();
        // _uwb_odometry_msg.pose.pose.position.z = _tag_utm.get_z();

        int row = 0;
        int col = row;
        _uwb_odometry_msg.pose.covariance[(row++)*6 + (col++)] = pow(_uwb_stddev,2);
        _uwb_odometry_msg.pose.covariance[(row++)*6 + (col++)] = pow(_uwb_stddev,2);
        _uwb_odometry_msg.pose.covariance[(row++)*6 + (col++)] = pow(_uwb_stddev,2);

        // _uwb_odometry_msg.pose.pose.orientation.x = 0.0;
        // _uwb_odometry_msg.pose.pose.orientation.y = 0.0;
        // _uwb_odometry_msg.pose.pose.orientation.z = 0.0;
        // _uwb_odometry_msg.pose.pose.orientation.w = 1.0;
        // _uwb_odometry_msg.twist.twist.linear.x = 0.0;
        // _uwb_odometry_msg.twist.twist.linear.y = 0.0;
        // _uwb_odometry_msg.twist.twist.linear.z = 0.0;
        
        _uwb_odometry_pub.publish(_uwb_odometry_msg);
    }
}

// Callback function for INSPVA (processed into odometry) messages to reformate to geometry_msgs/PointStamped
// This function gets the UTM position from Novatel's SPAN filter (GPS-INS)
void UwbGnssFusion::odometry_callback(const apollo::localization::Gps &odometry_msg)
{
    if (odometry_msg.has_localization())
    {
        // Store tag position w.r.t. UWB local frame and convert to meters
        const auto &pose = odometry_msg.localization();
        _gnss_utm.set_pose(pose.position().x(),pose.position().y(),pose.position().z());
        //ROS_INFO("GNSS XYZ: %f\t%f\t%f",pose.position().x(),pose.position().y(),pose.position().z());
        //ROS_INFO("GNSS XYZ: %f\t%f\t%f",_gnss_utm.get_x(),_gnss_utm.get_y(),_gnss_utm.get_z());
        
        // nav_msgs::Odometry odom_nav_msg;
        // odom_nav_msg.header.stamp = ros::Time::now();
        // odom_nav_msg.header.frame_id = _odom_frame;
        // odom_nav_msg.child_frame_id = _base_link_frame;
        // odom_nav_msg.pose.pose.position.x = pose.position().x();
        // odom_nav_msg.pose.pose.position.y = pose.position().y();
        // odom_nav_msg.pose.pose.position.z = pose.position().z();
        // odom_nav_msg.pose.pose.orientation.x = pose.orientation().qx();
        // odom_nav_msg.pose.pose.orientation.y = pose.orientation().qy();
        // odom_nav_msg.pose.pose.orientation.z = pose.orientation().qz();
        // odom_nav_msg.pose.pose.orientation.w = pose.orientation().qw();
        // odom_nav_msg.twist.twist.linear.x = pose.linear_velocity().x();
        // odom_nav_msg.twist.twist.linear.y = pose.linear_velocity().y();
        // odom_nav_msg.twist.twist.linear.z = pose.linear_velocity().z();
        // _gnss_odometry_pub.publish(odom_nav_msg);

        // Store message information to be published
        _gnss_odometry_msg.header.stamp = ros::Time::now();
        _gnss_odometry_msg.header.frame_id = _utm_frame;
        _gnss_odometry_msg.child_frame_id = _base_link_frame;
        _gnss_odometry_msg.pose.pose.position.x = pose.position().x();
        _gnss_odometry_msg.pose.pose.position.y = pose.position().y();
        _gnss_odometry_msg.pose.pose.position.z = pose.position().z();

        double qx = pose.orientation().qx();
        double qy = pose.orientation().qy();
        double qz = pose.orientation().qz();
        double qw = pose.orientation().qw();

        double vx = pose.linear_velocity().x();
        double vy = pose.linear_velocity().y();
        double vz = pose.linear_velocity().z();

        // Put velocity in base_link frame, as it currently is reported in UTM frame
        tf2::Transform utm2base_trans(tf2::Quaternion(qx,qy,qz,qw).inverse()); // take inverse
        tf2::Transform vel_odom_trans(tf2::Quaternion(0,0,0,1),tf2::Vector3(vx,vy,vz));
        tf2::Transform vel_base_trans = utm2base_trans*vel_odom_trans;

        _gnss_odometry_msg.pose.pose.orientation.x = qx;
        _gnss_odometry_msg.pose.pose.orientation.y = qy;
        _gnss_odometry_msg.pose.pose.orientation.z = qz;
        _gnss_odometry_msg.pose.pose.orientation.w = qw;
        
        _gnss_odometry_msg.twist.twist.linear.x = vel_base_trans.getOrigin().x();
        _gnss_odometry_msg.twist.twist.linear.y = vel_base_trans.getOrigin().y();
        _gnss_odometry_msg.twist.twist.linear.z = vel_base_trans.getOrigin().z();
    }
}

// Calback function for BESTGNSSPOS messages
// This function gets the standard deviation of the GNSS
void UwbGnssFusion::gnssbestpose_callback(const apollo::drivers::gnss::GnssBestPose &gnssbestpose_msg)
{
    /* 
    Note: though these don't exactly correspond to UTM standard deviations,
    they are in meters, and latitude aligns with Easting direction and longitude
    aligns with northing direction.
    */
    if (gnssbestpose_msg.has_latitude_std_dev())
    {
        float x_stddev = gnssbestpose_msg.latitude_std_dev();
        float y_stddev = gnssbestpose_msg.longitude_std_dev();
        float z_stddev = gnssbestpose_msg.height_std_dev();
        _gnss_utm.set_stddev(x_stddev,y_stddev,z_stddev);
    }

    _gnss_bestpose_msg.header.stamp = ros::Time::now();
    _gnss_bestpose_msg.header.frame_id = _base_link_frame;
    _gnss_bestpose_msg.latitude = gnssbestpose_msg.latitude();
    _gnss_bestpose_msg.longitude = gnssbestpose_msg.longitude();
    _gnss_bestpose_msg.altitude = gnssbestpose_msg.height_msl();
    _gnss_bestpose_msg.position_covariance[0] = pow(gnssbestpose_msg.latitude_std_dev(),2);
    _gnss_bestpose_msg.position_covariance[4] = pow(gnssbestpose_msg.longitude_std_dev(),2);
    _gnss_bestpose_msg.position_covariance[8] = pow(gnssbestpose_msg.height_std_dev(),2);
    _gnss_bestpose_msg.position_covariance_type = 2;
    
    // Publish gnss odometry message (includes covariance from BESTGNSSPOS)
    if(pow(gnssbestpose_msg.latitude_std_dev(),2) > pow(_gnss_stddev_threshold,2))
    {
        _gnss_bestpose_msg.latitude = nan("");
        _gnss_bestpose_msg.longitude = nan("");
        _gnss_bestpose_msg.altitude = nan("");
    }
    // Publish message
    _gnss_bestpose_pub.publish(_gnss_bestpose_msg);

    // Add covariance to odometry message
    int row = 0;
    int col = row;
    _gnss_odometry_msg.pose.covariance[(row++)*6 + (col++)] = pow(gnssbestpose_msg.latitude_std_dev(),2);
    _gnss_odometry_msg.pose.covariance[(row++)*6 + (col++)] = pow(gnssbestpose_msg.longitude_std_dev(),2);
    _gnss_odometry_msg.pose.covariance[(row++)*6 + (col++)] = pow(gnssbestpose_msg.height_std_dev(),2);
}

// Calback function for CORRIMUDATA messages
void UwbGnssFusion::corrimu_callback(const apollo::localization::Imu &imu_msg)
{
    _gnss_imu_msg.header.stamp = ros::Time::now();
    _gnss_imu_msg.header.frame_id = _base_link_frame;
    _gnss_imu_msg.linear_acceleration.x = imu_msg.imu().linear_acceleration().x();
    _gnss_imu_msg.linear_acceleration.y = imu_msg.imu().linear_acceleration().y();
    _gnss_imu_msg.linear_acceleration.z = imu_msg.imu().linear_acceleration().z();
    
    _gnss_imu_msg.angular_velocity.x = imu_msg.imu().angular_velocity().x();
    _gnss_imu_msg.angular_velocity.y = imu_msg.imu().angular_velocity().y();
    _gnss_imu_msg.angular_velocity.z = imu_msg.imu().angular_velocity().z();

    // Euler angle convention: ZXY
    // Convert Euler angles to Quaternion
    Eigen::Quaterniond q = 
        Eigen::AngleAxisd(imu_msg.imu().euler_angles().z(), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(imu_msg.imu().euler_angles().x(), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(imu_msg.imu().euler_angles().y(), Eigen::Vector3d::UnitY());

    // Update IMU orientation
    _gnss_imu_msg.orientation.x = q.x();
    _gnss_imu_msg.orientation.y = q.y();
    _gnss_imu_msg.orientation.z = q.z();
    _gnss_imu_msg.orientation.w = q.w();
}

void UwbGnssFusion::robotlocalization_callback(const nav_msgs::Odometry &localizationpose_msg)
{
    boost::shared_ptr<::apollo::localization::LocalizationEstimate> localization(
        new ::apollo::localization::LocalizationEstimate() );

    // Transform to utm frame for Dreamview

    // Frame id's
    std::string targetFrame;
    std::string sourceFrame;

    // TO DO: implement exceptions, as done in ros_filter_utilities.cpp
    ros::Time time = ros::Time(0); // ros::Time::now();

    // base_link to utm transformation
    tf2::Transform base2utm_trans;
    
    // Get odom to utm transform
    try
    {
        // base_link to utm transformation
        targetFrame = _utm_frame;
        sourceFrame = _base_link_frame;
        tf2::fromMsg(_tf_buffer.lookupTransform(targetFrame, sourceFrame, time, _tf_timeout).transform,base2utm_trans);
        //printf("X: %.2f Y: %.2f\n",base2utm_trans.getOrigin().x(),base2utm_trans.getOrigin().y());

        localization->mutable_pose()->mutable_position()->set_x(base2utm_trans.getOrigin().x());
        localization->mutable_pose()->mutable_position()->set_y(base2utm_trans.getOrigin().y());
        localization->mutable_pose()->mutable_position()->set_z(base2utm_trans.getOrigin().z());
    }
    catch (tf2::TransformException &ex) {ROS_ERROR("%s",ex.what());}

    
    double qx = localizationpose_msg.pose.pose.orientation.x;
    double qy = localizationpose_msg.pose.pose.orientation.y;
    double qz = localizationpose_msg.pose.pose.orientation.z;
    double qw = localizationpose_msg.pose.pose.orientation.w;

    // Set fields from nav_msgs Odometry message from robot_localization
    localization->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    // localization->mutable_pose()->mutable_position()->set_x(localizationpose_msg.pose.pose.position.x);
    // localization->mutable_pose()->mutable_position()->set_y(localizationpose_msg.pose.pose.position.y);
    // localization->mutable_pose()->mutable_position()->set_z(localizationpose_msg.pose.pose.position.z);
    localization->mutable_pose()->mutable_orientation()->set_qx(localizationpose_msg.pose.pose.orientation.x);
    localization->mutable_pose()->mutable_orientation()->set_qy(localizationpose_msg.pose.pose.orientation.y);
    localization->mutable_pose()->mutable_orientation()->set_qz(localizationpose_msg.pose.pose.orientation.z);
    localization->mutable_pose()->mutable_orientation()->set_qw(localizationpose_msg.pose.pose.orientation.w);
    // // odom_nav_msg.pose.covariance = pose.covariance();
    localization->mutable_pose()->mutable_linear_velocity()->set_x(localizationpose_msg.twist.twist.linear.x);
    localization->mutable_pose()->mutable_linear_velocity()->set_y(localizationpose_msg.twist.twist.linear.y);
    localization->mutable_pose()->mutable_linear_velocity()->set_z(localizationpose_msg.twist.twist.linear.z);

    // Set fields not present in robot_localization
    // TO DO: this is the function that should be called, but having compiling issues.
    // For now, we use copy/pasted code from the proper files:
    // double heading = apollo::common::math::QuaternionToHeading(
    //     localization->pose().orientation().qw(), localization->pose().orientation().qx(),
    //     localization->pose().orientation().qy(), localization->pose().orientation().qz());
    
    // ****************************
    // From modules/common/math/quaternion.h: QuaternionToHeading()

    // From modules/common/math/euler_angles_zxy.h: euler_angles_zxy()
    //double roll = std::atan2(2.0 * (qw * qy - qx * qz), 2.0 * (qw*qw + qz*qz) - 1.0);
    //double pitch = std::asin(2.0 * (qw * qx + qy * qz));
    double yaw = std::atan2(2.0 * (qw * qz - qx * qy), 2.0 * (qw*qw + qy*qy) - 1.0);

    // From modules/common/math/math_utils.cc: NormalizeAngle()
    double angle = yaw + M_PI_2;
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    double heading = a - M_PI;
    // ****************************

    localization->mutable_pose()->set_heading(heading);

    _localization_pose_pub.publish(localization);
}

// Main function for running ROS node processes
void UwbGnssFusion::run(ros::NodeHandle& nh)
{
    // Get parameters from configuration file
    std::string cfg_file;
    nh.param("uwb_conf", cfg_file, std::string("./conf/uwb_conf.txt"));
    apollo::drivers::uwb::config::Config config;
    if (!apollo::drivers::uwb::parse_config_text(cfg_file, &config))
    {
        ROS_FATAL_STREAM("Failed to load config file: " << cfg_file);
        return;
    }
    
    // Read in date of experiment from uwb_conf.txt file
    _uwb_experiment_date = config.uwb_experiment_date();
    // Read in standard deviation threshold of GNSS measurements from uwb_conf.txt file
    _gnss_stddev_threshold = config.gnss_stddev_threshold();
    // Read in standard deviation estimate of UWB measurements from uwb_conf.txt file
    _uwb_stddev = config.uwb_stddev();

    // Global UTM positions of anchors
    _anchor0.set_pose(config.a0_x(),config.a0_y(),config.a0_z());
    _anchor1.set_pose(config.a1_x(),config.a1_y(),config.a1_z());

    // Find angle to transform UWB local frame to UTM global frame
    float theta = angle_uwb2utm(_anchor0.get_pose(), _anchor1.get_pose());
    // Save angle in class
    UwbGnssFusion::set_theta(theta);
    ROS_INFO("theta (UWB to UTM frame transformation: %f\n",theta);

    // Subscribers
    ros::Subscriber uwb_raw_pose = nh.subscribe(
        "/apollo/sensor/uwb/raw_pose",100,&UwbGnssFusion::uwb_callback,this);
    ros::Subscriber odometry = nh.subscribe(
        "/apollo/sensor/gnss/odometry",100,&UwbGnssFusion::odometry_callback,this);
    ros::Subscriber gnssbestpose = nh.subscribe(
        "/apollo/sensor/gnss/best_pose",100,&UwbGnssFusion::gnssbestpose_callback,this);
    ros::Subscriber corrimu = nh.subscribe(
        "/apollo/sensor/gnss/corrected_imu",100,&UwbGnssFusion::corrimu_callback,this);
    ros::Subscriber localizationpose = nh.subscribe(
        "/robot_localization/odometry/filtered",100,&UwbGnssFusion::robotlocalization_callback,this);
    
    // Publishers
    //ros::Publisher uwb_fused_pose = nh.advertise<geometry_msgs::PointStamped>("/apollo/sensor/uwb/fused_pose", 100);
    _uwb_odometry_pub = nh.advertise<nav_msgs::Odometry>("/apollo/sensor/uwb/uwb_odometry", 100);
    _gnss_odometry_pub = nh.advertise<nav_msgs::Odometry>("/apollo/sensor/uwb/gnss_odometry", 100);
    _gnss_bestpose_pub = nh.advertise<sensor_msgs::NavSatFix>("/apollo/sensor/uwb/gnss_navsatfix", 100);
    _gnss_imu_msg_pub = nh.advertise<sensor_msgs::Imu>("/apollo/sensor/uwb/gnss_imu", 100);
    _localization_pose_pub = nh.advertise<apollo::localization::LocalizationEstimate>("/apollo/localization/pose", 100);
    
    // Run at 100 Hz (same rate as odometry)
    ros::Rate loop_rate(100);


    while (ros::ok())
    {
        // Publish gnss odometry message (includes covariance from BESTGNSSPOS)
        if(_gnss_odometry_msg.pose.covariance[0] > pow(_gnss_stddev_threshold,2))
        {
            _gnss_odometry_msg.pose.pose.position.x = nan("");
            _gnss_odometry_msg.pose.pose.position.y = nan("");
            _gnss_odometry_msg.pose.pose.position.z = nan("");
        }
        
        _gnss_odometry_pub.publish(_gnss_odometry_msg);
        _gnss_imu_msg_pub.publish(_gnss_imu_msg);
        
        /*
        // Check variance of data
        //float stddev_array[] = {_gnss_utm.get_x_stddev(),_gnss_utm.get_y_stddev(),_gnss_utm.get_z_stddev()};
        float stddev_array[] = {_gnss_utm.get_x_stddev(),_gnss_utm.get_y_stddev()}; // only x and y stddev
        float max_stddev = *std::max_element(stddev_array,stddev_array+3);
        //ROS_INFO("max_stddev: %f\n",max_stddev);
        if (max_stddev > _gnss_stddev_threshold && _has_raw_pose)
        {
            // Use IndoTraq localization if GNSS has too much variance
            _fused_pose.header.stamp = ros::Time::now();
            _fused_pose.header.frame_id = "UTM UWB";
            _fused_pose.point.x = _tag_utm.get_x();
            _fused_pose.point.y = _tag_utm.get_y();
            _fused_pose.point.z = _tag_utm.get_z();
        }
        else
        {
            _fused_pose.header.stamp = ros::Time::now();
            _fused_pose.header.frame_id = "UTM GNSS";
            _fused_pose.point.x = _gnss_utm.get_x();
            _fused_pose.point.y = _gnss_utm.get_y();
            _fused_pose.point.z = _gnss_utm.get_z();
        }
        
        // Publish fused pose data
        uwb_fused_pose.publish(_fused_pose);
        */

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // Initialization
    ros::init(argc, argv, "uwb_gnss_fusion");

    // Node for communication with ROS environment
    ros::NodeHandle nh;

    // Call class constructor
    UwbGnssFusion Ugf;
    // Call start function to begin publishing and subscribing
    Ugf.run(nh);
    
    return 0;
}


/* Needed localization pose fields
header
    timestamp_sec: 1531951021.3
    module_name: "localization"
    sequence_num: 35743
position
    x: 300806.41313
    y: 4705713.19169
    z: 235.527608066
orientation
    qx: 0.0113223441118
    qy: -0.00327490007516
    qz: -0.143084950425
    qw: 0.989640225797
linear_velocity
    x: 0.146967299093
    y: 0.467684277964
    z: -0.0217218837943
linear_acceleration
    x: -0.120762402362
    y: -0.25687886744
    z: 0.214537149086
angular_velocity
    x: -0.0080111937551
    y: -0.00374431249513
    z: 0.0988951103194
heading: 1.28365861802
linear_acceleration_vrf
    x: -0.0423510156182
    y: -0.275477421165
    z: 0.22116753967
angular_velocity_vrf
    x: -0.00630170281262
    y: -0.00354932252866
    z: 0.099025919828
euler_angles
    x: -0.00324272120551
    y: 0.0233493937975
    z: -0.287137708774
*/