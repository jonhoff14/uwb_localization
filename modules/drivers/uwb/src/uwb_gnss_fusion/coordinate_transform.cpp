// IndoTraq coordinate transformation
// Jonathan Hoff
// 2018-06-29

#include "uwb_gnss_fusion/coordinate_transform.h"


UTMCoordinate::UTMCoordinate(){}

UTMCoordinate::~UTMCoordinate(){}

UTMCoordinate::UTMCoordinate(double x, double y, double z)
    : _x(x), _y(y), _z(z)
{
    this->_pose << _x, _y, _z;
}

Eigen::Vector3d UTMCoordinate::get_pose() {return this->_pose;}

double UTMCoordinate::get_x() {return this->_x;}
double UTMCoordinate::get_y() {return this->_y;}
double UTMCoordinate::get_z() {return this->_z;}

float UTMCoordinate::get_x_stddev() {return this->_x_stddev;}
float UTMCoordinate::get_y_stddev() {return this->_y_stddev;}
float UTMCoordinate::get_z_stddev() {return this->_z_stddev;}

void UTMCoordinate::set_pose(double x, double y, double z)
{
    this->_x = x;
    this->_y = y;
    this->_z = z;
    this->_pose << _x, _y, _z;
}

void UTMCoordinate::set_stddev(float x_stddev, float y_stddev, float z_stddev)
{
    this->_x_stddev = x_stddev;
    this->_y_stddev = y_stddev;
    this->_z_stddev = z_stddev;
}


void RotateZ(float angle, Eigen::Matrix3d& Rotz)
{
    Rotz << cos(angle), -sin(angle), 0.0,
            sin(angle),  cos(angle), 0.0,
            0.0,         0.0,        1.0;

}

// Transform UWB Indotraq tag (in coordinate frame anchor0) to the GNSS UTM coordinate frame
// a: anchor frame
// u: UTM frame
// tag_uwb: tag position w.r.t. anchor 0 (a0)
// a0_utm: a0 position w.r.t. UTM coordinate system
// theta: z angle of UWB frame w.r.t. UTM frame
Eigen::Vector3d trans_tag2utm(Eigen::Vector3d tag_uwb, Eigen::Vector3d a0_utm, float angle)
{
    Eigen::Vector3d tag_utm;
    Eigen::Matrix3d Rotz;
    RotateZ(angle, Rotz);
    tag_utm = a0_utm + Rotz*tag_uwb;

    return tag_utm;
}

float angle_uwb2utm(Eigen::Vector3d a0_utm, Eigen::Vector3d a1_utm)
{
    double dx = a1_utm(0)-a0_utm(0);
    double dy = a1_utm(1)-a0_utm(1);
    float angle = atan2(dy,dx);
    return angle;
}