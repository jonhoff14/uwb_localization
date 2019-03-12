// IndoTraq coordinate transformation
// Jonathan Hoff
// 2018-06-29

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>

class UTMCoordinate
{
    public:
        UTMCoordinate();

        ~UTMCoordinate();

        UTMCoordinate(double x, double y, double z);

        Eigen::Vector3d get_pose();
        double get_x();
        double get_y();
        double get_z();
        float get_x_stddev();
        float get_y_stddev();
        float get_z_stddev();
        void set_pose(double x, double y, double z);

        void set_stddev(float x_stddev, float y_stddev, float z_stddev);


    private:
        double _x; // UTM easting
        double _y; // UTM northing
        double _z; // UTM height
        float _x_stddev;
        float _y_stddev;
        float _z_stddev;
        Eigen::Vector3d _pose;
};

class LatLongCoordinate
{
    public:
        LatLongCoordinate(double lat, double lon, double alt)
            : _lat(lat), _lon(lon), _alt(alt)
        {
        }
    private:
        double _lat; // latitude
        double _lon; // longitude
        double _alt; // altitude
};

void RotateZ(float angle, Eigen::Matrix3d& Rotz);

Eigen::Vector3d trans_tag2utm(Eigen::Vector3d tag_uwb, Eigen::Vector3d a0_utm, float angle);

float angle_uwb2utm(Eigen::Vector3d a0_utm, Eigen::Vector3d a1_utm);
