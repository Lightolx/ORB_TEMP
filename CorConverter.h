//
// Created by lightol on 18-11-18.
//

#ifndef ORB_TEMP_CORCONVERTER_H
#define ORB_TEMP_CORCONVERTER_H

#include <iostream>
#include <eigen3/Eigen/Eigen>

#include "OpenDRIVE_1.4H.hxx"

const double degree_2_radian = 0.017453293;
const double radian_2_degree = 57.295777937;
const double WGS84_a = 6378137.0;
const double WGS84_b = 6356752.314245;
const double WGS84_f_inv = 298.257223563;

const double WGS84_a_over_b = WGS84_a / WGS84_b;
const double WGS84_b_over_a = WGS84_b / WGS84_a;
const double WGS84_e = sqrt((WGS84_a * WGS84_a - WGS84_b * WGS84_b) / (WGS84_a * WGS84_a));

static double ref_x, ref_y, ref_z;
static double ref_sinlat, ref_coslat, ref_sinlon, ref_coslon;

namespace LINE_PNP
{
class GPSdata {
public:
    GPSdata(Eigen::Vector3d lla, Eigen::Vector3d rpy) :LLA(lla), RPY(rpy) {}

public:
    Eigen::Vector3d LLA;       // lantitude, longitude, alttitude
    Eigen::Vector3d RPY;       // raw, pitch, yaw
};

struct Pose {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};

class CorConverter {
public:
    static void Convert2NED(std::shared_ptr<OpenDRIVE> pHDmap);

private:
    static void ConvertBoundary2NED(roadBoundary::geometry_type &geometry);

    static void LLA2ECEF(double lat, double lon, double alt, double& x, double& y,
                         double& z);

    static void LLA2NED(double lat, double lon, double alt, double& north, double& east,
                        double& down);

    static bool ConvertOXTS2Pose(const std::vector<GPSdata>& GPSdatas, std::vector<Pose>& poses);
};
}



#endif  // ORB_TEMP_CORCONVERTER_H
