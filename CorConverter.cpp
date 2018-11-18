//
// Created by lightol on 18-11-18.
//

#include "CorConverter.h"

namespace LINE_PNP
{
void CorConverter::Convert2NED(std::shared_ptr<OpenDRIVE> pHDmap) {
    OpenDRIVE::road_sequence &roads = pHDmap->road();

    for (OpenDRIVE::road_type &road : roads) {
        road::lanes_type &driveway = road.lanes();

        // lanes
        lanes::laneSection_sequence &laneSections = driveway.laneSection();

        for (lanes::laneSection_type &laneSection : laneSections) {
            // 一个laneSection里面总共有3种5条线:
            // 道路可行驶区域左右边界 leftBoundary, rightBoundary；
            // 所有车道的中心线，也就是reference line,有且仅有一条；
            // 左边车道和右边车道的车道线 leftLanes, rightLanes, 比如4车道的话左右就是各两条车道线;
            laneSection::leftBoundary_type &leftBoundary = laneSection.leftBoundary();
            laneSection::rightBoundary_type &rightBoundary = laneSection.rightBoundary();
            laneSection::left_type &leftLanes = laneSection.left();
            laneSection::center_type &centerLane = laneSection.center();
            laneSection::right_type &rightLanes = laneSection.right();

            // Step1: 左右boundary做变换
            roadBoundary::geometry_type &geometryLeft = leftBoundary.geometry();
            ConvertBoundary2NED(geometryLeft);
            roadBoundary::geometry_type &geometryRight = rightBoundary.geometry();
            ConvertBoundary2NED(geometryRight);

            // Step2: 道路中心线
            roadBoundary::geometry_type &geometryCenter = centerLane.lane().border().geometry();
            ConvertBoundary2NED(geometryCenter);

            // Step3: 对左右两边的车道线
            //左边车道的车道线
            halfRoad::lane_sequence &lanesInLeft = leftLanes.lane();
            for (auto &lane : lanesInLeft) {
                roadBoundary::geometry_type &geometry1 = lane.border().geometry();
                ConvertBoundary2NED(geometry1);

                lane::leftBorder_optional &leftBorder = lane.leftBorder();
                if (leftBorder.present()) {
                    roadBoundary::geometry_type &geometry2 = leftBorder.get().geometry();
                    ConvertBoundary2NED(geometry2);
                }
            }

            //右边车道的车道线
            halfRoad::lane_sequence &lanesInRight = rightLanes.lane();
            for (auto &lane : lanesInRight) {
                roadBoundary::geometry_type &geometry1 = lane.border().geometry();
                ConvertBoundary2NED(geometry1);

                lane::leftBorder_optional &leftBorder = lane.leftBorder();
                if (leftBorder.present()) {
                    roadBoundary::geometry_type &geometry2 = leftBorder.get().geometry();
                    ConvertBoundary2NED(geometry2);
                }
            }
        }

        // Step4: 起始线
        objects::object_sequence &objects = road.objects().object();
        for (objects::object_type &object : objects) {
            if (object.type().present()) {
                if (object.type().get() == "startline" || object.type().get() == "stopline") {
                    roadBoundary::geometry_type &geometryStart = object.geometry();
                    ConvertBoundary2NED(geometryStart);
                }
            }
        }
    }

    // Step5: junction
    OpenDRIVE::junction_sequence &junctions = pHDmap->junction();
    for (OpenDRIVE::junction_type &junction : junctions) {
        boundary::geometry_type &geometryJunc = junction.boundary().geometry();
        ConvertBoundary2NED(geometryJunc);
    }
}

void CorConverter::ConvertBoundary2NED(roadBoundary::geometry_type &geometry) {
    double x(0.0), y(0.0), z(0.0);
    std::vector<Eigen::Vector3d> pts;

    geometry::point_sequence &points = geometry.point();

    for (geometry::point_type point : points) {
        x = *point.y();
        y = *point.x();
        z = *point.z();
        pts.push_back(Eigen::Vector3d(x, y, z));
    }

    int nPts = pts.size();

    std::vector<GPSdata> GPSdatas;
    GPSdatas.emplace_back(GPSdata(Eigen::Vector3d(30.24664434, 120.23206484, 8.64414310),
                                  Eigen::Vector3d(0.0, 0.0, 0.0)));

    for (int i = 0; i < nPts; ++i) {
        GPSdatas.emplace_back(GPSdata(pts[i], Eigen::Vector3d(0.0, 0.0, 0.0)));
    }

    std::vector<Pose> poses;
    ConvertOXTS2Pose(GPSdatas, poses);

    for (int i = 0; i < nPts; ++i) {
        pts[i] = poses[i+1].t;
    }

    for (int i = 0; i < pts.size(); ++i) {
        points[i].x() = pts[i].x();
        points[i].y() = pts[i].y();
        points[i].z() = pts[i].z();
    }
}

void CorConverter::LLA2ECEF(double lat, double lon, double alt, double& x, double& y, double& z) {
    double coslat = cos(lat);
    double sinlat = sin(lat);
    double coslon = cos(lon);
    double sinlon = sin(lon);
    double N = WGS84_a / sqrt(1 - WGS84_e * WGS84_e * sinlat * sinlat);
    double R = (N + alt) * coslat;
    x = R * coslon;
    y = R * sinlon;
    z = (WGS84_b_over_a * WGS84_b_over_a * N + alt) * sinlat;
}

void CorConverter::LLA2NED(double lat, double lon, double alt,
                           double& north, double& east, double& down) {
    // first convert into ECEF system
    double x, y, z;
    LLA2ECEF(lat, lon, alt, x, y, z);
    // get NED
    double dx = x - ref_x;
    double dy = y - ref_y;
    double dz = z - ref_z;
    north = -ref_sinlat * ref_coslon * dx - ref_sinlat * ref_sinlon * dy +
            ref_coslat * dz;
    east = -ref_sinlon * dx + ref_coslon * dy;
    down = -ref_coslat * ref_coslon * dx - ref_coslat * ref_sinlon * dy -
           ref_sinlat * dz;
}

bool CorConverter::ConvertOXTS2Pose(const std::vector<GPSdata>& GPSdatas,
                                    std::vector<Pose>& poses) {
    if (GPSdatas.empty()) return false;

    bool isFirst = true;
    Eigen::Matrix3d R0;
    Eigen::Vector3d t0;
    Eigen::Matrix3d Rmer2I0;  //  mercator坐标与车初始方向的变换。
    Eigen::Vector3d Lt0;
    Eigen::Matrix3d R_ned_enu;
    R_ned_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    for (auto& oxts : GPSdatas) {
        Pose pos;
        Eigen::Vector3d rpy = oxts.RPY;

        double lat_rad = oxts.LLA(0) * degree_2_radian;
        double lon_rad = oxts.LLA(1) * degree_2_radian;
        if (isFirst) {
            ref_coslat = cos(oxts.LLA(0) * degree_2_radian);
            ref_sinlat = sin(oxts.LLA(0) * degree_2_radian);
            ref_coslon = cos(oxts.LLA(1) * degree_2_radian);
            ref_sinlon = sin(oxts.LLA(1) * degree_2_radian);
            double x, y, z;
            LLA2ECEF(lat_rad, lon_rad, oxts.LLA(2), x, y, z);
            ref_x = x;
            ref_y = y;
            ref_z = z;
        }
        double north, east, down;
        LLA2NED(lat_rad, lon_rad, oxts.LLA(2), north, east, down);
        /// ned2cameraR(rpy); ///
        /// ned2LIDAR(rpy);
        pos.R = Eigen::Matrix3d(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));
        // // convert ned
        pos.t = Eigen::Vector3d(north, east, down);
        if (isFirst) {
            R0 = pos.R.inverse();
            t0 = -pos.R.inverse() * pos.t;
            isFirst = false;
            // continue;
        }
        // this pos actually is the position relative to IMU_0
        pos.R = R0 * pos.R;
        pos.t = R0 * pos.t + t0;
        poses.emplace_back(pos);
    }
    return true;
}
}


