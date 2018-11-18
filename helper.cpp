//
// Created by lightol on 18-11-18.
//

#include "helper.h"

namespace LINE_PNP
{
double helper::ComputeDistPt2Line(const laneSection::center_type &lane, const Eigen::Vector3d &Oc) {
    auto geometry = lane.lane().border().geometry();
    geometry::point_sequence points = geometry.point();
    auto point1 = points.front();
    auto point2 = points.back();
    Eigen::Vector3d p1 = Eigen::Vector3d(*point1.x(), *point1.y(), *point1.z());
    Eigen::Vector3d p2 = Eigen::Vector3d(*point2.x(), *point2.y(), *point2.z());
    Eigen::Vector3d p1p2 = p2 - p1;
    p1p2.normalize();
    Eigen::Vector3d p1O = Oc - p1;
    double dist = p1O.cross(p1p2).norm();
    Eigen::Vector3d rv = p1p2.cross(p1O).normalized();
    if (rv.z() > 0) {  // 方向向下，说明车在右边车道
        return -dist;
    } else {
        return dist;
    }
}

double helper::ComputeDistPt2LineSeg(const laneSection::center_type &lane, const Eigen::Vector3d &Oc) {
    auto geometry = lane.lane().border().geometry();
    geometry::point_sequence points = geometry.point();
    int numPts = points.size();

    // Step1: 求出在reference line上离光心最近的一点
    std::vector<Eigen::Vector3d> pts;
    pts.resize(numPts);
    std::vector<double> distP2P;
    distP2P.resize(numPts);
    Eigen::Vector3d p = Eigen::Vector3d::Zero();  // reference line上的任一点

    for (int i = 0; i < numPts; ++i)
    {
        auto point = points[i];
        p[0] = *point.x();
        p[1] = *point.y();
        p[2] = *point.z();
        pts[i] = p;
        distP2P[i] = (p - Oc).norm();
    }

    double minDist = *std::min_element(distP2P.begin(), distP2P.end());
//    int idx = std::distance(distP2P.begin(), std::min_element(distP2P.begin(), distP2P.end()));
//    p = pts[idx];

    // Step2: 判断光心在reference line的左边还是右边
    auto point1 = points.front();
    auto point2 = points.back();

    Eigen::Vector3d p1 = Eigen::Vector3d(*point1.x(), *point1.y(), *point1.z());
    Eigen::Vector3d p2 = Eigen::Vector3d(*point2.x(), *point2.y(), *point2.z());
    Eigen::Vector3d p1p2 = p2 - p1;
    p1p2.normalize();
    Eigen::Vector3d p1O = Oc - p1;
    Eigen::Vector3d rv = p1p2.cross(p1O).normalized();
    if (rv.z() > 0) {  // 方向向下，说明车在右边车道
        return -minDist;
    } else {
        return minDist;
    }
}

double helper::ComputeDistPt2Line(const halfRoad::lane_type &lane, const Eigen::Vector3d &center,
                                  const Eigen::Vector3d &dir) {
    lane::border_type border = lane.border();
    border::geometry_type geometry = border.geometry();
    double x(0.0), y(0.0), z(0.0);
    std::vector<Eigen::Vector3d> pts;
    geometry::point_sequence points = geometry.point();
    int numPts = points.size();
    for (geometry::point_type point : points) {
        x = *point.x();
        y = *point.y();
        z = *point.z();
        pts.push_back(Eigen::Vector3d(x, y, z));
    }

    std::vector<double> dists;
    dists.reserve(numPts);

    for (const Eigen::Vector3d &pt : pts) {
        double dist = fabs((pt - center).cross(dir).norm());
        dists.push_back(dist);
    }

    double DistAva = std::accumulate(dists.begin(), dists.end(), 0.0)/numPts;

    return DistAva;
}

double helper::ComputeAnglePt2Line(const laneSection::center_type &lane,
                                   const Eigen::Vector3d &Oc) {
    auto geometry = lane.lane().border().geometry();
    geometry::point_sequence points = geometry.point();
    auto point1 = points.front();
    auto point2 = points.back();
    Eigen::Vector3d p1 = Eigen::Vector3d(*point1.x(), *point1.y(), *point1.z());
    Eigen::Vector3d p2 = Eigen::Vector3d(*point2.x(), *point2.y(), *point2.z());
    Eigen::Vector3d p1p2 = p2 - p1;
    p1p2.normalize();
    Eigen::Vector3d p1O = Oc - p1;
    p1O.normalize();
    double angle1 = acos(p1p2.dot(p1O));

    Eigen::Vector3d p2p1 = p1 - p2;
    p2p1.normalize();
    Eigen::Vector3d p2O = Oc - p2;
    p2O.normalize();
    double angle2 = acos(p2p1.dot(p2O));

    return std::max(angle1, angle2)*180/M_PI;
}

double helper::ComputeStraightness(const roadBoundary::geometry_type &geometry,
                                   Eigen::Vector3d &center, Eigen::Vector3d &dir) {
    double x(0.0), y(0.0), z(0.0);
    center = Eigen::Vector3d(0, 0, 0);
    std::vector<Eigen::Vector3d> pts;
    geometry::point_sequence points = geometry.point();
    int numPts = points.size();
    for (geometry::point_type point : points) {
        x = *point.x();
        y = *point.y();
        z = *point.z();
        pts.push_back(Eigen::Vector3d(x, y, z));
        center += Eigen::Vector3d(x, y, z);
    }
    center /= numPts;

    // 做个eigen分解，求出这批直线的主方向
    Eigen::Matrix<double, 3, Eigen::Dynamic> linePts = Eigen::MatrixXd::Zero(3, numPts);
    for (int i = 0; i < numPts; ++i) {
        linePts.col(i) = pts[i] - center;
    }

    Eigen::MatrixXd pointCloud = linePts*linePts.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(pointCloud);
    dir = eigenSolver.eigenvectors().col(2);
    double dists = 0.0;
    for (int i = 0; i < numPts; ++i) {
        double dist = linePts.col(i).cross(dir).norm();
        dists += dist;
    }

    return dists/numPts;
}

std::vector<Eigen::Vector3d> helper::ComputeROIpts(geometry::point_sequence points,
                                                   Eigen::Vector3d Oc, bool isReverse) {
    double dist = 0;

//    Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
    std::vector<double> dists;
    dists.resize(points.size());

    for (int i = 0; i < points.size(); ++i) {
        auto point = points[i];
        Eigen::Vector3d p = Eigen::Vector3d(*point.x(), *point.y(), *point.z());
        dist = (p - Oc).norm();
        dists[i] = dist;
    }

    double minDist = *std::min_element(dists.begin(), dists.end());
    int id1 = std::distance(dists.begin(), std::min_element(dists.begin(), dists.end()));
    std::vector<Eigen::Vector3d> pts;

    if (isReverse) {
        id1 -= 1;   // 为了保证线段起点在视野里面

//        pts.reserve(id1);
        for (int i = id1; i > 0; --i) {
            auto point = points[i];
            Eigen::Vector3d p = Eigen::Vector3d(*point.x(), *point.y(), *point.z());
            pts.push_back(p);
        }
    } else {
        id1 += 1;

        pts.reserve(points.size() - id1);
        for (int i = id1; i < points.size(); ++i) {
            auto point = points[i];
            Eigen::Vector3d p = Eigen::Vector3d(*point.x(), *point.y(), *point.z());
            pts.push_back(p);
        }
    }

    return pts;
}

int helper::FindNearestPoint(const geometry::point_sequence &points, const Eigen::Vector3d &Oc) {
    int numPts = points.size();
    std::vector<double> distP2P;
    distP2P.resize(numPts);
    Eigen::Vector3d p = Eigen::Vector3d::Zero();  // reference line上的任一点

    for (int i = 0; i < numPts; ++i)
    {
        auto point = points[i];
        p[0] = *point.x();
        p[1] = *point.y();
        p[2] = *point.z();
        distP2P[i] = (p - Oc).norm();
    }

    int idx = std::distance(distP2P.begin(), std::min_element(distP2P.begin(), distP2P.end()));
    return idx;
}

bool helper::JudgeDirection(const geometry::point_sequence &points, const Eigen::Vector3d &Oc,
                            const Eigen::Vector3d &pNearest)
{
    // Step2: 判断光心在reference line的左边还是右边
    auto pointEnd = points.back();
    Eigen::Vector3d pEnd = Eigen::Vector3d(*pointEnd.x(), *pointEnd.y(), *pointEnd.z());
    Eigen::Vector3d p1 = pEnd - pNearest;  // reference line的方向
    p1.normalize();
    Eigen::Vector3d p2 = Oc - pNearest;    // 相机光心的方向
    Eigen::Vector3d rv = p2.cross(p1).normalized();
    if (rv.z() > 0) {  // 方向向上，说明车在右边车道
        return true;
    } else {
        return false;
    }
}
}


