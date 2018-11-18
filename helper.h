//
// Created by lightol on 18-11-18.
// 专门用来进行计算点线距离等常规运算

#ifndef ORB_TEMP_HELPER_H
#define ORB_TEMP_HELPER_H

#include <iostream>

#include <eigen3/Eigen/Eigen>
#include "OpenDRIVE_1.4H.hxx"
#include "Types.h"

using std::cout;
using std::endl;

namespace LINE_PNP
{
class helper {
public:
    static std::vector<Eigen::Vector3d> ComputeROIpts(geometry::point_sequence lane,
                                                      Eigen::Vector3d Oc, bool isReverse = false);

    // 计算线上的每个点到reference line的距离作为车道线宽度
    static double ComputeDistPt2Line(const halfRoad::lane_type& lane,
                                     const Eigen::Vector3d &center, const Eigen::Vector3d &dir);

    static double ComputeDistPt2Line(const laneSection::center_type& lane,
                                     const Eigen::Vector3d &Oc);

    // 求出光心到各reference上面所有点的最小距离
    static double ComputeDistPt2LineSeg(const laneSection::center_type& lane,
                                        const Eigen::Vector3d &Oc);

    // 求出在points序列中离点Oc最近的一点
    static int FindNearestPoint(const geometry::point_sequence &points, const Eigen::Vector3d &Oc);

    // 判断当前光心是在reference line的左侧还是右侧，直接根据Oc与pNearest的连线与points向量的叉乘可以得到
    static bool JudgeDirection(const geometry::point_sequence &points, const Eigen::Vector3d &Oc,
                        const Eigen::Vector3d &pNearest);

    static double ComputeAnglePt2Line(const laneSection::center_type& lane,
                                      const Eigen::Vector3d &Oc);

    static double ComputeStraightness(const roadBoundary::geometry_type &geometry,
                                      Eigen::Vector3d &center, Eigen::Vector3d &dir);
};

}

#endif  // ORB_TEMP_HELPER_H
