//
// Created by lightol on 18-11-18.
//

#ifndef ORB_TEMP_TYPES_H
#define ORB_TEMP_TYPES_H

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/mat.hpp>

namespace LINE_PNP
{
struct InputFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// camera
    cv::Mat imL;

    /// ground truth position
    Eigen::Matrix4d gtTwc;
    /// 2D lanes
    // 2D图像上表示每条车道线的上下两个端点
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > lanesEndPoint;
    // 表示这张image上所有被判定为车道线的像素
    std::vector<std::vector<Eigen::Vector2d> > vvLanePixels;
    std::vector<int> numPixels;
};
}

#endif  // ORB_TEMP_TYPES_H
