//
// Created by lightol on 18-11-26.
//

#ifndef LINE_PNP_FRAME_H
#define LINE_PNP_FRAME_H

#include <opencv2/core/mat.hpp>
#include <eigen3/Eigen/Eigen>

namespace LINE_PNP
{
class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame();
    Frame(const cv::Mat &image, const Eigen::Matrix4d Tcw,
          const std::vector<std::vector<Eigen::Vector2d> > &vLanePixels, const Eigen::Matrix3d &K);

    Frame(const Frame& frame);

    // 根据Tcw更新Twc等量
    void UpdateTwc();

    // 根据旋转向量更新Tcw等量
    void UpdataTcw();

    void FakeZ(const Frame &frame);

    // 把3D车道线重投影回经过pose优化的当前帧的图像平面上
    void ReprojectLanes(const std::vector<std::vector<Eigen::Vector3d> > &vvLanePts);
    void ReprojectLanes(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > &vvLanePts);

    Eigen::Matrix4d GetTcw() {
        return mTcw;
    }

    Eigen::Matrix4d GetTwc() {
        return mTwc;
    }

    cv::Mat mImage;

    int mnLanes;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > mvLaneEnds;  // 图像上每条车道线的上下两个端点
    std::vector<int> mvNumPixels;

    double* mpTcw;

    std::vector<std::vector<Eigen::Vector2d> > mvvLanePixels;  // 图像上所有被判定为车道线的像素

    // 车道线上的3D点重投影回该帧的图像平面上得到的像素
    std::vector<std::vector<Eigen::Vector2d> > mvvReprojectedLanePts;

    Eigen::Matrix4d mTcw;
    Eigen::Matrix4d mTwc;
    Eigen::Matrix3d mR;
    Eigen::Vector3d mt;
    Eigen::Vector3d mOc;

    int mnId;
    static int nNextId;

    bool bFinished;

    double mDistLeft;
    double mDistRight;

private:

    Eigen::Matrix3d mK;
};
}


#endif //LINE_PNP_FRAME_H
