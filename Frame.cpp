//
// Created by lightol on 18-11-26.
//

#include <sophus/so3.h>
#include <iostream>
#include "Frame.h"

namespace LINE_PNP
{

int Frame::nNextId = 0;

Frame::Frame(const cv::Mat &image, const Eigen::Matrix4d Tcw,
             const std::vector<std::vector<Eigen::Vector2d> > &vLanePixels, const Eigen::Matrix3d &K): mImage(image), mTcw(Tcw),
             mvvLanePixels(vLanePixels), mK(K), bFinished(false)
{
    mnId = nNextId++;

    // Step0: 更新mTwc等相关量
    UpdateTwc();

    // Step2: 将所有属于车道线的像素聚类成不同的车道线集合
//    mnLanes = mvLaneEnds.size();
}

void Frame::UpdateTwc()
{
    mR = mTcw.topLeftCorner(3, 3);
    mt = mTcw.topRightCorner(3, 1);
    mTwc = mTcw.inverse();
    mOc = mTwc.topRightCorner(3, 1);

    // 12维的pose转换成6维向量,其实就是旋转矩阵转化为旋转向量
    Sophus::SO3 SO3_R(mR);
    Eigen::Vector3d rotationVector = SO3_R.log();
    mpTcw = new double[6];

    for (int i = 0; i < 3; ++i) {
        mpTcw[i] = rotationVector[i];
    }

    for (int i = 0; i < 3; ++i) {
        mpTcw[i + 3] = mt[i];
    }
}

void Frame::UpdataTcw()
{
    Eigen::Vector3d rv = Eigen::Map<const Eigen::Vector3d>(mpTcw);
    Eigen::AngleAxisd angleAxisd(rv.norm(), rv.normalized());
    mR = Eigen::Matrix3d(angleAxisd);
    mt = Eigen::Map<const Eigen::Vector3d>(mpTcw + 3);
    mTcw.topLeftCorner(3, 3) = mR;
    mTcw.topRightCorner(3, 1) = mt;
    mTwc = mTcw.inverse();
}

void Frame::FakeZ(const LINE_PNP::Frame &frame)
{
    // Step1: 求出当前帧的光心在frame的相机坐标系下的坐标
    Eigen::Vector4d c = mTwc.col(3);  // 使用齐次坐标
    Eigen::Vector4d Xc = frame.mTcw * c;

    // Step2: 把z轴方向置为0,也就是默认在光轴方向上,Tcw2并没有动
    Xc[2] = 0;

    // Step3: 还原Tcw2的值
    Eigen::Vector4d fakedC = frame.mTwc * Xc;  // 在frame的相机坐标系下,z坐标为0的相机光心
    mTwc.col(3) = fakedC;
    mTcw = mTwc.inverse();
    UpdateTwc();
}

void Frame::ReprojectLanes(const std::vector<std::vector<Eigen::Vector3d> > &vvLanePts)
{
    int nLanes = vvLanePts.size();
    mvvReprojectedLanePts.clear();
    mvvReprojectedLanePts.resize(nLanes);
    double depth = 0.0;  // 临时变量

    for (int i = 0; i < nLanes; ++i)
    {
        const std::vector<Eigen::Vector3d> &vLanePts = vvLanePts[i];
        int nPts = vLanePts.size();
        mvvReprojectedLanePts[i].resize(nPts);

        for (int j = 0; j < nPts; ++j)
        {
            Eigen::Vector3d Pw = vLanePts[j];
            Eigen::Vector3d Pc = mR * Pw + mt;
            depth = Pc[2];

            if (depth < 0)
            {
                mvvReprojectedLanePts[i][j] = Eigen::Vector2d::Zero();
                continue;
            }

            Pc /= Pc[2];  // 变换到归一化平面
            Eigen::Vector3d Puv = mK * Pc;
            mvvReprojectedLanePts[i][j] = Puv.topRows(2);
        }
    }

    bFinished = true;
}

Frame::Frame(const Frame& frame)
{
    mnId = frame.mnId;
    mR = frame.mR;
    mnLanes = frame.mnLanes;
    mt = frame.mt;
    mOc = frame.mOc;
    mTcw = frame.mTcw;
    mTwc = frame.mTwc;
    mK = frame.mK;
    bFinished = frame.bFinished;
    mpTcw = new double[6];
    for (int i = 0; i < 6; ++i)
    {
        mpTcw[i] = frame.mpTcw[i];
    }

    mvLaneEnds.assign(frame.mvLaneEnds.begin(), frame.mvLaneEnds.end());

//    mvvLanePixels.assign(frame.mvvLanePixels.begin(), frame.mvvLanePixels.end());

    mvvReprojectedLanePts.assign(frame.mvvReprojectedLanePts.begin(), frame.mvvReprojectedLanePts.end());

    mImage = frame.mImage.clone();
}

Frame::Frame()
{
    mR = Eigen::Matrix3d::Identity();
    mnLanes = 0;
    mt = Eigen::Vector3d::Zero();
    mOc = Eigen::Vector3d::Zero();
    mTcw = Eigen::Matrix4d::Identity();
    mTwc = Eigen::Matrix4d::Identity();
    mK = Eigen::Matrix3d::Identity();
    mpTcw = new double[6];
    for (int i = 0; i < 6; ++i)
    {
        mpTcw[i] = 0;
    }

    bFinished = false;
}

}