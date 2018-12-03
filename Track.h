//
// Created by lightol on 18-11-23.
//

#ifndef LINE_PNP_TRACK_H
#define LINE_PNP_TRACK_H

#include <eigen3/Eigen/Eigen>
#include "OpenDRIVE_1.4H.hxx"
#include "Types.h"
#include "Frame.h"
#include "MapDrawer.h"

namespace LINE_PNP
{

class MapDrawer;

class Track
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Track():mpHDmap(nullptr) {
        double fx = 2.002991e+03;
        double fy = 2.002991e+03;
        double cx = 9.053329e+02;
        double cy = 3.885141e+02;
        mK << fx,  0, cx,
                0, fy, cy,
                0,  0,  1;
    }

    Eigen::Matrix4d Run(const InputFrame &frame);

    void PreProcess(const InputFrame &frame);

    bool LocateCamera();

    void ConstructLocalMap();

    std::vector<double> ComputeLaneDists() const;

    // 把3D车道线重投影回2D平面，提取出能在2D图像上看到的且距离光心最近的5m车道线
    bool ComputeROIlaneSeg();

    bool LinePnP();

    void PostPrecess();

    void VisualLanesAndReLanes() const;

    void ComputeDist2Lanes();

    Frame GetCurrentFrame() const {
        return mCurrentFrame;
    }

    void SetHDmapPtr(std::shared_ptr<OpenDRIVE> pHDmap) {
        mpHDmap = pHDmap.get();
    }

    void SetMapDrawerPtr(MapDrawer* pMapDrawer) {
        mpMapDrawer = pMapDrawer;
    }

    std::vector<double> mvErrors;
private:
    OpenDRIVE* mpHDmap;
    MapDrawer* mpMapDrawer;

    int mnLanes;   // 当前road上能够看到的3D车道线的数量
//    std::vector<std::vector<Eigen::Vector3d> > mvvLanePts;  // 各条车道线能够被看到部分的点集
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >  mvLaneEnds;  // 车道线上被选出来的两个点

    Eigen::Matrix3d mK;

    Frame mCurrentFrame;
    Frame mCurrentGtFrame;

    std::vector<std::vector<Eigen::Vector3d> > mvvLanePts;  // 从左到右排列的局部地图中车道线的点集
    bool bInRight;
};
}


#endif //LINE_PNP_TRACK_H
