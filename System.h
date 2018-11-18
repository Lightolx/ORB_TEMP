//
// Created by lightol on 18-11-18.
//

#ifndef ORB_TEMP_SYSTEM_H
#define ORB_TEMP_SYSTEM_H

#include <iostream>
#include <sophus/so3.h>
#include <eigen3/Eigen/Eigen>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <iomanip>

#include "OpenDRIVE_1.4H.hxx"
#include "Types.h"
#include "Solver.h"
#include "CorConverter.h"
#include "MapDrawer.h"
#include "Viewer.h"
#include "Track.h"

using std::cout;
using std::endl;

namespace LINE_PNP
{
class System {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit System(const std::string filename);

    Eigen::Matrix4d Track(const InputFrame &frame);

    void ReprojectLanes(const InputFrame &frame) const;

    MapDrawer* mpMapDrawer;

    Viewer* mpViewer;
    std::thread* mptViewer;

    LINE_PNP::Track* mpTracker;

private:
    double gtDistLeft;
    double gtDistRight;
    double realDistLeft;
    double realDistRight;
    std::shared_ptr<OpenDRIVE> mpHDmap;
    Eigen::Matrix3d K;

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > mvReprojectedLanes;  // 重投影回去的车道线的(u,v)
};

}
#endif  // ORB_TEMP_SYSTEM_H
