//
// Created by lightol on 18-11-18.
//

#ifndef ORB_TEMP_MAPDRAWER_H
#define ORB_TEMP_MAPDRAWER_H

#include <iostream>
#include <vector>
#include <pangolin/pangolin.h>
#include "Types.h"
#include "OpenDRIVE_1.4H.hxx"
#include "Track.h"

namespace LINE_PNP
{
class MapDrawer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit MapDrawer(): mpHDmap(nullptr) {
//        mvGtTrajPosition = std::vector<Eigen::Vector3d>();
//        mvGtTrajPosition.clear();
//        mvOptimizedTrajPosition = std::vector<Eigen::Vector3d>();
//        mvOptimizedTrajPosition.clear();
//        mvNoisedTrajPosition = std::vector<Eigen::Vector3d>();
//        mvNoisedTrajPosition.clear();
//        std::cout << "initial, numtraj = " << mvNoisedTrajPosition.size() << std::endl;
    }

    void GetCurrentOpenGLCameraMatrix(const Eigen::Matrix4d& pose, pangolin::OpenGlMatrix &M);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc, const Eigen::Vector3f &rgb, double w = 2.0);

    void DrawTrajectory() const;

    void DrawHDMap();

    void DrawHDPoints(roadBoundary::geometry_type geometry,
                      float r = 1.0f, float g = 1.0f, float b = 1.0f);

    void DrawRoiLanes(float r = 1.0f, float g = 1.0f, float b = 1.0f);

    void SetOptimizedCameraPose(const Eigen::Matrix4d &Twc) {
        mOptimizedCameraPose = Twc;
        mvOptimizedTrajPosition.push_back(Twc.topRightCorner(3, 1));
    }

    Eigen::Matrix4d GetOptimizedCameraPose() {
        return mOptimizedCameraPose;
    }

    void SetGtCameraPose(const Eigen::Matrix4d &Twc) {
        mGtCameraPose = Twc;
        mvGtTrajPosition.push_back(Twc.topRightCorner(3, 1));
    }

    Eigen::Matrix4d GetGtCameraPose() {
        return mGtCameraPose;
    }

    void SetNoisedCameraPose(const Eigen::Matrix4d &Twc) {
        mNoisedCameraPose = Twc;
        mvNoisedTrajPosition.push_back(Twc.topRightCorner(3, 1));
    }

    Eigen::Matrix4d GetNoisedCameraPose() {
        return mNoisedCameraPose;
    }

//    void SetGtTrajPosition(const std::vector<Eigen::Matrix4d> &vPose) {
//        int nImages = vPose.size();
//        mvGtTrajPosition.resize(nImages);
//
//        for (int i = 0; i < nImages; ++i)
//        {
//            mvGtTrajPosition[i] = vPose[i].topRightCorner(3, 1);
//        }
//    }

    void SetRoiLanes(std::vector<std::vector<Eigen::Vector3d> > vvLanePts) {
        mvvLanePts = vvLanePts;
    }

    void SetHDmapPtr(std::shared_ptr<OpenDRIVE> pHDmap) {
        mpHDmap = pHDmap.get();
    }

//    void SetTrackerPtr(Track* pTracker) {
//        mpTracker = pTracker;
//    }

    OpenDRIVE* mpHDmap;
//    Track* mpTracker;
private:

    Eigen::Matrix4d mGtCameraPose;       // ground truth of camera pose
    Eigen::Matrix4d mNoisedCameraPose;   // camera pose before optimized
    Eigen::Matrix4d mOptimizedCameraPose;         // camera pose after optimized

    std::vector<Eigen::Vector3d> mvGtTrajPosition;
    std::vector<Eigen::Vector3d> mvNoisedTrajPosition;
    std::vector<Eigen::Vector3d> mvOptimizedTrajPosition;

    std::vector<std::vector<Eigen::Vector3d> > mvvLanePts;  // roi lanes, 相机当前能够看到的所有车道线
};
}

#endif  // ORB_TEMP_MAPDRAWER_H
