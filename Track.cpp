//
// Created by lightol on 18-11-23.
//
#include <sophus/so3.h>
#include <eigen3/Eigen/Eigen>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <iomanip>
#include <opencv/cv.hpp>

#include "Track.h"
#include "helper.h"
#include "Solver.h"

namespace LINE_PNP {
bool Track::LocateCamera()
{
    Eigen::Vector3d Oc = mCurrentFrame.mOc;
    OpenDRIVE::road_sequence roads = mpHDmap->road();

    // Step1: 判断当前相机处在哪条road上
    // 求出current frame到各个road的reference line的最近距离
    int roadID = 0;                 // 最近reference line的ID
    double minDist = UINT_MAX;      // Oc到最近reference line的距离
    Eigen::Vector3d nearestP = Eigen::Vector3d::Zero();  // reference line上到Oc的最近点

    for (int i = 0; i < roads.size(); i++) {
        OpenDRIVE::road_type road = roads[i];
        // SectionID肯定为0,所以不记了,目前给出的xml地图就是这么干的...
        auto points = road.lanes().laneSection()[0].center().lane().border().geometry().point();
        int minID = helper::FindNearestPoint(points, Oc);  // points序列中离Oc最近的点的id
        Eigen::Vector3d p(*points[minID].x(), *points[minID].y(), *points[minID].z());
        double dist = (p - Oc).norm();
        if (dist < minDist) {
            roadID = i;
            nearestP = p;
            minDist = dist;
        }
    }

    // 如果到最近的车道还超过25m远,那么只能认为附近没有3D车道线
    if (minDist > 25) {
        cout << "Cannot find enough lanes at this position" << endl;
        return false;
    }

    mpHDmap->activeRoadID = roadID;
    mpHDmap->activeSectionID = 0;

    // Step2: 判断相机在reference line的左侧还是右侧,因为在2D图像上只可能提取出一侧的车道线
    // step2.1: 判断当前是在左边车道还是右边车道上, 然后提取出可能看到的的车道线序列
    //          从右往左分别有4种车道线, lanes in right, leftBoundary, reference line, lanes in left,
    lanes::laneSection_type laneSection = roads[roadID].lanes().laneSection()[0];
    laneSection::center_type centerLane = laneSection.center();
    // 检测当前帧的z轴正方向与reference line的方向是否一致,一致则说明在右边车道上,相反则说明在左边车道上
    // 将endPt变换到相机坐标系下
    auto startPt = centerLane.lane().border().geometry().point().front();  // ref line的起点
    auto endPt = centerLane.lane().border().geometry().point().back();
    Eigen::Vector3d p1 = Eigen::Vector3d(*startPt.x(), *startPt.y(), *startPt.z());
    Eigen::Vector3d p2 = Eigen::Vector3d(*endPt.x(), *endPt.y(), *endPt.z());
    Eigen::Vector3d pc1 = mCurrentFrame.mR * p1 + mCurrentFrame.mt;
    Eigen::Vector3d pc2 = mCurrentFrame.mR * p2 + mCurrentFrame.mt;

    if ((pc2 - pc1).z() > 0)  // 说明endPoint在startPoint的前面,同向,在右边车道,则考虑reference line和lanes in right
    {
        bInRight = true;
    } else  // 在左边车道,则考虑lanes in left和leftBorder;
    {
        bInRight = false;
    }

    return true;
}

Eigen::Matrix4d Track::Run(const LINE_PNP::InputFrame &frame)
{
    // Step0: 生成mCurrentFrame等预处理
    PreProcess(frame);

    // Step1: 定位相机在哪条路上,在reference line的左边还是右边
    if (!LocateCamera()) {
        std::cerr << "cannot locate camera" << endl;
        return frame.gtTwc;  // todo::这里返回什么
    }

    // Step1.5: 建立局部3D地图,相机当前所在的road及该road的successor(或predecessor),即当前相机可能看到的所有3D车道线
    ConstructLocalMap();

    // Step2: 在当前帧能看到的每一条3D车道线上各选出两点作为代表,用于接下来重投影回2D图像
    if (!ComputeROIlaneSeg()) {
        std::cerr << "cannot compute a roi lane" << endl;
        return frame.gtTwc;
    }
    // Step3: 将3D车道线重投影回2D图像后与对应2D车道线的像素误差作为代价函数,优化当前帧的pose
    if (!LinePnP()) {
        std::cerr << "line pnp failed" << endl;
        return frame.gtTwc;
    }

    // Step4: 后处理,根据优化后的旋转向量恢复当前帧的pose
    PostPrecess();

    // Step5: 重定位优化后的相机pose,也把3D车道线投回到2D平面看看结果对不对
    mCurrentFrame.ReprojectLanes(mvvLanePts);
    mCurrentGtFrame.ReprojectLanes(mvvLanePts);

    Eigen::Vector4d Oc4 = mCurrentFrame.mTwc.col(3);
    Eigen::Vector4d Pc4 = mCurrentGtFrame.mTcw * Oc4;
//    cout << (mCurrentFrame.mOc - mCurrentGtFrame.mOc).topRows(2).norm() << endl;
//    cout << std::fixed << std::setprecision(5) << fabs(Pc4[0]) << " " << fabs(Pc4[1]) << endl;
    mvErrors.push_back(fabs(Pc4[0]));
//    cout << std::accumulate(vErrors.begin(), vErrors.end(), 0.0) / vErrors.size() << endl;

    // Step6: 计算当前帧到左右车道的距离
    ComputeDist2Lanes();

    // Step6: 可视化结果
    VisualLanesAndReLanes();

    return mCurrentFrame.mTwc;

}

bool Track::ComputeROIlaneSeg()
{
    // Step3: 后处理,提取相机可以看见的车道线上的点
    Eigen::Vector3d Oc = mCurrentFrame.mOc;
    Eigen::Matrix3d R = mCurrentFrame.mR;
    Eigen::Vector3d t = mCurrentFrame.mt;

    mvLaneEnds.clear();
    mvLaneEnds.resize(mnLanes);
    for (int i = 0; i < mnLanes; ++i)
    {
        std::vector<Eigen::Vector3d> vLanePts = mvvLanePts[i];
        Eigen::Vector3d p2 = Eigen::Vector3d::Zero();

        // step1: 找到这条车道线上离相机光心最近的点
        int id0 = 0;
        double minDist = UINT_MAX;
        double tmpDist = 0.0;
        for (int j = 0; j < vLanePts.size(); ++j)
        {
            tmpDist = (vLanePts[j] - Oc).norm();
            if (tmpDist < minDist)
            {
                id0 = j;
                minDist = tmpDist;
            }
        }

        // step2: 找到车道线上出现在图像平面里的第一个点
        Eigen::Vector3d p = Eigen::Vector3d::Zero();   // 临时变量
        Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
        int startID = 0;
        for (int j = id0; j < vLanePts.size(); ++j)
        {
            p = vLanePts[j];
            Eigen::Vector3d pc = mCurrentFrame.mR * p + mCurrentFrame.mt;
            pc /= pc[2];
            Eigen::Vector3d uv = mK * pc;
            double u = uv[0];
            double v = uv[1];

            if (u > 0 && u < 1758 && v > 0 && v < 800)
            {
                p1 = p;
                startID = j;
                break;
            }
        }

        // step3: 找到车道线上图像平面里的第一个点后面5m左右的一个点
        for (int j = startID + 1; j < vLanePts.size(); ++j)
        {
            p = vLanePts[j];
            if ((p - p1).norm() > 10)
            {
                p2 = p;
                break;
            }
        }

        if (p1.isZero() || p2.isZero())  // 说明3D车道线无法在当前帧当前pose下被看到,或者看到的部分太短不足5m,可信度不够高
        {
            std::cerr << "a lane is too short" << endl;
        }

        mvLaneEnds[i] = std::make_pair(p1, p2);
    }

    // Step1: 3D车道线投影到2D图像上，进行一下重合度检查，如果在一定阈值范围内就认为是正确的匹配，开启下面的PNP流程
    return true;
}

void Track::PreProcess(const LINE_PNP::InputFrame &frame)
{
    // Step1: 生成mCurrentFrame
    Eigen::Matrix4d Twc = frame.gtTwc;
//    Eigen::Matrix3d R0 = Tcw.topLeftCorner(3, 3);
//    Eigen::Vector3d noiseRv = Eigen::Vector3d::Random();
//    Eigen::AngleAxisd noiseAngleAxis(0.0001*noiseRv.norm(), noiseRv.normalized());
//    Eigen::Matrix3d noiseR(noiseAngleAxis);
//    Tcw.topLeftCorner(3, 3) *= noiseR;
    Twc.topRightCorner(1, 1) += 10*Eigen::Matrix<double, 1, 1>::Random();  // x方向上加入幅值为2的噪声
    Twc.block(1, 3, 1, 1) += 1*Eigen::Matrix<double, 1, 1>::Random();
    Twc.block(2, 3, 1, 1) += 2*Eigen::Matrix<double, 1, 1>::Random();
//    Twc.topRightCorner(1, 1) += 11*Eigen::Matrix<double, 1, 1>::Identity();  // xy方向上加入幅值为2的噪声
    mCurrentFrame = Frame(frame.imL, Twc.inverse(), frame.vvLanePixels, mK);
    mCurrentFrame.mvLaneEnds = frame.lanesEndPoint;  // 这里其实是没有聚类好的结果的,需要自己在Frame类里完成
    mCurrentFrame.mvNumPixels = frame.numPixels;

    mCurrentFrame.mnLanes = mCurrentFrame.mvLaneEnds.size();
    mpMapDrawer->SetNoisedCameraPose(mCurrentFrame.GetTwc());

    // Step2: 生成mCurrentGtFrame,这个是为了作图方便,实际流程中并不能得到
    mCurrentGtFrame = Frame(frame.imL, frame.gtTwc.inverse(), frame.vvLanePixels, mK);
    mCurrentGtFrame.mvLaneEnds = frame.lanesEndPoint;
    mCurrentGtFrame.mnLanes = mCurrentGtFrame.mvLaneEnds.size();
    mpMapDrawer->SetGtCameraPose(mCurrentGtFrame.GetTwc());
}

bool Track::LinePnP()
{
    if (mnLanes != mCurrentFrame.mnLanes)  // 3D车道线与2D车道线数量不一样
    {
        return false;
    }

    // step0: 统计各条车道线上的像素数,像素数越多,这条车道线的重投影误差在优化中所占的权重越大
    int numPixelsAll = 0;
    for (int i = 0; i < mnLanes; ++i) {
        numPixelsAll += mCurrentFrame.mvNumPixels[i];
    }

    std::vector<double> vWeights;
    vWeights.resize(mnLanes);
    for (int i = 0; i < mnLanes; ++i) {
        vWeights[i] = double(mCurrentFrame.mvNumPixels[i]) / numPixelsAll;
    }

    ceres::Problem problem;
    double* pTcw = mCurrentFrame.mpTcw;
    for (int i = 0; i < mnLanes; ++i) {
        if (mvLaneEnds[i].first.isZero() || mvLaneEnds[i].second.isZero()) {
            continue;
        }
//        for (int i = 0; i < 3; ++i) {
        ceres::CostFunction* pCostFunction = new ceres::AutoDiffCostFunction
                <ReprojectError, 2, 6>(new ReprojectError(mCurrentFrame.mvLaneEnds[i], mvLaneEnds[i], mK, vWeights[i]));
        problem.AddResidualBlock(pCostFunction, nullptr, pTcw);
        for (int j = 0; j < 3; ++j) {
            problem.SetParameterLowerBound(pTcw, j, *(pTcw + j) - 0.01);
            problem.SetParameterUpperBound(pTcw, j, *(pTcw + j) + 0.01);
        }
    }

    if (problem.NumResidualBlocks() < 2)
    {
        std::cerr << "camera must observe at least 2 lanes" << std::endl;
        return false;
    }

    for (int j = 3; j < 6; ++j) {
//        if (j == 5 || j == 3)
//        {
//            problem.SetParameterLowerBound(pTcw, j, *(pTcw+j)-0.000000001);
//            problem.SetParameterUpperBound(pTcw, j, *(pTcw+j)+0.000000001);
//        } else
//        {
//            problem.SetParameterLowerBound(pTcw, j, *(pTcw+j)-0.9);
//            problem.SetParameterUpperBound(pTcw, j, *(pTcw+j)+0.9);
//        }
//        problem.SetParameterLowerBound(pTcw, j, *(pTcw+j)-1);
//        problem.SetParameterUpperBound(pTcw, j, *(pTcw+j)+1);
        if (j == 5) {
            problem.SetParameterLowerBound(pTcw, j, *(pTcw+j)-0.05);
            problem.SetParameterUpperBound(pTcw, j, *(pTcw+j)+0.05);
        }
    }

//    for (int j = 3; j < 6; j++)
//    {
//        cout << "t upper is " << problem.GetParameterUpperBound(pTcw, j) << endl;
//        cout << "t lower is " << problem.GetParameterLowerBound(pTcw, j) << endl;
//    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 101;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.FullReport() << "\n";

    if (summary.termination_type == ceres::CONVERGENCE) {
        return true;
    }

    return false;
}

void Track::PostPrecess()
{
    mCurrentFrame.UpdataTcw();

    // 手动操作,修正z方向上的分量与优化前保持一致
    mCurrentFrame.FakeZ(mCurrentGtFrame);

    mpMapDrawer->SetOptimizedCameraPose(mCurrentFrame.GetTwc());

//    cout << "gt is\n" << mCurrentGtFrame.GetTwc() << endl;
//    cout << "optimized is\n" << mCurrentFrame.GetTwc() << endl;
}

void Track::ConstructLocalMap()
{
    OpenDRIVE::road_sequence roads = mpHDmap->road();
    auto road = roads[mpHDmap->activeRoadID];
    lanes::laneSection_type laneSection = road.lanes().laneSection()[0];

    std::vector<geometry::point_sequence> vLanes;  // 临时变量,包含在道路一侧从左到右的所有车道线
//    bInRight = false;
    if (bInRight)  // 说明同向,在右边车道,则考虑reference line和lanes in right,考虑road的successor
    {
        bool bFindSuccessor = false;
        OpenDRIVE::road_type roadSuc = road;
        if (road.link().successor().elementType().get() == "road")
        {
            for (auto r : roads)
            {
                if (r.id().get() == road.link().successor().elementId().get())
                {
                    roadSuc = r;
                    bFindSuccessor = true;
                    break;
                }
            }
        }
        lanes::laneSection_type laneSectionSuc = roadSuc.lanes().laneSection()[0];

        // 加入reference line, 保证在相机视角下从左到右的顺序push_back
        geometry::point_sequence centerLanePts = laneSection.center().lane().border().geometry().point();
        if (bFindSuccessor)
        {
            geometry::point_sequence linkCenterLanePts = laneSectionSuc.center().lane().border().geometry().point();
            centerLanePts.insert(centerLanePts.end(), linkCenterLanePts.begin(), linkCenterLanePts.end());
        }
        vLanes.push_back(centerLanePts);

        halfRoad::lane_sequence lanesInRight = laneSection.right().lane();
        halfRoad::lane_sequence lanesInRightSuc = laneSectionSuc.right().lane();
        std::map<int, geometry::point_sequence, std::greater<int> > mIDvPoints;
        for (auto lane : lanesInRight)  // 在这里整理一下lanes的顺序,因为给的xml并不是按照从大到小顺序排列的,坑比
        {
            int laneID = lane.id().get();
            geometry::point_sequence lanePts = lane.border().geometry().point();
            if (bFindSuccessor)
            {
                for (auto laneSuc : lanesInRightSuc)
                {
                    if (laneSuc.id().get() == laneID)
                    {
                        geometry::point_sequence lanePtsSuc = laneSuc.border().geometry().point();
                        lanePts.insert(lanePts.end(), lanePtsSuc.begin(), lanePtsSuc.end());
                    }
                }
            }

            mIDvPoints[laneID] = lanePts;
        }

        for (auto iter = mIDvPoints.begin(); iter != mIDvPoints.end(); iter++)
        {
            vLanes.push_back(iter->second);
        }

        mnLanes = vLanes.size();
        mvvLanePts.resize(mnLanes);
        for (int i = 0; i < mnLanes; ++i)
        {
            int nPts = vLanes[i].size();
            mvvLanePts[i].resize(nPts);

            for (int j = 0; j < nPts; ++j)
            {
                mvvLanePts[i][j].x() = *vLanes[i][j].x();
                mvvLanePts[i][j].y() = *vLanes[i][j].y();
                mvvLanePts[i][j].z() = *vLanes[i][j].z();
            }
        }

    } else  // 在左边车道,则考虑lanes in left和leftBorder,考虑road的predecessor
    {
        bool bFindPredessor = false;
        OpenDRIVE::road_type roadPre = road;
        if (road.link().predecessor().elementType().get() == "road")
        {
            for (auto r : roads)
            {
                if (r.id().get() == road.link().predecessor().elementId().get())
                {
                    roadPre = r;
                    bFindPredessor = true;
                    break;
                }
            }
        }

        lanes::laneSection_type laneSectionPre = roadPre.lanes().laneSection()[0];

        halfRoad::lane_sequence lanesInLeft = laneSection.left().lane();
        halfRoad::lane_sequence lanesInLeftPre = laneSectionPre.left().lane();

        // 加入leftBoundary
        for (auto lane : lanesInLeft)
        {
            if (lane.leftBorder().present())
            {
                geometry::point_sequence lanePts = lane.leftBorder().get().geometry().point();

                if (bFindPredessor)
                {
                    for (auto lanePre : lanesInLeftPre)
                    {
                        if (lanePre.leftBorder().present())
                        {
                            geometry::point_sequence lanePtsPre = lanePre.leftBorder().get().geometry().point();
                            // 注意方向,pre在前,lanePts在后
                            lanePtsPre.insert(lanePtsPre.end(), lanePts.begin(), lanePts.end());

                            lanePts.clear();
                            lanePts.assign(lanePtsPre.begin(), lanePtsPre.end());
                        }
                    }
                }
                vLanes.push_back(lanePts);
            }
        }

        std::map<int, geometry::point_sequence> mIDvPoints;
        for (auto lane : lanesInLeft)  // 在这里整理一下lanes的顺序,因为给的xml并不是按照从大到小顺序排列的,坑比
        {
            int laneID = lane.id().get();
            geometry::point_sequence lanePts = lane.border().geometry().point();

            if (bFindPredessor)
            {
                for (auto lanePre : lanesInLeftPre)
                {
                    if (lanePre.id().get() == laneID)
                    {
                        geometry::point_sequence lanePtsPre = lanePre.border().geometry().point();
                        lanePtsPre.insert(lanePtsPre.end(), lanePts.begin(), lanePts.end());

                        lanePts.clear();
                        lanePts.assign(lanePtsPre.begin(), lanePtsPre.end());
                    }
                }
            }

            mIDvPoints[laneID] = lanePts;
        }

        for (auto iter = mIDvPoints.begin(); iter != mIDvPoints.end(); iter++)
        {
            vLanes.push_back(iter->second);
        }

        mnLanes = vLanes.size();
        mvvLanePts.resize(mnLanes);
        for (int i = 0; i < mnLanes; ++i)
        {
            int nPts = vLanes[i].size();
            mvvLanePts[i].resize(nPts);

            for (int j = 0; j < nPts; ++j)
            {
                // 反向输出,因为此时车的行驶方向与车道线方向相反
                mvvLanePts[i][j].x() = *vLanes[i][nPts - j - 1].x();
                mvvLanePts[i][j].y() = *vLanes[i][nPts - j - 1].y();
                mvvLanePts[i][j].z() = *vLanes[i][nPts - j - 1].z();
            }
        }
    }

    mpMapDrawer->SetRoiLanes(mvvLanePts);
}

void Track::VisualLanesAndReLanes() const
{

    // draw lanes
    cv::Mat image = mCurrentFrame.mImage.clone();
    if (image.empty()) {
        std::cerr << "empty image, something is wrong" << endl;
        return;
    }

    /*
    std::vector<std::vector<Eigen::Vector2d> > vvLanePts = mCurrentFrame.mvvLanePixels;
    uchar* ptr = image.data;
    int step0 = image.step[0];
    int step1 = image.step[1];
    int channels = image.channels();
    assert(channels == 3);
    int elemSize1 = image.elemSize1();

    std::vector<Eigen::Vector3i> vRGBs;
    vRGBs.push_back(Eigen::Vector3i(255, 0, 0));
    vRGBs.push_back(Eigen::Vector3i(0, 255, 0));
    vRGBs.push_back(Eigen::Vector3i(0, 0, 255));
    vRGBs.push_back(Eigen::Vector3i(255, 255, 0));
    vRGBs.push_back(Eigen::Vector3i(255, 0, 255));

        for (int i = 0; i < vvLanePts.size(); ++i)
        {
            std::vector<Eigen::Vector2d> pts = vvLanePts[i];
            Eigen::Vector3i rgb = vRGBs[i];

            for (auto pt : pts) {
                int u = pt[0];
                int v = pt[1];
                *(ptr + v*step0 + u*step1 + 0*elemSize1) = rgb[0];
                *(ptr + v*step0 + u*step1 + 1*elemSize1) = rgb[1];
                *(ptr + v*step0 + u*step1 + 2*elemSize1) = rgb[2];
            }

        }
        */

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > vEndPointPairs = mCurrentFrame.mvLaneEnds;
    int nLanes = vEndPointPairs.size();
    for (int i = 0; i < nLanes; ++i) {
        // cv::Point的格式是(u,v),即先(col,row)
        cv::Point p1(vEndPointPairs[i].first[0], vEndPointPairs[i].first[1]);
        cv::Point p2(vEndPointPairs[i].second[0], vEndPointPairs[i].second[1]);
        if (i ==0 || i == 1) {
            cv::line(image, p1, p2, cv::Scalar(0, 0, 255), 3);
        } else {
            cv::line(image, p1, p2, cv::Scalar(255, 0, 255), 3);
        }
    }

    CvMat imMsg = image;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1.5, 1.5, 0, 1);
    const char* msgLeft = "Left dist: ";
    const char* leftDist = std::to_string(mCurrentFrame.mDistLeft - 0.975).c_str();
    cvPutText(&imMsg, msgLeft, cvPoint(80,70), &font, cvScalar(0, 0, 255));
    cvPutText(&imMsg, leftDist, cvPoint(350,70), &font, cvScalar(0, 0, 255));
    const char* msgRight = "Right dist: ";
    const char* rightDist = std::to_string(mCurrentFrame.mDistRight - 0.975).c_str();
    cvPutText(&imMsg, msgRight, cvPoint(80,170), &font, cvScalar(0, 0, 255));
    cvPutText(&imMsg, rightDist, cvPoint(350,170), &font, cvScalar(0, 0, 255));

//    cv::imshow("current image", image);
//    cv::waitKey(1);

    cv::Mat im = mCurrentFrame.mImage.clone();
    std::vector<std::vector<Eigen::Vector2d> > vvReprojectedLanePts = mCurrentFrame.mvvReprojectedLanePts;
    for (const std::vector<Eigen::Vector2d> & vReprojectedLanePts : vvReprojectedLanePts)
    {
//        Eigen::Vector2d p1 = vReprojectedLanePts.front();
//        Eigen::Vector2d p2 = vReprojectedLanePts.back();
//        cv::Point point1(p1[0], p1[1]);
//        cv::Point point2(p2[0], p2[1]);
//        cv::line(im, point1, point2, cv::Scalar(0, 0, 255), 3);
        for (const Eigen::Vector2d & pt : vReprojectedLanePts)
        {
            if (pt.isZero())
            {
                continue;
            }

            cv::circle(im, cv::Point(pt[0], pt[1]), 2, cv::Scalar(0, 255, 255), 2);
        }

    }

    cv::imshow("optimized ReProjected image", im);
    cv::waitKey(1);

    cv::Mat im2 = mCurrentFrame.mImage.clone();
    std::vector<std::vector<Eigen::Vector2d> > vvReprojectedLanePts2 = mCurrentGtFrame.mvvReprojectedLanePts;
    for (const std::vector<Eigen::Vector2d> & vReprojectedLanePts : vvReprojectedLanePts2)
    {
//        Eigen::Vector2d p1 = vReprojectedLanePts.front();
//        Eigen::Vector2d p2 = vReprojectedLanePts.back();
//        cv::Point point1(p1[0], p1[1]);
//        cv::Point point2(p2[0], p2[1]);
//        cv::line(im, point1, point2, cv::Scalar(0, 0, 255), 3);
        for (const Eigen::Vector2d & pt : vReprojectedLanePts)
        {
            if (pt.isZero())
            {
                continue;
            }

            cv::circle(im2, cv::Point(pt[0], pt[1]), 2, cv::Scalar(0, 255, 255), 2);
        }
    }

    cv::imshow("Gt ReProjected image", im2);
    cv::waitKey(1);
}

void Track::ComputeDist2Lanes()
{
    std::vector<double> vDist;
    vDist.reserve(mnLanes);

    Eigen::Vector2d Oc = mCurrentFrame.mOc.topRows(2);
    for (auto vLanePts : mvvLanePts)
    {
        // step1: 找出这条车道线上与当前帧距离最近的一点
        double minDist = UINT_MAX;
        int minID = 0;

        double tmpDist = 0.0;
        int nIncrease = 0;  // 计数该点距离大于上一点距离,加速寻找最近点
        for (int i = 0; i < vLanePts.size(); ++i)
        {
            tmpDist = (vLanePts[i].topRows(2) - Oc).norm();
            if (tmpDist < minDist)
            {
                minDist = tmpDist;
                minID = i;
                nIncrease = 0;
            } else
            {
                nIncrease++;
            }

            if (nIncrease > 10)
            {
                break;
            }
        }

        // step2: 计算光心到最近点minID前一点与后一点形成的线段的距离
        Eigen::Vector3d p1 = vLanePts[minID - 1];
        Eigen::Vector3d p2 = vLanePts[minID + 1];
        Eigen::Vector3d C = mCurrentFrame.mOc;
        // 全都投影到水平面上
        p1[2] = 0;
        p2[2] = 0;
        C[2] = 0;

        Eigen::Vector3d l = p2 - p1;
        l.normalize();
        Eigen::Vector3d C1 = C - p1;
        double dist = C1.cross(l).norm();
        vDist.push_back(dist);
    }

    for (int i = 0; i < mnLanes - 1; ++i)
    {
        if (vDist[i] + vDist[i + 1] < 4)
        {
            mCurrentFrame.mDistLeft = vDist[i];
            mCurrentFrame.mDistRight = vDist[i + 1];
            break;
        }
    }
}

}