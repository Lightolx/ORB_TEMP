//
// Created by lightol on 18-11-21.
//

#include <opencv/cv.hpp>
#include "System.h"
#include "helper.h"

namespace LINE_PNP
{
System::System(const std::string filename):mpHDmap(nullptr) {
    mpHDmap = OpenDRIVE_(filename);
    if (!(mpHDmap)) {
        std::cerr << "Cannot load HDmap file at " << filename << endl;
        abort();
    }

    CorConverter::Convert2NED(mpHDmap);

    mpMapDrawer = new MapDrawer();
    mpMapDrawer->SetHDmapPtr(mpHDmap);

    mpTracker = new LINE_PNP::Track();
    mpTracker->SetHDmapPtr(mpHDmap);
    mpTracker->SetMapDrawerPtr(mpMapDrawer);

    mpViewer = new Viewer(mpMapDrawer);
    mpViewer->SetTracker(mpTracker);
    mptViewer = new std::thread(&Viewer::Run, mpViewer);
}

Eigen::Matrix4d System::Track(const InputFrame &frame) {
    return mpTracker->Run(frame);
}

//void System::VisualLanes(const InputFrame &frame) const
//{
//    cv::Mat image = frame.imL.clone();
//    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > vEndPointPairs = frame.lanesEndPoint;
//    int nLanes = vEndPointPairs.size();
//
//    // Step1: 画出所有被判定为车道线的像素
//    std::vector<Eigen::Vector2d> pts = frame.lanePixels;
//    uchar* ptr = image.data;
//    int step0 = image.step[0];
//    int step1 = image.step[1];
//    int channels = image.channels();
//    assert(channels == 3);
//    int elemSize1 = image.elemSize1();
//
//    for (auto pt : pts) {
//        int row = pt[0];
//        int col = pt[1];
//        for (int i = row; i < row+1; ++i) {
//            for (int j = col; j < col+1; ++j) {
//                *(ptr + i*step0 + j*step1 + 0*elemSize1) = 255;
//                *(ptr + i*step0 + j*step1 + 0*elemSize1) = 0;
//                *(ptr + i*step0 + j*step1 + 0*elemSize1) = 255;
//            }
//        }
//    }
//
//    // Step2: 画出车道线拟合出的直线
//    for (int i = 0; i < nLanes; ++i) {
//        // cv::Point的格式是(u,v),即先(col,row)
//        cv::Point p1(vEndPointPairs[i].first[0], vEndPointPairs[i].first[1]);
//        cv::Point p2(vEndPointPairs[i].second[0], vEndPointPairs[i].second[1]);
//        if (i ==0 || i == 1) {
//            cv::line(image, p1, p2, cv::Scalar(0, 0, 255), 3);
//        } else {
//            cv::line(image, p1, p2, cv::Scalar(0, 255, 255), 3);
//        }
//    }
//
//    CvMat im = image;
//    CvFont font;
//    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1.5, 1.5, 0, 1);
//    const char* msgLeft = "Left dist: ";
//    const char* leftDist = std::to_string(gtDistLeft - 0.625).c_str();
//    cvPutText(&im, msgLeft, cvPoint(80,70), &font, cvScalar(0, 0, 255));
//    cvPutText(&im, leftDist, cvPoint(350,70), &font, cvScalar(0, 0, 255));
//
//    const char* msgRight = "Right dist: ";
//    const char* RightDist = std::to_string(gtDistRight - 0.625).c_str();
//    cvPutText(&im, msgRight, cvPoint(80,115), &font, cvScalar(0, 0, 255));
//    cvPutText(&im, RightDist, cvPoint(350,115), &font, cvScalar(0, 0, 255));
//
//
//    cv::namedWindow("lanes", cv::WINDOW_NORMAL);
//    cv::resizeWindow("lanes", 800, 400);
//
//    cv::imshow("lanes", image);
//    cv::waitKey(1);
//}

void System::ReprojectLanes(const InputFrame &frame) const
{
    cv::Mat image = frame.imL;

    for (int i = 0; i < mvReprojectedLanes.size(); ++i) {
        // cv::Point的格式是(u,v),即先(col,row)
        cv::Point p1(mvReprojectedLanes[i].first[0], mvReprojectedLanes[i].first[1]);
        cv::Point p2(mvReprojectedLanes[i].second[0], mvReprojectedLanes[i].second[1]);
        if (i ==0 || i == 1) {
            cv::line(image, p1, p2, cv::Scalar(0, 0, 255), 3);
        } else {
            cv::line(image, p1, p2, cv::Scalar(0, 255, 255), 3);
        }
    }

    CvMat im = image;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1.5, 1.5, 0, 1);
    const char* msgLeft = "Left dist: ";
    const char* leftDist = std::to_string(realDistLeft - 0.625).c_str();
    cvPutText(&im, msgLeft, cvPoint(80,70), &font, cvScalar(0, 0, 255));
    cvPutText(&im, leftDist, cvPoint(350,70), &font, cvScalar(0, 0, 255));

    const char* msgRight = "Right dist: ";
    const char* RightDist = std::to_string(realDistRight - 0.625).c_str();
    cvPutText(&im, msgRight, cvPoint(80,115), &font, cvScalar(0, 0, 255));
    cvPutText(&im, RightDist, cvPoint(350,115), &font, cvScalar(0, 0, 255));

    cv::namedWindow("relanes", cv::WINDOW_NORMAL);
    cv::resizeWindow("relanes", 800, 400);

    cv::imshow("relanes", image);
    cv::waitKey(1);
}
}