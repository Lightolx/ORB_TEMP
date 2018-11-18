//
// Created by lightol on 18-11-18.
//

#include <unistd.h>

#include "Viewer.h"
#include <GL/gl.h>
#include <GL/glut.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace LINE_PNP
{
void
print_bitmap_string(void* font, char* s)
{
    if (s && strlen(s)) {
        while (*s) {
            glutBitmapCharacter(font, *s);
            s++;
        }
    }
}

void Viewer::Run() {
    pangolin::CreateWindowAndBind("SenseVP Map: Viewer", 1024, 768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory", true, true);
    pangolin::Var<bool> menuShowLanes("menu.Show Lanes", true, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 2000,
                                       2000, 512, 389, 0.1, 50000),
            pangolin::ModelViewLookAt(0, -100,
                                      -0.1, -1, 0, 0, 1.0, 0.0, 0.0)
    );

    pangolin::Handler3D *handler = new pangolin::Handler3D(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(handler);

    pangolin::OpenGlMatrix gtTwc;
    gtTwc.SetIdentity();

    pangolin::OpenGlMatrix noisedTwc;
    noisedTwc.SetIdentity();

    pangolin::OpenGlMatrix optimizidTwc;
    optimizidTwc.SetIdentity();

    bool bFollow = true;

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        if (!mpTracker->GetCurrentFrame().bFinished)
//        {
//            usleep(3000);
//        }

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(mpMapDrawer->GetGtCameraPose(), gtTwc);
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(mpMapDrawer->GetOptimizedCameraPose(), optimizidTwc);
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(mpMapDrawer->GetNoisedCameraPose(), noisedTwc);

        if (menuFollowCamera && bFollow) {
            s_cam.Follow(gtTwc);
        } else if (menuFollowCamera && !bFollow) {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
                    0, -1000, -0.1, 0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(gtTwc);
            bFollow = true;
        } else if (!menuFollowCamera && bFollow) {
            bFollow = false;
        }

        d_cam.Activate(s_cam);
        // black background
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        // draw map
        mpMapDrawer->DrawCurrentCamera(gtTwc, Eigen::Vector3f(0, 1, 0));
//        mpMapDrawer->DrawCurrentCamera(noisedTwc, Eigen::Vector3f(0 , 0, 1), 1.0);
        mpMapDrawer->DrawCurrentCamera(optimizidTwc, Eigen::Vector3f(1, 0, 0));

        if (menuShowTrajectory)
            mpMapDrawer->DrawTrajectory();
        if (menuShowLanes)
            mpMapDrawer->DrawHDMap();

        if (!mpTracker) {
            std::cerr << "mpTracker is null" << std::endl;
        }

        pangolin::FinishFrame();

        /*
        Frame frame = mpTracker->GetCurrentFrame();
        // draw lanes
        cv::Mat image = frame.mImage.clone();
        if (image.empty()) {
            continue;
        }

        std::vector<std::vector<Eigen::Vector2d> > vvLanePts = frame.mvvLanePixels;
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

//        for (int i = 0; i < vvLanePts.size(); ++i)
//        {
//            std::vector<Eigen::Vector2d> pts = vvLanePts[i];
//            Eigen::Vector3i rgb = vRGBs[i];
//
//            for (auto pt : pts) {
//                int u = pt[0];
//                int v = pt[1];
//                *(ptr + v*step0 + u*step1 + 0*elemSize1) = rgb[0];
//                *(ptr + v*step0 + u*step1 + 1*elemSize1) = rgb[1];
//                *(ptr + v*step0 + u*step1 + 2*elemSize1) = rgb[2];
//            }
//
//        }

        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > vEndPointPairs = frame.mvLaneEnds;
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

//        CvMat im = image;
//        CvFont font;
//        cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1.5, 1.5, 0, 1);
//        const char* msgLeft = "Left dist: ";
//        const char* leftDist = std::to_string(frame.mnId).c_str();
//        cvPutText(&im, msgLeft, cvPoint(80,70), &font, cvScalar(0, 0, 255));
//        cvPutText(&im, leftDist, cvPoint(350,70), &font, cvScalar(0, 0, 255));

//        cv::imshow("current image", image);
//        cv::waitKey(1);

        cv::Mat im = frame.mImage.clone();
//        if (im.empty()) {
//            continue;
//        }

        std::vector<std::vector<Eigen::Vector2d> > vvReprojectedLanePts = frame.mvvReprojectedLanePts;
        for (const std::vector<Eigen::Vector2d> & vReprojectedLanePts : vvReprojectedLanePts)
        {
            Eigen::Vector2d p1 = vReprojectedLanePts.front();
            Eigen::Vector2d p2 = vReprojectedLanePts.back();
            cv::Point point1(p1[0], p1[1]);
            cv::Point point2(p2[0], p2[1]);
            cv::line(im, point1, point2, cv::Scalar(0, 0, 255), 3);
        }

        cv::imshow("ReProjected image", im);
        cv::waitKey(1);
         */
    }
}
}
