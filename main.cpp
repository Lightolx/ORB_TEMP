#include <iostream>
#include <fstream>
#include <thread>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "eigen3/Eigen/Eigen"

#include "Timer.h"
#include "Solver.h"
#include "Types.h"
#include "System.h"
#include "Viewer.h"

#include <GL/gl.h>
#include <GL/glut.h>

using std::cout;
using std::endl;
using std::cerr;
using namespace LINE_PNP;

void LoadSTCCImagePosition(const std::string &positionFile, std::vector<Eigen::Matrix4d> &vGtTwcs);

void LoadSTCCImageLanes(const std::string &filepath,
                        std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > > &vvEndPointPairs,
                        int nImages, int nStart, int nEnd);

void LoadSTCCImageLanes(const std::string &endPointsFile,
                        std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > > &vvEndPointPairs);

void LoadSTCCLanePixels(const std::string &filepath,
                        std::vector<std::vector<std::vector<Eigen::Vector2d> > > &vvLanePixels,
                        std::vector<std::vector<int> > &vNumPixels,
                        int nImages, int nStart, int nEnd);

void VisualLanes(const InputFrame &frame, int ni)
{
    cv::Mat image = frame.imL.clone();
    if (image.empty())
    {
        return;
    }

    std::vector<std::vector<Eigen::Vector2d> > vvLanePts = frame.vvLanePixels;
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

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > vEndPointPairs = frame.lanesEndPoint;
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

    CvMat im = image;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 1.5, 1.5, 0, 1);
    const char* msgLeft = "Left dist: ";
    const char* leftDist = std::to_string(ni).c_str();
    cvPutText(&im, msgLeft, cvPoint(80,70), &font, cvScalar(0, 0, 255));
    cvPutText(&im, leftDist, cvPoint(350,70), &font, cvScalar(0, 0, 255));

    cv::imshow("current image", image);
    cv::waitKey();
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    if (argc != 2)
    {
        cout << "usage: " << argv[0] << endl;
        return 1;
    }
    std::string path_to_data(argv[1]);

    // Read in images
    int nImages = 0;
    const std::string leftVideo(path_to_data + "/CAM101.avi");
    cv::VideoCapture leftCap;
    if (leftVideo.substr(leftVideo.size() - 4) == ".avi")
    {
        leftCap.open(leftVideo);
        if (!leftCap.isOpened())
        {
            cout << "open video failed." << endl;
        }

        nImages = leftCap.get(CV_CAP_PROP_FRAME_COUNT);
    }

    // Load camera position
    std::vector<Eigen::Matrix4d> vGtTwcs;
    vGtTwcs.reserve(nImages);
    LoadSTCCImagePosition(path_to_data + "/position.txt", vGtTwcs);

    // Load 2D lane information, i.e., endpoint pairs of each lane
    std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > > vvEndPointPairs;
    std::vector<std::vector<std::vector<Eigen::Vector2d> > > vvvLanePixels;
    std::vector<std::vector<int> > vvNumPixels;
    int startIndex = 5100;
    int endIndex = 5300;
//    LoadSTCCImageLanes("/home/lightol/backup/cluster_post/",
//                       vvEndPointPairs, nImages, startIndex, endIndex);  // 4040, 4240
    LoadSTCCImageLanes("endPoints4.txt", vvEndPointPairs);
    LoadSTCCLanePixels("/home/lightol/backup/cluster_post/", vvvLanePixels, vvNumPixels, nImages, startIndex, endIndex);

    System SLAM = System(path_to_data + "/HDmap3D_HZ_GPS.xml");

//     Main loop
    for (int ni = 0; ni < vvEndPointPairs.size(); ++ni)
    {
//        cout << ni << endl;
        InputFrame frame;
        leftCap >> frame.imL;
        if (frame.imL.empty())
        {
            cerr << endl << "Failed to load image" << endl;
        }

        if (ni < startIndex)
        {
            continue;
        } else if (ni > endIndex)
        {
//            getchar();
            break;
        }

        frame.gtTwc = vGtTwcs[ni];
        frame.lanesEndPoint = vvEndPointPairs[ni];
        frame.vvLanePixels = vvvLanePixels[ni];
        frame.numPixels = vvNumPixels[ni];

        Eigen::Matrix4d Twc = SLAM.Track(frame);
//        VisualLanes(frame, ni);
//        usleep(100000);
    }

    auto vErrors = SLAM.mpTracker->mvErrors;
    std::ofstream fout("errors.txt");
    for (double error : vErrors)
    {
        fout << std::fixed << std::setprecision(5) << error << endl;
    }
    fout.close();

}

void LoadSTCCImagePosition(const std::string &positionFile, std::vector<Eigen::Matrix4d> &vGtTwcs)
{
    std::ifstream fPosition(positionFile);
    if (!fPosition.is_open())
    {
        cerr << "open file " << positionFile << "failed." << endl;
        return;
    }
    std::string line;
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    while (getline(fPosition, line))
    {
        std::string positionStr = line.substr(line.find(' ') + 1);
        std::stringstream ss(positionStr);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                ss >> Twc(i, j);
            }
        }

        vGtTwcs.push_back(Twc);
    }
}

void LoadSTCCImageLanes(const std::string &filepath,
                        std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > > &vvEndPointPairs,
                        int nImages, int nStart, int nEnd)
{
    vvEndPointPairs.resize(nImages);
    for (int ni = nStart; ni < nEnd; ++ni)
    {
        cout << ni << endl;
        // Step0: 找到顺序排列的2D图像的文件名
        std::string filename = filepath + std::to_string(ni) + ".png";
        cv::Mat im = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        if (im.empty())
        {
            cerr << "cannot load 2D lanes at " << filename << endl;
            abort();
        }

        // Step1: 把每条车道线上有哪些像素点提取出来
        std::map<int, std::vector<Eigen::Vector2d> > lanes;
        int cols = im.cols;
        int rows = im.rows;

        for (int i = 480; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                int laneID = (int) im.at<uchar>(i, j);
                if (laneID > 0)
                {
                    lanes[laneID].push_back(Eigen::Vector2d(j, i));
                }
            }
        }

        // Step1.5: 如果车道线数量超过4条,那么就删除包含像素数量最少的那条车道线
        if (lanes.size() > 4)
        {
            int minNumPixel = UINT_MAX;
            int minLaneID = 0;
            for (auto iter = lanes.begin(); iter != lanes.end(); iter++)
            {
                if (iter->second.size() < minNumPixel)
                {
                    minLaneID = iter->first;
                    minNumPixel = iter->second.size();
                }
            }

            lanes.erase(minLaneID);
        }

        // Step2: 把每条车道线的像素集合拟合成一条直线，并求出车道线的上下两个端点
        std::map<double, std::pair<Eigen::Vector2d, Eigen::Vector2d> > mpairEndPoins;
        for (auto iter = lanes.begin(); iter != lanes.end(); iter++)
        {
            std::vector<Eigen::Vector2d> pts = iter->second;  // 属于这条车道线的所有像素点

            // step2.1: ceres直线拟合出[a,b]
            double a = 1;
            double b = 0;
            ceres::Problem problem;

            for (const Eigen::Vector2d &pt : pts)
            {
                ceres::CostFunction *pCostFunction = new ceres::AutoDiffCostFunction<
                        costFunctor, 1, 1, 1>(
                        new costFunctor(pt[0], pt[1]));
//                problem.AddResidualBlock(pCostFunction, new ceres::CauchyLoss(0.5), &a, &b);
                problem.AddResidualBlock(pCostFunction, nullptr, &a, &b);
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 100;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // step2.2:求出这条车道线的上下行边界[rowMin, rowMax]
            std::vector<double> rows;  // 表示一条车道线上的像素经过的所有row
            rows.reserve(pts.size());
            for (const Eigen::Vector2d &pt : pts)
            {
                rows.push_back(pt.y());
            }
            double minv = *std::min_element(rows.begin(), rows.end());
            minv = std::max(0.0, minv);
            double maxv = *std::max_element(rows.begin(), rows.end());
            maxv = std::min(800.0, maxv);

            // 求出车道线的上下两个端点
            double u1 = (minv - b) / a;    // 车道线上端点对应的u
            double u2 = (maxv - b) / a;    // 车道线下端点对应的u,容易出现小于0或大于1758等极端情况
            u2 = std::max(0.0, u2);
            u2 = std::min(1758.0, u2);
            Eigen::Vector2d p1(u1, minv);
            Eigen::Vector2d p2(u2, maxv);
            std::pair<Eigen::Vector2d, Eigen::Vector2d> pairEndPoints = std::make_pair(p1, p2);

            // step2.3: 求出这条车道线与图像下边界的截距
            double c = (800 - b) / a;
            mpairEndPoins[c] = pairEndPoints;
        }

        // Step3: 以上下两个端点为代表，输出每条车道线
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > vEndPointPairs;
        vEndPointPairs.reserve(mpairEndPoins.size());
        // 因为map会自动根据截距c排序,所有按顺序输出车道线自动是从左到右
        for (auto iter = mpairEndPoins.begin(); iter != mpairEndPoins.end(); iter++)
        {
            vEndPointPairs.push_back(iter->second);
        }
        vvEndPointPairs[ni] = vEndPointPairs;
    }

    std::ofstream fout("endPoints4.txt");
    for (int ni = 0; ni < vvEndPointPairs.size(); ++ni)
    {
        auto vEndPointPairs = vvEndPointPairs[ni];
        int nLanes = vEndPointPairs.size();  // 这张image上总共检测出了多少条车道线

        fout << nLanes << " ";
        for (int i = 0; i < nLanes; ++i)
        {
            Eigen::Vector2d p1 = vEndPointPairs[i].first;
            Eigen::Vector2d p2 = vEndPointPairs[i].second;
            fout << std::setprecision(5) << p1[0] << " " << p1[1] << " "
                 << p2[0] << " " << p2[1] << " ";
        }
        fout << endl;
    }
    fout.close();
}

void LoadSTCCImageLanes(const std::string &endPointsFile,
                        std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > > &vvEndPointPairs)
{
    std::ifstream fin(endPointsFile);
    std::string ptline;

    int nLanes = 0;
    double vmin = 0;
    double vmax = 0;
    double u1 = 0.0;
    double u2 = 0.0;
    while (getline(fin, ptline))
    {
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > vEndPointPairs;

        std::stringstream ss(ptline);
        ss >> nLanes;
        for (int i = 0; i < nLanes; ++i)
        {
            ss >> u1 >> vmin >> u2 >> vmax;
            vEndPointPairs.push_back(std::make_pair(Eigen::Vector2d(u1, vmin),
                                                    Eigen::Vector2d(u2, vmax)));
        }

        vvEndPointPairs.push_back(vEndPointPairs);
    }
}

void LoadSTCCLanePixels(const std::string &filepath,
                        std::vector<std::vector<std::vector<Eigen::Vector2d> > > &vvvLanePixels,
                        std::vector<std::vector<int> > &vvNumPixels,
                        int nImages, int nStart, int nEnd)
{
    vvvLanePixels.resize(nImages);
    vvNumPixels.resize(nImages);
    for (int ni = nStart; ni < nEnd; ++ni)
    {
        // Step0: 找到顺序排列的2D图像的文件名
        std::string filename = filepath + std::to_string(ni) + ".png";
        cv::Mat im = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        if (im.empty())
        {
            cerr << "cannot load 2D lanes at " << filename << endl;
            abort();
        }

        // Step1: 把每条车道线上有哪些像素点提取出来
        std::map<int, std::vector<Eigen::Vector2d> > lanes;
        int cols = im.cols;
        int rows = im.rows;

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                int laneID = (int) im.at<uchar>(i, j);
                if (laneID > 0)
                {
                    lanes[laneID].push_back(Eigen::Vector2d(j, i));
                }
            }
        }

        // Step1.5: 如果车道线数量超过4条,那么就删除包含像素数量最少的那条车道线
//        if (lanes.size() > 4)
//        {
//            int minNumPixel = UINT_MAX;
//            int minLaneID = 0;
//            for (auto iter = lanes.begin(); iter != lanes.end(); iter++)
//            {
//                if (iter->second.size() < minNumPixel)
//                {
//                    minLaneID = iter->first;
//                    minNumPixel = iter->second.size();
//                }
//            }
//
//            lanes.erase(minLaneID);
//        }

        vvNumPixels[ni].reserve(lanes.size());
        vvvLanePixels[ni].resize(lanes.size());
        int nLane = 0;
        for (auto iter = lanes.begin(); iter != lanes.end(); iter++)
        {
            std::vector<Eigen::Vector2d> vLanePts = iter->second;
            vvvLanePixels[ni][nLane++].assign(vLanePts.begin(), vLanePts.end());

//            // 求出各条车道线最上面与最下面两个端点,求出两点的像素距离
//            double upperV = 0.0;
//            double bottomV = UINT_MAX;
//            int upperID = 0;
//            int bottomID = 0;
//            for (int i = 0; i < vLanePts.size(); i++)
//            {
//                Eigen::Vector2d pt = vLanePts[i];
//
//                if (pt[1] > upperV)
//                {
//                    upperV = pt[1];
//                    upperID = i;
//                }
//                else if (pt[1] < bottomV)
//                {
//                    bottomV = pt[1];
//                    bottomID = i;
//                }
//            }
//
//            vvNumPixels[ni].push_back(int((vLanePts[upperID] - vLanePts[bottomID]).norm()));
            vvNumPixels[ni].push_back(vLanePts.size());
        }
    }
}


