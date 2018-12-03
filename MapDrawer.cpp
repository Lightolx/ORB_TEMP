//
// Created by lightol on 18-11-21.
//

#include "MapDrawer.h"

namespace LINE_PNP
{
void MapDrawer::GetCurrentOpenGLCameraMatrix(const Eigen::Matrix4d& pose,
                                             pangolin::OpenGlMatrix &M) {
    if (!pose.isZero()) {
        Eigen::Matrix3d Rwc = pose.topLeftCorner(3, 3);
        Eigen::Vector3d twc = pose.topRightCorner(3, 1);

        M.m[0] = Rwc(0, 0);
        M.m[1] = Rwc(1, 0);
        M.m[2] = Rwc(2, 0);
        M.m[3] = 0.0;

        M.m[4] = Rwc(0, 1);
        M.m[5] = Rwc(1, 1);
        M.m[6] = Rwc(2, 1);
        M.m[7] = 0.0;

        M.m[8] = Rwc(0, 2);
        M.m[9] = Rwc(1, 2);
        M.m[10] = Rwc(2, 2);
        M.m[11] = 0.0;

        M.m[12] = twc(0);
        M.m[13] = twc(1);
        M.m[14] = twc(2);
        M.m[15] = 1.0;
    } else {
        M.SetIdentity();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc,
                                  const Eigen::Vector3f &rgb, double w) {
//    const float &w = 2.0;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(3);
    // Red:current frame
    glColor3f(rgb[0], rgb[1], rgb[2]);


    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::DrawTrajectory() const {
    // draw all trajectories
    glLineWidth(1);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    for (size_t i = 2; i < mvGtTrajPosition.size(); i++) {
        auto p1 = mvGtTrajPosition[i-1];
        auto p2 = mvGtTrajPosition[i];
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();

    glLineWidth(1);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    for (size_t i = 2; i < mvNoisedTrajPosition.size(); i++) {
        auto p1 = mvNoisedTrajPosition[i-1];
        auto p2 = mvNoisedTrajPosition[i];
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();

    glLineWidth(1);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    for (size_t i = 2; i < mvOptimizedTrajPosition.size(); i++) {
        auto p1 = mvOptimizedTrajPosition[i-1];
        auto p2 = mvOptimizedTrajPosition[i];
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();
}

void MapDrawer::DrawHDMap() {
    if (!(mpHDmap))  {  // 凭什么run()的时候就会出现空指针，debug的时候就不会出现？
        return;
    }

    OpenDRIVE::road_sequence roads = mpHDmap->road();

    std::ifstream fin("color.txt");
    std::string ptline;
    Eigen::Vector3d RGB = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> vRGBs;
    while (getline(fin, ptline)) {
        std::stringstream ss(ptline);
        ss >> RGB[0] >> RGB[1] >> RGB[2];
        vRGBs.push_back(RGB);
    }

    for (int i = 0; i < roads.size(); ++i) {
        if (i < 6) {
            continue;
        }
//        else if (i == 11)
//        {
//            continue;
//        }

        road::lanes_type driveway = roads[i].lanes();

        // Step1: lanes
        lanes::laneSection_sequence laneSections = driveway.laneSection();

        for (lanes::laneSection_type laneSection : laneSections) {
            // 一个laneSection里面总共有3种5条线:
            // 道路可行驶区域左右边界 leftBoundary, rightBoundary；
            // 所有车道的中心线，也就是reference line,有且仅有一条；
            // 左边车道和右边车道的车道线 leftLanes, rightLanes, 比如3车道的话左右就是各两条车道线;
            laneSection::leftBoundary_type leftBoundary = laneSection.leftBoundary();
            laneSection::rightBoundary_type rightBoundary = laneSection.rightBoundary();
            laneSection::left_type leftLanes = laneSection.left();
            laneSection::center_type centerLane = laneSection.center();
            laneSection::right_type rightLanes = laneSection.right();

            // Step1: 可行驶区域左右边界
            roadBoundary::geometry_type geometry = leftBoundary.geometry();
            DrawHDPoints(geometry, 1.0, 0.0, 0.0);
            geometry = rightBoundary.geometry();
            DrawHDPoints(geometry, 1.0, 0.0, 0.0);

            // Step2: 车道线
            // 车道的中心线
            lane::border_type border = centerLane.lane().border();
            geometry = border.geometry();
            Eigen::Vector3d colors = (Eigen::Vector3d::Random() + Eigen::Vector3d::Ones()) / 2;

            double r = vRGBs[i][0];
            double g = vRGBs[i][1];
            double b = vRGBs[i][2];
//            DrawHDPoints(geometry, r, g, b);
            DrawHDPoints(geometry, 0.0, 1.0, 0.0);

            std::vector<halfRoad::lane_type> allLane;
            //左边车道的车道线
            halfRoad::lane_sequence lanesInLeft = leftLanes.lane();
            allLane.insert(allLane.end(), lanesInLeft.begin(), lanesInLeft.end());
            //右边车道的车道线
            halfRoad::lane_sequence lanesInRight = rightLanes.lane();
            allLane.insert(allLane.end(), lanesInRight.begin(), lanesInRight.end());

            for (halfRoad::lane_type lane : allLane) {
                lane::border_type border = lane.border();
                border::geometry_type geometry = border.geometry();
                DrawHDPoints(geometry, 1.0, 1.0, 1.0);

                //  这个leftBorder是干嘛的
                lane::leftBorder_optional leftBorder = lane.leftBorder();
                if (leftBorder.present()) {
                    geometry = leftBorder.get().geometry();
                    DrawHDPoints(geometry, 1.0, 1.0, 1.0);
                }
            }
        }

//        // Step3: 路口起始线和停止线objects, startline and stopline
//        objects::object_sequence objects = road.objects().object();
//        for (objects::object_type object : objects) {
//            if (object.type().present()) {
//                if (object.type().get() == "startline" || object.type().get() == "stopline") {
//                    DrawHDPoints(object.geometry(), 0.0, 1.0, 0.0);
//                }
//            }
//        }
    }

//    // Step4: junctions
//    OpenDRIVE::junction_sequence junctions = mpHDmap->junction();
//    for (OpenDRIVE::junction_type junction : junctions) {
//        boundary::geometry_type geometry = junction.boundary().geometry();
//        DrawHDPoints(geometry, 1.0, 1.0, 0.0);
//    }
      // Step5: 当前相机可能看到的所有车道
//        DrawRoiLanes(0.0, 1.0, 0.0);

}

void MapDrawer::DrawHDPoints(roadBoundary::geometry_type geometry, float r, float g, float b) {
    double x(0.0), y(0.0), z(0.0);
    std::vector<Eigen::Vector3d> pts;

    geometry::point_sequence points = geometry.point();

    for (geometry::point_type point : points) {
        x = *point.x();
        y = *point.y();
        z = *point.z();
        pts.push_back(Eigen::Vector3d(x, y, z));
    }

//    ConvertGPS2UTM(pts);

    int numPts = pts.size();

    glLineWidth(2.0);
    glColor4f(r, g, b, 0.6f);
    glBegin(GL_LINES);

    for (int i = 0; i < numPts - 1; ++i) {
        glVertex3f(pts[i](0), pts[i](1), pts[i](2));
        glVertex3f(pts[i+1](0), pts[i+1](1), pts[i+1](2));
    }

    glEnd();
}

void MapDrawer::DrawRoiLanes(float r, float g, float b)
{
    for (auto pts : mvvLanePts)
    {
        int numPts = pts.size();

        glLineWidth(4.0);
        glColor4f(r, g, b, 0.6f);
        glBegin(GL_LINES);

        for (int i = 0; i < numPts - 1; ++i) {
            glVertex3f(pts[i](0), pts[i](1), pts[i](2));
            glVertex3f(pts[i+1](0), pts[i+1](1), pts[i+1](2));
        }

        glEnd();
    }
}

}


