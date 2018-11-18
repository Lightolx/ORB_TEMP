//
// Created by lightol on 18-11-18.
//

#ifndef ORB_TEMP_VIEWER_H
#define ORB_TEMP_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>
#include "MapDrawer.h"
#include "Track.h"

namespace LINE_PNP
{
class Viewer {
public:
    explicit Viewer(MapDrawer *pMapDrawer) {
        mpMapDrawer = pMapDrawer;}

    void Run();

    void SetTracker(Track* pTracker) {
        mpTracker = pTracker;
    }

private:
    MapDrawer* mpMapDrawer;
    Track* mpTracker;
};
}

#endif  // ORB_TEMP_VIEWER_H
