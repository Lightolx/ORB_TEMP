cmake_minimum_required(VERSION 3.12)
project(line_pnp)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(XercesC REQUIRED)
include_directories(${XercesC_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})

add_executable(lane_pnp main.cpp Timer.cpp Timer.h OpenDRIVE_1.4H.hxx OpenDRIVE_1.4H.cxx
               Solver.h Types.h System.h System.cpp helper.h helper.cpp CorConverter.h
               CorConverter.cpp MapDrawer.h MapDrawer.cpp Viewer.h Viewer.cpp Track.cpp Track.h Frame.cpp Frame.h)
target_link_libraries(lane_pnp ${OpenCV_LIBS} ${CERES_LIBRARIES} ${XercesC_LIBRARY} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${GLUT_LIBRARY})