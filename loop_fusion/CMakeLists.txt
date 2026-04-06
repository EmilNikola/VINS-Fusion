cmake_minimum_required(VERSION 2.8.3)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
# -DEIGEN_USE_MKL_ALL
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

option(LOOP_FUSION_STANDALONE "Build loop_fusion without ROS/catkin" ON)

# We still use catkin to obtain camodocal/camera_models in standalone mode,
# but we avoid roscpp/cv_bridge/etc.
if(LOOP_FUSION_STANDALONE)
    find_package(catkin REQUIRED COMPONENTS
            camera_models
    )
    include_directories(${catkin_INCLUDE_DIRS})
else()
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            nav_msgs
            camera_models
            cv_bridge
            roslib
    )
    catkin_package()
    include_directories(${catkin_INCLUDE_DIRS})
endif()

find_package(OpenCV 4)
find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(loop_fusion_node
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
)

# In standalone mode, catkin_LIBRARIES will contain camera_models/camodocal,
# which provides camodocal::CameraFactory symbols.
if(LOOP_FUSION_STANDALONE)
    target_link_libraries(loop_fusion_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
    install(TARGETS loop_fusion_node RUNTIME DESTINATION bin)
else()
    target_link_libraries(loop_fusion_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
    install(TARGETS loop_fusion_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()