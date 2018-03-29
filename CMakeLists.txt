cmake_minimum_required(VERSION 2.8.3)
project(vins_so)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_model
    code_utils
    )

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vins
    )
include_directories("include")


set(slideWindow_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/estimator/slideWindow/slidewindowbase.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/slideWindow/slidewindowimu.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/slideWindow/slidewindowpose.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/slideWindow/slidewindowposevel.cpp
    )
set(factor_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/estimator/factor/pose_local_parameterization.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/factor/stereoprojectionfactor.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/factor/ProjectionFactorSingleCam.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/factor/ProjectionFactorMultiCam.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/factor/marginalization_factor.cpp
    )
set(feature_manager_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/estimator/feature_manager/feature_manager.cpp
    )
set(initial_LIB_SOURCE_FILES
    # initial lib
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialSys/initialsys.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialBase/initialgen.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialBase/initialbase.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialMono/initialmonovio.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialMono/solve_5pts.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialMono/initial_sfm.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialStereoVio/initialstereovio.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialStereoPnP/initialstereoviopnp.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialEx/exParam.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialEx/InitialEx.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialEx/InitialExRotationCamImu.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/InitialEx/InitialExRotationCamCam.cpp
    ${PROJECT_SOURCE_DIR}/src/estimator/initial_lib/VisaulImuAligment/visaulimualigment.cpp
    )
set(loop_closure_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DBoW/BowVector.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DBoW/FBrief.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DBoW/QueryResults.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DUtils/Random.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DUtils/Timestamp.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/DVision/BRIEF.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/ThirdParty/VocabularyBinary.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/loop_closure.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/keyframe.cpp
    ${PROJECT_SOURCE_DIR}/src/loop-closure/keyframe_database.cpp
    )

#add_library(vins STATIC
#    ${slideWindow_LIB_SOURCE_FILES}
#    ${factor_LIB_SOURCE_FILES}
#    ${feature_manager_LIB_SOURCE_FILES}
#    ${initial_LIB_SOURCE_FILES}
#    )
#target_link_libraries(vins ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(vins_so
    src/estimator_node.cpp
    src/estimator/vins_parameters.cpp
    src/estimator/estimator.cpp

    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp

    ${slideWindow_LIB_SOURCE_FILES}
    ${factor_LIB_SOURCE_FILES}
    ${feature_manager_LIB_SOURCE_FILES}
    ${initial_LIB_SOURCE_FILES}
#    ${loop_closure_LIB_SOURCE_FILES}
    )

target_link_libraries(vins_so dw ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(vins_test
    src/test.cpp
    )

target_link_libraries(vins_test dw ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
