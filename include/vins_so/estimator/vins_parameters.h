#pragma once

#include "vins_so/utility/utility.h"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

const double FOCAL_LENGTH    = 460.0;
const int WINDOW_SIZE        = 10;
const int NUM_OF_F           = 1000;
const double LOOP_INFO_VALUE = 50.0;
const int MAX_NUM_OF_CAMS    = 22;

//#define DEPTH_PRIOR
//#define GT

#define UNIT_SPHERE_ERROR
#define INV_DEPTH 1

extern int NUM_OF_CAM;
extern int NUM_OF_STEREO;
extern double INIT_DEPTH;
extern double MAX_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector< Eigen::Matrix3d > RIC;
extern std::vector< Eigen::Vector3d > TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::vector< std::string > EX_CALIB_RESULT_PATHS;

extern std::string VINS_RESULT_PATH;
extern std::string VINS_FOLDER_PATH;

extern int LOOP_CLOSURE;
extern int MIN_LOOP_NUM;
extern int MAX_KEYFRAME_NUM;
extern std::string PATTERN_FILE;
extern std::string VOC_FILE;

extern std::vector< std::pair< int, int > > STEREO_CAM_IDS;
extern std::vector< std::string > CAM_NAMES;
extern std::vector< std::string > IMAGE_TOPICS;
extern std::vector< std::string > FEATURE_TOPICS;
extern std::string IMU_TOPIC;

extern bool DOWN_CAMERA_VISIABLE;
extern bool IN_AIR;
extern int MIN_VISIABLE_PARALLAX;

void
readParameters( ros::NodeHandle& n );

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE    = 7,
    SIZE_SPEED   = 3,
    SIZE_BIAS    = 6,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P  = 0, // position
    O_R  = 3, // oreitation
    O_V  = 6, // velocity
    O_BA = 9, // bias of accelerometer
    O_BG = 12 // bias of gyroscope
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
