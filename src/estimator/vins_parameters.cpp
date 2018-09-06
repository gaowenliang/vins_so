#include "vins_so/estimator/vins_parameters.h"

double INIT_DEPTH;
double MAX_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector< Eigen::Matrix3d > RIC;
std::vector< Eigen::Vector3d > TIC;

Eigen::Vector3d G{ 0.0, 0.0, 9.8 };

int NUM_OF_CAM    = 2;
int NUM_OF_STEREO = 1;

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
std::vector< std::string > EX_CALIB_RESULT_PATHS;
std::string VINS_RESULT_PATH;
int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
// std::string CAM_NAMES;
std::vector< std::string > CAM_NAMES;
std::vector< std::pair< int, int > > STEREO_CAM_IDS;

std::string PATTERN_FILE;
std::string VOC_FILE;
std::vector< std::string > IMAGE_TOPICS;
std::vector< std::string > FEATURE_TOPICS;
// std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string WHEEL_MEC_TOPIC;
int IMAGE_ROW, IMAGE_COL;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;

bool DOWN_CAMERA_VISIABLE = false;
bool IN_AIR               = false;
int MIN_VISIABLE_PARALLAX;
