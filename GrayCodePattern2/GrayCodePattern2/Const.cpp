#include "Const.h";
#include <opencv2/core.hpp>;

extern const int proj_width = 1024;
extern const int proj_height = 768;

extern const int cam_width = 1920;//1920;//4032;// 1280;//800;//4896;
extern const int cam_height = 1080;//1080;//3024;// 720;//600;//3264;
extern const int cam_exp = 50;

extern const float thePositionInPeriod = 0.5; // percent, the sample position in a period of pattern
extern const float periodOfEachPattern = 0.1; // second

// const bool isStereoCamera = false;

// dir
extern const cv::String exprNum = "7";
extern const cv::String expr_dir = "expr" + exprNum + "\\";
extern const cv::String calib_file = "calibration_result\\camera4.xml";
extern const cv::String disparityMap_file = "matlab\\disparityMap" + exprNum;
extern const cv::String projectorGroupNum_file = expr_dir + "projectorGroupNum.txt";
extern const cv::String imageGroupNum_file = "imageGroupNum.txt";
extern const char* images_group_dir = "partten_images%02d\\";
extern const int images_group_dir_length = 18;
extern const char* projector_group_dir = "projector_position%02d\\";
extern const int projector_group_dir_length = 22;
extern const cv::String images_file = "pattern_im";
extern const cv::String imagesName_file = "imgName.yaml";
extern const cv::String imgType = ".jpg";
extern const cv::String decodefileType = ".pxl";
extern const cv::String sfm_dir = expr_dir + "sfm_images\\";
extern const cv::String sfm_file = sfm_dir + "d.nvm.cmvs\\00\\cameras_v2.txt";
extern const cv::String nvm_file = sfm_dir + "d.nvm";
extern const cv::String shadowMask_file = "shadowMask";
extern const cv::String plyFileName = "pointcloud.ply";
//extern const cv::String ply_file = "pointCloud.ply";

//extern const char* iphone_file = "IMG_%04d.JPG";
//extern const int iphone_file_length = 12;
extern const cv::String iphone_vedio_type = ".mov";
extern const cv::String new_iphone_vedio_file = "v" + iphone_vedio_type;
extern const cv::String iphone_video_dir = "v\\";
extern const int iphone_vedio_start_thresh = 5;

extern const bool isThresh = true;
extern const size_t white_thresh = 5;
extern const size_t black_thresh = 40;

extern const float mapping_thresh = 0.5;

extern const cv::Point nullPoint = -1;

// loop closing
extern const double initial_max_distance_thresh = 0.02;

//extern const bool isRemap = true;

// Optimize disparity map
//extern const bool isOptimize = true;
//extern const float downPortion = 0.001;
//extern const float upPortion = 0.999;

//extern const bool isShowResult = false;

//extern const bool isUnderWorld = false;