#include <opencv2/core.hpp>;

extern const int proj_width;
extern const int proj_height;

extern const int cam_width;
extern const int cam_height;
extern const int cam_exp;

extern const bool isStereoCamera;

// dir
extern const cv::String exprNum;
extern const cv::String expr_dir;
extern const cv::String calib_file;
extern const cv::String disparityMap_file;
extern const cv::String projectorGroupNum_file;
extern const cv::String imageGroupNum_file;
extern const char* images_group_dir;
extern const int images_group_dir_length;
extern const char* projector_group_dir;
extern const int projector_group_dir_length;
extern const cv::String images_file;
extern const cv::String imagesName_file;
extern const cv::String imgType;
extern const cv::String sfm_dir;
extern const cv::String sfm_file;
extern const cv::String ply_file;

extern const bool isThresh;
extern const size_t white_thresh;
extern const size_t black_thresh;

extern const bool isRemap;

// Optimize disparity map
extern const bool isOptimize;
extern const float downPortion;
extern const float upPortion;

extern const bool isShowResult;

extern const bool isUnderWorld;