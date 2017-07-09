#include <opencv2/core.hpp>;
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
using namespace std;
#ifndef TOOLS_H
#define TOOLS_H
class Tools
{
public:
	struct PointWithCode {
		cv::Point2f point;
		int indexInSiftFile;
		int code;
		bool isMerged; // Is merged while calculating the code map
	};

	struct ThreeDPoint {
		cv::Point3d point;
		int r;
		int g;
		int b;
	};

	static int copyFile(const cv::String fromName, const cv::String toName);

	static int writePic(const cv::Mat& im, const cv::String& fname);

	static int writeGroupNumFile(cv::String& filePath, const int groupNum);

	static int readGroupNumFile(cv::String filePath, int& groupNum);

	static int readStringList(cv::String& filename, vector<cv::String>& l);

	static int getSFMResult(const int count, cv::Mat& R, cv::Mat& T);

	static void myCalcHist(cv::Mat gray_plane);

	static bool goWithLine(ifstream & in, int line);

	static bool goWithStep(ifstream & in, int step);

	static void saveCamsPixelsForReconstuction(vector<PointWithCode> camPixels, cv::String path);

	static void loadCamsPixelsForReconstuction(vector<PointWithCode>& camPixels, cv::String path, int& lastIdx);

	static void getAllFiles(cv::String path, vector<cv::String>& files);

	static void readPointCloudFromNvm(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

	static void savePointCloudInPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal);
};
#endif