#include <opencv2/core.hpp>;
using namespace std;
#ifndef TOOLS_H
#define TOOLS_H
class Tools
{
public:
	static int copyFile(const cv::String fromName, const cv::String toName);

	static int writePic(const cv::Mat& im, const cv::String& fname);

	static int writeGroupNumFile(cv::String& filePath, const int groupNum);

	static int readGroupNumFile(cv::String filePath, int& groupNum);

	static int readStringList(cv::String& filename, vector<string>& l);

	static int getSFMResult(const int count, cv::Mat& R, cv::Mat& T);

	static void myCalcHist(cv::Mat gray_plane);

	static void saveCamsPixelsForReconstuction(vector<cv::Point> *camPixels, cv::String path);

	static void loadCamsPixelsForReconstuction(vector<vector<cv::Point>>& camPixels, cv::String path);
};
#endif