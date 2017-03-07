#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>;
#include "Tools.h"
#include "Const.h"
#include "Path.h"

using namespace std;


#define cvQueryHistValue_1D( hist, idx0 ) \
    ((float)cvGetReal1D( (hist)->bins, (idx0)))

int Tools::copyFile(const cv::String fromFile, const cv::String toFile) {
	fstream in(fromFile, ios::in | ios::binary);
	if (!in)
	{
		cerr << "open " << fromFile << " failed" << endl;
		in.close();
		return 0;
	}

	fstream out(toFile, ios::out | ios::binary);
	if (!out)
	{
		cerr << "open " << toFile << " failed" << endl;
		out.close();
		return 0;
	}

	char temp;
	while (in.get(temp))
	{
		out << temp;
	}
	out << endl;
	cout << "success!" << endl;
	in.close();
	out.close();
}

int Tools::writePic(const cv::Mat& im, const cv::String& fname)
{
	ofstream ouF;
	ouF.open(fname.c_str());
	if (!ouF)
	{
		cerr << "failed to open the file : " << fname << endl;
		return 0;
	}
	for (int r = 0; r < im.rows; r++)
	{
		for (int c = 0; c < im.cols; c++) {
			ouF << im.at<float>(r, c);
			ouF << " ";
		}
		ouF << "\n";
	}
	ouF.close();
	return 1;
}

int Tools::writeGroupNumFile(cv::String& filePath, const int groupNum)
{
	ofstream out(filePath);
	if (out.is_open())
	{
		out << groupNum;
		out.close();
	}
	else {
		return -1;
	}
	return 1;
}

int Tools::readGroupNumFile(cv::String filePath, int& groupNum)
{
	ifstream in(filePath);
	if (!in.is_open())
	{
		cout << "Error opening file";
		return -1;
	}
	in >> groupNum;
	return 1;
}

int Tools::readStringList(cv::String& filename, vector<string>& l) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "failed to open " << filename << endl;
		return -1;
	}
	cv::FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != cv::FileNode::SEQ)
	{
		cerr << "cam 1 images are not a sequence! FAIL" << endl;
		return -1;
	}
	cv::FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
	{
		l.push_back((string)*it);
	}
	return 1;
}

bool goWithLine(ifstream & in, int line)
{
	int i;
	char buf[1024];
	for (i = 0; i < line; i++)
	{
		if (!in.getline(buf, sizeof(buf))) {
			return false;
		}
	}
	return true;
}

int Tools::getSFMResult(const int count, cv::Mat& R, cv::Mat& T) {
	R.create(3,3,CV_64F);
	T.create(3, 1, CV_64F);
	double num;
	ifstream in(root_dir + sfm_file);
	if (!in.is_open())
	{
		cout << "Error opening file";
		return 0;
	}
	ostringstream countStr;
	countStr << count;
	cv::String targetPathStr = countStr.str() + imgType;
	const char* targetPath = targetPathStr.c_str();
	// goto line 19
	if (goWithLine(in, 19)) {
		do {
			char nowPath[1024];
			in.getline(nowPath, sizeof(nowPath));
			char *last = strrchr(nowPath, '\\') +1 ;
			if (strcmp(targetPath, last) == 0) {
				goWithLine(in, 2);
				for (int i = 0; i < 3; i++) {
					in >> num;
					T.at<double>(i, 0) = num;
				}
				goWithLine(in, 4);
				for (int i = 0; i < 9; i++) {
					in >> num;
					R.at<double>(i / 3, i % 3) = num;
				}
				return 1;
			}
		} while (goWithLine(in, 13));
	}
	return 0;
}

void Tools::myCalcHist(cv::Mat gray_plane)
{
	IplImage *src;
	src = &IplImage(gray_plane);
	int hist_size = 256;
	int hist_height = 256;
	float range[] = { 0,255 };
	float* ranges[] = { range };
	CvHistogram* gray_hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
	cvCalcHist(&src, gray_hist, 0, 0);
	cvNormalizeHist(gray_hist, 1.0);

	int scale = 2;
	IplImage* hist_image = cvCreateImage(cvSize(hist_size*scale, hist_height), 8, 3);
	cvZero(hist_image);
	float max_value = 0;
	cvGetMinMaxHistValue(gray_hist, 0, &max_value, 0, 0);

	for (int i = 0; i < hist_size; i++)
	{
		float bin_val = cvQueryHistValue_1D(gray_hist, i);
		int intensity = cvRound(bin_val*hist_height / max_value);
		cvRectangle(hist_image,
			cvPoint(i*scale, hist_height - 1),
			cvPoint((i + 1)*scale - 1, hist_height - intensity),
			CV_RGB(255, 255, 255));
	}
	cvGetMinMaxHistValue(gray_hist, 0, &max_value, 0, 0);
	cvNamedWindow("H-S Histogram", 1);
	cvShowImage("H-S Histogram", hist_image);
}

void Tools::saveCamsPixelsForReconstuction(vector<cv::Point> *camPixels, cv::String path) {
	ofstream ouF;
	ouF.open(path.c_str());
	int sz = proj_width * proj_height;
	for (int i = 0; i < sz; i++) {
		vector<cv::Point> points = camPixels[i];
		int pointSz = points.size();
		for (int j = 0; j < pointSz; j++) {
			cv::Point point = points[j];
			ouF << point.x << " " << point.y << " ";
		}
		ouF << "-1 ";
	}
	ouF.close();
}

void Tools::loadCamsPixelsForReconstuction(vector<cv::Point>* camPixels, cv::String path) {
	ifstream inF;
	inF.open(path.c_str());
	int sz = proj_width * proj_height;
	camPixels = new vector<cv::Point>[sz];
	string line;
	int x;
	int y;
	for (int i = 0; i < sz; i++) {
		vector<cv::Point> points;
		points.resize(0);
		while (1) {
			inF >> x;
			if (x == -1) {
				break;
			}
			inF >> y;
			cv::Point point(x, y);
			points.push_back(point);
		}
		camPixels[i]= points;
	}
	inF.close();
}