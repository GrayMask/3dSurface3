#include <opencv2/core.hpp>;
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

using namespace std;
#ifndef VIRTUALCAMERA_H
#define VIRTUALCAMERA_H
class VirtualCamera
{

public:

	VirtualCamera(void);
	VirtualCamera(cv::Mat cameraMatrix_, cv::Mat distortion_, cv::Mat rotationMatrix_, cv::Mat translationVector_);
	~VirtualCamera(void);


	cv::Mat distortion;
	cv::Mat rotationMatrix;
	cv::Mat translationVector;
	cv::Mat cameraMatrix;

	cv::Point3f position;

	cv::Point2f fc;
	cv::Point2f cc;

	//int width;
	//int height;

};
#endif