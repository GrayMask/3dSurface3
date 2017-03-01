#include "VirtualCamera.h"
#include <opencv2/core.hpp>;
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

using namespace std;


VirtualCamera::VirtualCamera(void)
{
}

VirtualCamera::VirtualCamera(cv::Mat cameraMatrix_, cv::Mat distortion_, cv::Mat rotationMatrix_, cv::Mat translationVector_)
{
	cameraMatrix_.convertTo(cameraMatrix, CV_32FC1);
	distortion_.convertTo(distortion, CV_32FC1);
	rotationMatrix_.convertTo(rotationMatrix, CV_32FC1);
	translationVector_.convertTo(translationVector, CV_32FC1);
	cc.x = cameraMatrix.at<float>(0, 2);
	cc.y = cameraMatrix.at<float>(1, 2);
	fc.x = cameraMatrix.at<float>(0, 0);
	fc.y = cameraMatrix.at<float>(1, 1);
}

VirtualCamera::~VirtualCamera(void)
{

}