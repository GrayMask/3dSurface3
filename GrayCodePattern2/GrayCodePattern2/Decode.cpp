#include <io.h>
#include <direct.h>
#include <stdio.h>
#include <cmath>
#include "Decode.h"
#include "Tools.h"
#include "Path.h"
#include "Const.h"
#include "FeaturePoints.h"
#include "Utilities.h"
#include <iostream>
#include <windows.h>
#include <map>

#define MAX_THREADS 6

void makeSfmDir(int const numOfProjectorGroup) {
	// make a new dir
	if (_access((root_dir + sfm_dir).c_str(), 6) == -1)
	{
		_mkdir((root_dir + sfm_dir).c_str());
	}
	int whiteImgIndex = log(proj_width) / log(2) * 4 + 1;
	ostringstream name;
	name << whiteImgIndex;
	cv::String imagesName = images_file + name.str() + imgType;
	cv::String imagesDir1;
	int numOfImageGroup;
	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup, imagesDir1);
		for (int j = 0; j < numOfImageGroup; j++) {
			if (i == 0 || j != 0) { // expect the first image of the projecter position from 1
									// copy images to the new dir
				char* imagesGroupDirTemp1 = new char[images_group_dir_length];
				sprintf(imagesGroupDirTemp1, images_group_dir, j);
				cv::String imagesDir2 = imagesDir1 + cv::String(imagesGroupDirTemp1) + imagesName;
				ostringstream num;
				num << count;
				Tools::copyFile(imagesDir2, root_dir + sfm_dir + num.str() + imgType);
				count++;
			}
		}
	}
}

bool getProjPixel_v2(cv::InputArrayOfArrays patternImages, int x, int y, cv::Point &projPix)
{
	std::vector<cv::Mat>& _patternImages = *(std::vector<cv::Mat>*) patternImages.getObj();
	std::vector<uchar> grayCol;
	std::vector<uchar> grayRow;
	int numOfColBits = log(proj_width) / log(2);
	int numOfRowBits = log(proj_height) / log(2);
	bool error = false;
	int xDec, yDec;
	// process column images
	for (size_t count = 0; count < numOfColBits; count++)
	{
		// get pixel intensity for regular pattern projection and its inverse
		double val1 = _patternImages[count * 2].at<uchar>(cv::Point(x, y));
		double val2 = _patternImages[count * 2 + 1].at<uchar>(cv::Point(x, y));
		// check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
		if (abs(val1 - val2) < white_thresh)
			error = true;
		// determine if projection pixel is on or off
		if (val1 > val2)
			grayCol.push_back(1);
		else
			grayCol.push_back(0);
	}
	xDec = Utilities::grayToDec_v2(grayCol);
	// process row images
	for (size_t count = 0; count < numOfRowBits; count++)
	{
		// get pixel intensity for regular pattern projection and its inverse
		double val1 = _patternImages[count * 2 + numOfColBits * 2].at<uchar>(cv::Point(x, y));
		double val2 = _patternImages[count * 2 + numOfColBits * 2 + 1].at<uchar>(cv::Point(x, y));
		// check if the intensity difference between the values of the normal and its inverse projection image is in a valid range
		if (abs(val1 - val2) < white_thresh)
			error = true;
		// determine if projection pixel is on or off
		if (val1 > val2)
			grayRow.push_back(1);
		else
			grayRow.push_back(0);
	}
	yDec = Utilities::grayToDec_v2(grayRow);
	if ((yDec >= proj_height || xDec >= proj_width))
	{
		error = true;
	}
	projPix.x = xDec;
	projPix.y = yDec;
	return error;
}


void decode(cv::Mat& shadowMask, vector<cv::Mat>& captured_pattern, vector<vector<cv::Point>>& camPixels)
{
	cv::Point projPixel;
	// Storage for the pixels of the two cams that correspond to the same pixel of the projector
	camPixels.resize(proj_width * proj_height);
	for (int i = 0; i < cam_width; i++)
	{
		for (int j = 0; j < cam_height; j++)
		{
			//if the pixel is not shadowed, reconstruct
			if (shadowMask.at<uchar>(j, i))
			{
				//for a (x,y) pixel of the camera returns the corresponding projector pixel by calculating the decimal number
				bool error = getProjPixel_v2(captured_pattern, i, j, projPixel);
				if (error)
				{
					continue;
				}
				camPixels[projPixel.x * proj_height + projPixel.y].push_back(cv::Point(i, j));
			}
		}
	}
}

void computeShadows(cv::Mat& shadowMask, cv::Mat& whiteImg, cv::Mat& blackImg)
{
	std::cout << "Estimating Shadows...";

	int w = cam_width;
	int h = cam_height;

	shadowMask.release();

	shadowMask = cv::Mat(h, w, CV_8U, cv::Scalar(0));

	for (int i = 0; i<w; i++)
	{
		for (int j = 0; j<h; j++)
		{
			float blackVal, whiteVal;
			blackVal = (float)Utilities::matGet2D(blackImg, i, j);
			whiteVal = (float)Utilities::matGet2D(whiteImg, i, j);
			if (whiteVal - blackVal > black_thresh) {
				Utilities::matSet2D(shadowMask, i, j, 1);
			}
			else {
				Utilities::matSet2D(shadowMask, i, j, 0);
			}
		}
	}
	std::cout << "done!\n";
}

typedef struct para {
	int numOfGroup;
	cv::String imagesDir1;
};

void getAveragePixels(vector<vector<cv::Point>>& camsPixels, vector<Tools::PointWithCode>& camsAvgPixels) {
	int sz = camsPixels.size();
	for (int i = 0; i < sz; i++) {
		cv::Point2f avg = 0;
		int numOFPoint = camsPixels[i].size();
		Tools::PointWithCode pwc;
		if (numOFPoint > 0) {
			for (int m = 0; m < numOFPoint; m++) {
				avg += (cv::Point2f)camsPixels[i][m];
			}
			avg /= numOFPoint;
			pwc.point = avg;
			pwc.code = i;
		}
		else {
			pwc.code = -1;
		}
		camsAvgPixels.push_back(pwc);
	}
}

DWORD WINAPI getFeaturePoints(LPVOID pM) {
	para* p = (para *)pM;
	ostringstream numStr;
	numStr << p->numOfGroup;
	cout << p->imagesDir1 + " " + numStr.str() << " start!" << endl;
	vector<vector<cv::Point>> camsPixels;
	vector<cv::String> camFolder;
	camFolder.resize(0);
	char* imagesGroupDirTemp = new char[images_group_dir_length];
	sprintf(imagesGroupDirTemp, images_group_dir, p->numOfGroup);
	cv::String filename = p->imagesDir1 + cv::String(imagesGroupDirTemp) + imagesName_file;
	Tools::readStringList(filename, camFolder);
	vector<cv::Mat> captured_pattern;
	Utilities::loadCamImgs(p->imagesDir1 + cv::String(imagesGroupDirTemp), camFolder, captured_pattern);
	cv::Mat shadowMask;
	int numOfIimg = captured_pattern.size();
	computeShadows(shadowMask, captured_pattern[numOfIimg - 2], captured_pattern[numOfIimg - 1]);
	Utilities::writeShadowMask(shadowMask, p->imagesDir1 + shadowMask_file + numStr.str() + imgType);

	//decodePaterns(shadowMask, captured_pattern, camsPixels);
	decode(shadowMask, captured_pattern, camsPixels);
	vector<Tools::PointWithCode> camsAvgPixels;
	getAveragePixels(camsPixels, camsAvgPixels);
	Tools::saveCamsPixelsForReconstuction(camsAvgPixels, p->imagesDir1 + numStr.str() + decodefileType);
	//delete[] camsPixels;
	delete[] imagesGroupDirTemp;
	cout << p->imagesDir1 + " " + numStr.str() << " finish!" << endl;
	return 0;
}

void waitThread(HANDLE hArray[], int hp= MAX_THREADS) {
	WaitForMultipleObjects(hp, hArray, true, INFINITE);
	for (int i = 0; i < MAX_THREADS; i++) {
		if (hArray[i] != NULL) {
			CloseHandle(hArray[i]);
			hArray[i] = NULL;
		}
	}
}


void Decode::executeDecoding() {
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	makeSfmDir(numOfProjectorGroup);

	cv::String imagesDir1;
	int numOfImageGroup;
	char* projectorGroupDirTemp = new char[projector_group_dir_length];


	HANDLE hArray[MAX_THREADS];
	para p[MAX_THREADS];
	int hp = 0;
	int j,i;
	for (i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup, imagesDir1);
		for (j = 0; j < numOfImageGroup; j++)
		{
			p[hp].imagesDir1 = cv::String(imagesDir1.c_str());
			p[hp].numOfGroup = j;
			hArray[hp] = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)getFeaturePoints, (LPVOID)&p[hp], 1, 0);
			hp++;
			if (hp == MAX_THREADS) {
				waitThread(hArray);
				hp = 0;
			}
		}
	}
	waitThread(hArray, hp);
}
