#include <io.h>
#include <direct.h>
#include <stdio.h>
#include <cmath>
#include "Sfm.h"
#include "Tools.h"
#include "Path.h"
#include "Const.h"
#include "FeaturePoints.h"
#include "Utilities.h"
#include <iostream>

using namespace std;

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
	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		char* projectorGroupDirTemp = new char[projector_group_dir_length];
		sprintf(projectorGroupDirTemp, projector_group_dir, i);
		cv::String imagesDir1 = root_dir + expr_dir + cv::String(projectorGroupDirTemp);
		int numOfImageGroup;
		Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfImageGroup);
		for (int j = 0; j < numOfImageGroup; j++) {
			if (i==0 || j!=0) { // expect the first image of the projecter position from 1
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

			if (whiteVal - blackVal > black_thresh)
			{
				Utilities::matSet2D(shadowMask, i, j, 1);
			}
			else
			{
				Utilities::matSet2D(shadowMask, i, j, 0);
			}
		}
	}
	std::cout << "done!\n";
}

//for a (x,y) pixel of the camera returns the corresponding projector pixel
bool getProjPixel(int x, int y, cv::Point &p_out, vector<cv::Mat>& captured_pattern)
{
	vector<bool> grayCol;
	vector<bool> grayRow;
	int numOfColBits = 10;
	int numOfRowBits = 10;

	bool error = false;
	//int error_code = 0;
	int xDec, yDec;

	//prosses column images
	for (int count = 0; count<numOfColBits; count++)
	{
		//get pixel intensity for regular pattern projection and it's inverse 
		double val1, val2;
		val1 = Utilities::matGet2D(captured_pattern[count * 2], x, y);
		val2 = Utilities::matGet2D(captured_pattern[count * 2 + 1], x, y);

		//check if intensity deference is in a valid rage
		if (abs(val1 - val2) < white_thresh)
			error = true;

		//determine if projection pixel is on or off
		if (val1>val2)
			grayCol.push_back(1);
		else
			grayCol.push_back(0);

	}

	xDec = Utilities::grayToDec(grayCol);

	//prosses row images
	for (int count = 0; count < numOfRowBits; count++)
	{

		double val1, val2;

		val1 = Utilities::matGet2D(captured_pattern[count * 2 + numOfColBits * 2], x, y);
		val2 = Utilities::matGet2D(captured_pattern[count * 2 + numOfColBits * 2 + 1], x, y);

		if (abs(val1 - val2) < white_thresh)  //check if the difference between the values of the normal and it's inverce projection image is valid
			error = true;

		if (val1 > val2)
			grayRow.push_back(1);
		else
			grayRow.push_back(0);

	}

	//decode
	yDec = Utilities::grayToDec(grayRow);

	if ((yDec > proj_height || xDec > proj_width))
	{
		error = true;
	}

	p_out.x = xDec;
	p_out.y = yDec;

	return error;
}

void decodePaterns(cv::Mat& shadowMask, vector<cv::Mat>& captured_pattern, vector<cv::Point> *camPixels)
{
	std::cout << "Decoding paterns...";

	int w = cam_width;
	int h = cam_height;

	cv::Point projPixel;

	for (int i = 0; i<w; i++)
	{
		for (int j = 0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if (shadowMask.at<uchar>(j, i))
			{

				//get the projector pixel for camera (i,j) pixel
				bool error = getProjPixel(i, j, projPixel, captured_pattern);

				if (error)
				{
					shadowMask.at<uchar>(j, i) = 0;
					continue;
				}
				camPixels[Utilities::ac(projPixel.x, projPixel.y)].push_back(cv::Point(i, j));

			}
		}
	}
	std::cout << "done!\n";
}

void saveFeaturePoints(vector<cv::Point> *camPixels, int num) {
	FeatureData::LocationData* ld = new FeatureData::LocationData;
	FeatureData::DescriptorData* dd = new FeatureData::DescriptorData;
	vector<float> ldData;
	ldData.resize(0);
	int sz = proj_width * proj_height;
	int countOfPoint = 0;
	for (int i = 0; i < sz; i++) {
		vector<cv::Point> points = camPixels[i];
		int psz = points.size();
		if (psz > 0) {
			float avgX = 0;
			float avgY = 0;
			for (int j = 0; j < psz; j++) {
				cv::Point point = points[j];
				avgX += point.x;
				avgY += point.y;
			}
			avgX /= psz;
			avgY /= psz;
			ldData.push_back(avgX);
			ldData.push_back(avgY);
			ldData.push_back(0);
			ldData.push_back(0);
			ldData.push_back(0);
			countOfPoint++;
		}
	}
	unsigned char* ddData = new unsigned char[128 * countOfPoint];
	memset(ddData, 0, 128 * countOfPoint);
	ld->setx(5, countOfPoint, ldData.data());
	dd->setx(128, countOfPoint, ddData);
	FeatureData fd(ld, dd);
	ostringstream numStr;
	numStr << num;
	fd.saveSIFTB2((root_dir + sfm_dir + numStr.str() + ".SIFT").c_str());
}

void saveMatch(vector<cv::Point> ***camsPixels, int numOfProjectorGroup) {
	ofstream ouF;
	ouF.open((root_dir + sfm_dir + "match.txt").c_str());
	int sz = proj_width * proj_height;
	int count = 0;
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	int lastPoistionFeatureNum = 0;
	for (int p = 0; p < numOfProjectorGroup; p++) {
		sprintf(projectorGroupDirTemp, projector_group_dir, p);
		cv::String imagesDir1 = root_dir + expr_dir + cv::String(projectorGroupDirTemp);
		int numOfImageGroup;
		Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfImageGroup);
		for (int i = 0; i < numOfImageGroup - 1; i++) {
			for (int j = i + 1; j < numOfImageGroup; j++) {
				vector<int> matches[2];
				matches[0].resize(0);
				matches[1].resize(0);
				int count1 = 0;
				int count2 = 0;
				vector<cv::Point> *camPixels1 = camsPixels[p][i];
				vector<cv::Point> *camPixels2 = camsPixels[p][j];
				for (int k = 0; k < sz; k++) {
					vector<cv::Point> points1 = camPixels1[k];
					vector<cv::Point> points2 = camPixels2[k];
					int psz1 = points1.size();
					int psz2 = points2.size();
					if (psz1 > 0 && psz2 > 0) {
						if (count1 % 10 == 0) {
							int count1Fin = (p == 0 || i != 0) ? count1 : (count1 + lastPoistionFeatureNum);
							matches[0].push_back(count1Fin);
							matches[1].push_back(count2);
						}
						count1++;
						count2++;
					}
					else if (psz1 > 0) {
						count1++;
					}
					else if (psz2 > 0) {
						count2++;
					}
				}
				int matchSz = matches[0].size();
				ouF << count << imgType << " " << count + j - i << imgType << " " << matchSz << "\n";
				for (int k = 0; k < 2; k++) {
					for (int l = 0; l < matchSz; l++) {
						ouF << matches[k][l] << " ";
					}
					ouF << "\n";
				}
				if (j == numOfImageGroup-1) {
					lastPoistionFeatureNum = count2;
				}
			}
			if (p == 0 || i != 0) {
				count++;
			}
		}
	}
	
	ouF.close();
}

void getProjectorGroupNum(int& num) {

}

void matchFeaturePoints(int const numOfProjectorGroup) {
	vector<cv::Point>*** camsPixels = new vector<cv::Point>**[numOfProjectorGroup];
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	for (int i = 0; i < numOfProjectorGroup; i++) {
		sprintf(projectorGroupDirTemp, projector_group_dir, i);
		cv::String imagesDir1 = root_dir + expr_dir + cv::String(projectorGroupDirTemp);
		int numOfImageGroup;
		Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfImageGroup);
		camsPixels[i] = new vector<cv::Point>*[numOfImageGroup];
		for (int j = 0; j < numOfImageGroup; j++)
		{
			camsPixels[i][j] = new vector<cv::Point>[proj_width * proj_height];
			//camPixels = camsPixels[i];
			vector<string> camFolder;
			camFolder.resize(0);
			char* imagesGroupDirTemp = new char[images_group_dir_length];
			sprintf(imagesGroupDirTemp, images_group_dir, i);
			cv::String filename = imagesDir1 + cv::String(imagesGroupDirTemp) + imagesName_file;
			Tools::readStringList(filename, camFolder);
			vector<cv::Mat> captured_pattern;
			Utilities::loadCamImgs(camFolder, captured_pattern);
			cv::Mat shadowMask;
			int numOfIimg = captured_pattern.size();
			computeShadows(shadowMask, captured_pattern[numOfIimg - 2], captured_pattern[numOfIimg - 1]);

			decodePaterns(shadowMask, captured_pattern, camsPixels[i][j]);
		}
	}
	// save feature points
	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		sprintf(projectorGroupDirTemp, projector_group_dir, i);
		cv::String imagesDir1 = root_dir + expr_dir + cv::String(projectorGroupDirTemp);
		int numOfImageGroup;
		Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfImageGroup);
		for (int j = 0; j < numOfImageGroup; j++) {
			if (i=0 || j!=0) {
				if (j == numOfImageGroup-1 && i < numOfProjectorGroup-1) {
					int sz = sizeof(camsPixels[i][j]);
					vector<cv::Point>* temp = (vector<cv::Point>*)malloc(sz * 2);
					memcpy(temp, camsPixels[i][j], sz);
					memcpy(temp + sz, camsPixels[i+1][0], sz);
					saveFeaturePoints(temp, count);
				}
				else {
					saveFeaturePoints(camsPixels[i][j], count);
				}
				count++;
			}
		}
	}
	// save match

	//saveMatch(camsPixels, numOfGroup);
}

void Sfm::executeSfm() {
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	makeSfmDir(numOfProjectorGroup);
	//matchFeaturePoints(numOfProjectorGroup);
}