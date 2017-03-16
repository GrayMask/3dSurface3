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
#include <map>

//using namespace std;


struct PointWithCode {
	cv::Point point;
	int code;
};

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
		Tools::readGroupNumFile(imagesDir1 + imageGroupNum_file, numOfImageGroup);
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

void saveFeaturePoints(vector<PointWithCode>& camPixels, int num) {
	FeatureData::LocationData* ld = new FeatureData::LocationData;
	FeatureData::DescriptorData* dd = new FeatureData::DescriptorData;
	vector<float> ldData;
	ldData.resize(0);
	int sz = camPixels.size();
	for (int i = 0; i < sz; i++) {
		PointWithCode pwc = camPixels[i];
		ldData.push_back(pwc.point.x);
		ldData.push_back(pwc.point.y);
		ldData.push_back(0);
		ldData.push_back(0);
		ldData.push_back(0);
	}
	unsigned char* ddData = new unsigned char[128 * sz];
	memset(ddData, 0, 128 * sz);
	ld->setx(5, sz, ldData.data());
	dd->setx(128, sz, ddData);
	FeatureData fd(ld, dd);
	ostringstream numStr;
	numStr << num;
	fd.saveSIFTB2((root_dir + sfm_dir + numStr.str() + ".SIFT").c_str());
}

float distanceOfTwoPoints(cv::Point point1, cv::Point point2) {
	int xx = point1.x - point2.x;
	int yy = point1.y - point2.y;
	return sqrt(xx * xx + yy * yy);
}

void calcCodeMapOfTwoProjecterPosition(vector<PointWithCode>& camsPixels1, vector<PointWithCode>& camsPixels2, cv::Mat shadowMask2, map<int, int>& codeMap) {
	int sz1 = camsPixels1.size();
	int sz2 = camsPixels2.size();
	for (int i = 0; i < sz1; i++) {
		PointWithCode pwc1 = camsPixels1[i];
		shadowMask2.convertTo(shadowMask2, CV_32S);
		if (Utilities::matGet2D(shadowMask2 ,pwc1.point.x, pwc1.point.y) == 1) {
			float nearestDist = mapping_thresh;
			for (int j = 0; j < sz2; j++) {
				PointWithCode pwc2 = camsPixels2[j];
				float dist = distanceOfTwoPoints(pwc1.point, pwc2.point);
				if (dist < nearestDist) {
					codeMap[pwc1.code] = pwc2.code;
					nearestDist = dist;
				}
			}
		}
	}
}

int findIndexOfSameCodeInList(vector<PointWithCode>& camPixels, int code) {
	int sz = camPixels.size();
	for (int i = 0; i < sz; i++) {
		if (camPixels[i].code == code) {
			return i;
		}
	}
	return -1;
}

void saveMatch(vector<PointWithCode> **camsPixels, int numOfProjectorGroup) {
	std::cout << "Saving Match..." << endl;
	ofstream ouF;
	ouF.open((root_dir + sfm_dir + "match.txt").c_str());
	int count = 0;
	for (int p = 0; p < numOfProjectorGroup - 1; p++) {
		int numOfImageGroup1, numOfImageGroup2;
		Utilities::readNumOfImageGroup(p, numOfImageGroup1);
		Utilities::readNumOfImageGroup(p + 1, numOfImageGroup2);
		map<int, int> codeMap;
		cv::Mat shadowMask;
		Utilities::readShadowMask(shadowMask, p+1, 0);
		calcCodeMapOfTwoProjecterPosition(camsPixels[p][numOfImageGroup1-1], camsPixels[p+1][0], shadowMask, codeMap);
		int sizeOfLastOfProjectorGroup1 = camsPixels[p][numOfImageGroup1 - 1].size();
		for (int i = 0; i < numOfImageGroup1 + numOfImageGroup2 - 2; i++) {
			for (int j = i + 1; j < numOfImageGroup1 + numOfImageGroup2 -1; j++) {
				int numI = i > numOfImageGroup1 - 2 ? 1 : 0;
				int numJ = j > numOfImageGroup1 - 1 ? 1 : 0;
				int ii = i - numI*(numOfImageGroup1 - 1);
				int jj = j - numJ*(numOfImageGroup1 - 1);
				vector<int> matches[2];
				matches[0].resize(0);
				matches[1].resize(0);
				vector<PointWithCode> camPixels1 = camsPixels[p + numI][ii];
				vector<PointWithCode> camPixels2 = camsPixels[p + numJ][jj];
				int sz = camPixels1.size();
				for (int k = 0; k < sz; k++) {
					PointWithCode pwc1 = camPixels1[k];
					int codeFin = pwc1.code;
					if (numI != numJ) {
						map<int, int>::iterator iter;
						iter = codeMap.find(pwc1.code);
						if (iter != codeMap.end()) {
							codeFin = iter->second;
						}
						else {
							continue;
						}
					}
					int index = findIndexOfSameCodeInList(camPixels2, codeFin);
					if (index > 0) {
						if (i == numOfImageGroup1 - 1) {
							matches[0].push_back(k + sizeOfLastOfProjectorGroup1);
						} else {
							matches[0].push_back(k);
						}
						matches[1].push_back(index);
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
			}
			count++;
		}
	}
	
	ouF.close();
	std::cout << "Save Match end" << endl;
}

void matchFeaturePoints(int const numOfProjectorGroup) {
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	for (int i = 0; i < numOfProjectorGroup; i++) {
		sprintf(projectorGroupDirTemp, projector_group_dir, i);
		cv::String imagesDir1 = root_dir + expr_dir + cv::String(projectorGroupDirTemp);
		int numOfImageGroup;
		Tools::readGroupNumFile(imagesDir1 + imageGroupNum_file, numOfImageGroup);
		//camsPixels[i] = new vector<cv::Point>*[numOfImageGroup];
		for (int j = 0; j < numOfImageGroup; j++)
		{
			ostringstream numStr;
			numStr << j;
			vector<cv::Point>* camsPixels = new vector<cv::Point>[proj_width * proj_height];
			vector<string> camFolder;
			camFolder.resize(0);
			char* imagesGroupDirTemp = new char[images_group_dir_length];
			sprintf(imagesGroupDirTemp, images_group_dir,j);
			cv::String filename = imagesDir1 + cv::String(imagesGroupDirTemp) + imagesName_file;
			Tools::readStringList(filename, camFolder);
			vector<cv::Mat> captured_pattern;
			Utilities::loadCamImgs(imagesDir1 + cv::String(imagesGroupDirTemp), camFolder, captured_pattern);
			cv::Mat shadowMask;
			int numOfIimg = captured_pattern.size();
			computeShadows(shadowMask, captured_pattern[numOfIimg - 2], captured_pattern[numOfIimg - 1]);
			Utilities::writeShadowMask(shadowMask, imagesDir1 + shadowMask_file + numStr.str() + imgType);

			decodePaterns(shadowMask, captured_pattern, camsPixels);
			Tools::saveCamsPixelsForReconstuction(camsPixels, imagesDir1 + numStr.str() + decodefileType);
			delete[] camsPixels;
			delete[] imagesGroupDirTemp;
		}
	}

}
void Sfm::executeDecoding() {
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	makeSfmDir(numOfProjectorGroup);
	matchFeaturePoints(numOfProjectorGroup);
}

void Sfm::executeMatching() {
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	// save feature points
	vector<PointWithCode>** avgCamsPixels = new vector<PointWithCode>*[numOfProjectorGroup];
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	for (int i = 0; i < numOfProjectorGroup; i++) {
		sprintf(projectorGroupDirTemp, projector_group_dir, i);
		cv::String imagesDir1 = root_dir + expr_dir + cv::String(projectorGroupDirTemp);
		int numOfImageGroup;
		Tools::readGroupNumFile(imagesDir1 + imageGroupNum_file, numOfImageGroup);
		avgCamsPixels[i] = new vector<PointWithCode>[numOfImageGroup];
		for (int j = 0; j < numOfImageGroup; j++) {
			int count = 0;
			avgCamsPixels[i][j].resize(0);
			vector<vector<cv::Point>> camsPixels;
			ostringstream numStr;
			numStr << j;
			Tools::loadCamsPixelsForReconstuction(camsPixels, imagesDir1 + numStr.str() + decodefileType);
			int sz = camsPixels.size();
			for (int n = 0; n < sz; n++) {
				cv::Point avg = 0;
				int numOFPoint = camsPixels[n].size();
				if (numOFPoint > 0) {
					for (int m = 0; m < numOFPoint; m++) {
						avg += camsPixels[n][m];
					}
					avg /= numOFPoint;
					PointWithCode pwc;
					pwc.code = n;
					pwc.point = avg;
					avgCamsPixels[i][j].push_back(pwc);
				}
			}
		}
	}
	
	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		sprintf(projectorGroupDirTemp, projector_group_dir, i);
		cv::String imagesDir1 = root_dir + expr_dir + cv::String(projectorGroupDirTemp);
		int numOfImageGroup;
		Tools::readGroupNumFile(imagesDir1 + imageGroupNum_file, numOfImageGroup);
		for (int j = 0; j < numOfImageGroup; j++) {
			if (i == 0 || j != 0) {
				std::cout << "Saving Feature Points... (count " << count << ")" << endl;
				if (j == numOfImageGroup - 1 && i < numOfProjectorGroup - 1) {
					vector<PointWithCode> temp;
					temp.resize(0);
					temp.insert(temp.end(), avgCamsPixels[i][j].begin(), avgCamsPixels[i][j].end());
					temp.insert(temp.end(), avgCamsPixels[i + 1][0].begin(), avgCamsPixels[i + 1][0].end());
					saveFeaturePoints(temp, count);
				}
				else {
					saveFeaturePoints(avgCamsPixels[i][j], count);
				}
				count++;
			}
		}
	}

	// save match
	saveMatch(avgCamsPixels, numOfProjectorGroup);
}