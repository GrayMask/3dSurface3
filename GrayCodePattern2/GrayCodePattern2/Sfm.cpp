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
	cv::Point2f point;
	int indexInSiftFile;
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
	cv::String imagesDir1;
	int numOfImageGroup;
	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup, imagesDir1);
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
bool getProjPixel(int x, int y, cv::Point2f &p_out, vector<cv::Mat>& captured_pattern)
{
	vector<bool> grayCol;
	vector<bool> grayRow;
	int numOfColBits = log(proj_width) / log(2);
	int numOfRowBits = log(proj_height) / log(2);

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

	cv::Point2f projPixel;

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
	int count = 0;
	for (int i = 0; i < sz; i++) {
		PointWithCode pwc = camPixels[i];
		if (pwc.indexInSiftFile != -1) {
			ldData.push_back(pwc.point.x);
			ldData.push_back(pwc.point.y);
			ldData.push_back(0);
			ldData.push_back(0);
			ldData.push_back(0);
			count++;
		}
	}
	unsigned char* ddData = new unsigned char[128 * count];
	memset(ddData, 0, 128 * count);
	ld->setx(5, count, ldData.data());
	dd->setx(128, count, ddData);
	FeatureData fd(ld, dd);
	ostringstream numStr;
	numStr << num;
	fd.saveSIFTB2((root_dir + sfm_dir + numStr.str() + ".SIFT").c_str());
}

float distanceOfTwoPoints(cv::Point2f point1, cv::Point2f point2) {
	float xx = point1.x - point2.x;
	float yy = point1.y - point2.y;
	return xx * xx + yy * yy;
}

/* update at 20170410 for shortening mapping time*/
void simplifyPointWithCodeVector(vector<PointWithCode>& input, vector<PointWithCode>& output, cv::Mat shadowMask) {
	int sz = input.size();
	for (int i = 0; i < sz; i++) {
		PointWithCode pwc = input[i];
		if (pwc.indexInSiftFile != -1 && Utilities::matGet2D(shadowMask, pwc.point.x, pwc.point.y) == 1) {
			output.push_back(pwc);
		}
	}
}

void simplifyPointWithCodeVectorTo2d(vector<PointWithCode>& input, vector<PointWithCode>** output, cv::Mat shadowMask) {
	int sz = input.size();
	for (int i = 0; i < sz; i++) {
		PointWithCode pwc = input[i];
		if (pwc.indexInSiftFile != -1 && Utilities::matGet2D(shadowMask, pwc.point.x, pwc.point.y) == 1) {
			int xInt = pwc.point.x;
			int yInt = pwc.point.y;
			output[xInt][yInt].resize(0);
			output[xInt][yInt].push_back(input[i]);
		}
	}
}

void calcCodeMapOfTwoProjecterPosition(vector<PointWithCode>& camsPixels1, vector<PointWithCode>& camsPixels2, cv::Mat shadowMask1, cv::Mat shadowMask2, map<int, int>& codeMap) {
	vector<PointWithCode> camsPixels1_;
	vector<PointWithCode>** camsPixels2_2d = new vector<PointWithCode>*[cam_width];
	for (int i = 0; i < cam_width; i++)
		camsPixels2_2d[i] = new vector<PointWithCode>[cam_height];

	int length = ceil(mapping_thresh);
	int length_ = length * 2 + 1;
	float nearestDistSquare = mapping_thresh * mapping_thresh;
	simplifyPointWithCodeVector(camsPixels1, camsPixels1_, shadowMask2);
	simplifyPointWithCodeVectorTo2d(camsPixels2, camsPixels2_2d, shadowMask1);
	int sz1 = camsPixels1_.size();
	//int sz2 = camsPixels2_.size();
	for (int i = 0; i < sz1; i++) {
		PointWithCode pwc1 = camsPixels1_[i];
		float nearestDist = nearestDistSquare;
		for (int j = 0; j < length_; j++) {
			for (int k = 0; k < length_; k++) {
				int xInt = pwc1.point.x - length + j;
				int yInt = pwc1.point.y - length + k;
				if (xInt >= 0 && xInt < cam_width && yInt >= 0 && yInt < cam_height && camsPixels2_2d[xInt][yInt].size() > 0) {
					vector<PointWithCode> pwcVec = camsPixels2_2d[xInt][yInt];
					for (PointWithCode pwc2 : pwcVec) {
						float dist = distanceOfTwoPoints(pwc1.point, pwc2.point);
						if (dist < nearestDist) {
							codeMap[pwc1.code] = pwc2.code;
							nearestDist = dist;
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < cam_width; i++)
	{
		delete[] camsPixels2_2d[i];
	}
	delete camsPixels2_2d;
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
		cv::Mat shadowMask1;
		cv::Mat shadowMask2;
		Utilities::readShadowMask(shadowMask1, p, numOfImageGroup1 - 1);
		Utilities::readShadowMask(shadowMask2, p+1, 0);
		cout << "Calculating Code Map Of Two Projecter Position " << p << " and " << p+1 << " ..." << endl;
		calcCodeMapOfTwoProjecterPosition(camsPixels[p][numOfImageGroup1-1], camsPixels[p+1][0], shadowMask1, shadowMask2, codeMap);
		cout << "End" << endl;
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
					if (pwc1.indexInSiftFile != -1) {
						int codeFin = k;
						if (numI != numJ) {
							map<int, int>::iterator iter;
							iter = codeMap.find(k);
							if (iter != codeMap.end()) {
								codeFin = iter->second;
							}
							else {
								continue;
							}
						}
						int idx2 = camPixels2[codeFin].indexInSiftFile;
						if (idx2 != -1) {
							matches[0].push_back(pwc1.indexInSiftFile);
							matches[1].push_back(idx2);
						}
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
		count = count - numOfImageGroup2 + 1;
	}
	
	ouF.close();
	std::cout << "Save Match end" << endl;
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

void matchFeaturePoints(int const numOfProjectorGroup) {
	cv::String imagesDir1;
	int numOfImageGroup;
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup, imagesDir1);
		for (int j = 0; j < numOfImageGroup; j++)
		{
			ostringstream numStr;
			numStr << j;
			vector<vector<cv::Point>> camsPixels;
			vector<cv::String> camFolder;
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

			//decodePaterns(shadowMask, captured_pattern, camsPixels);
			decode(shadowMask, captured_pattern, camsPixels);
			Tools::saveCamsPixelsForReconstuction(camsPixels, imagesDir1 + numStr.str() + decodefileType);
			//delete[] camsPixels;
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
	int idx = 0;
	int numOfImageGroup;
	cv::String imagesDir1;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup, imagesDir1);
		avgCamsPixels[i] = new vector<PointWithCode>[numOfImageGroup];
		for (int j = 0; j < numOfImageGroup; j++) {
			idx = j==0? idx : 0;
			avgCamsPixels[i][j].resize(0);
			vector<vector<cv::Point2f>> camsPixels;
			ostringstream numStr;
			numStr << j;
			Tools::loadCamsPixelsForReconstuction(camsPixels, imagesDir1 + numStr.str() + decodefileType);
			int sz = camsPixels.size();
			for (int n = 0; n < sz; n++) {
				cv::Point2f avg = 0;
				int numOFPoint = camsPixels[n].size();
				PointWithCode pwc;
				if (numOFPoint > 0) {
					for (int m = 0; m < numOFPoint; m++) {
						avg += camsPixels[n][m];
					}
					avg /= numOFPoint;
					pwc.indexInSiftFile = idx;
					pwc.point = avg;
					pwc.code = n;
					idx++;
				}
				else {
					pwc.indexInSiftFile = -1;
				}
				avgCamsPixels[i][j].push_back(pwc);
			}
		}
	}

	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup);
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

void Sfm::simplifyMatchFile() {
	int multiple = 3;
	ifstream inF;
	inF.open((root_dir + sfm_dir + "match.txt").c_str());
	ofstream ouF;
	ouF.open((root_dir + sfm_dir + "match2.txt").c_str());
	while (!inF.eof()) {
		string fileA, fileB;
		int count, temp;
		inF >> fileA >> fileB >> count;
		int newMulti = count < 20000 ? 1 : multiple;
		int newCount = (count + newMulti -1) / newMulti;
		ouF << fileA << " " << fileB << " " << newCount << "\n";
		for (int i = 0; i < count; i++) {
			inF >> temp;
			if (i % newMulti == 0) {
				ouF << temp << " ";
			}
		}
		ouF << "\n";
		for (int i = 0; i < count; i++) {
			inF >> temp;
			if (i % newMulti == 0) {
				ouF << temp << " ";
			}
		}
		ouF << "\n";
	}
	ouF.close();
	inF.close();
}