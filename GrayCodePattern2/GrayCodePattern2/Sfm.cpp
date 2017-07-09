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

float distanceOfTwoPoints_2(cv::Point2f point1, cv::Point2f point2) {
	float xx = point1.x - point2.x;
	float yy = point1.y - point2.y;
	return xx * xx + yy * yy;
}

void saveFeaturePoints_2(vector<Tools::PointWithCode>& camPixels, int num) {
	FeatureData::LocationData* ld = new FeatureData::LocationData;
	FeatureData::DescriptorData* dd = new FeatureData::DescriptorData;
	vector<float> ldData;
	ldData.resize(0);
	int sz = camPixels.size();
	int count = 0;
	for (int i = 0; i < sz; i++) {
		Tools::PointWithCode pwc = camPixels[i];
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

  

/* update at 20170410 for shortening mapping time*/
void simplifyPointWithCodeVector_2(vector<Tools::PointWithCode>& input, vector<Tools::PointWithCode>& output, cv::Mat shadowMask) {
	int sz = input.size();
	for (int i = 0; i < sz; i++) {
		Tools::PointWithCode pwc = input[i];
		if (pwc.indexInSiftFile != -1 && Utilities::matGet2D(shadowMask, pwc.point.x, pwc.point.y) == 1) {
			output.push_back(pwc);
		}
	}
}

void simplifyPointWithCodeVectorTo2d_2(vector<Tools::PointWithCode>& input, vector<Tools::PointWithCode>** output, cv::Mat shadowMask) {
	int sz = input.size();
	for (int i = 0; i < sz; i++) {
		Tools::PointWithCode pwc = input[i];
		if (pwc.indexInSiftFile != -1 && Utilities::matGet2D(shadowMask, pwc.point.x, pwc.point.y) == 1) {
			int xInt = pwc.point.x;
			int yInt = pwc.point.y;
			output[xInt][yInt].resize(0);
			output[xInt][yInt].push_back(input[i]);
		}
	}
}

void calcCodeMapOfTwoProjecterPosition_2(vector<Tools::PointWithCode>& camsPixels1, vector<Tools::PointWithCode>& camsPixels2, cv::Mat shadowMask1, cv::Mat shadowMask2, map<int, int>& codeMap) {
	vector<Tools::PointWithCode> camsPixels1_;
	vector<Tools::PointWithCode>** camsPixels2_2d = new vector<Tools::PointWithCode>*[cam_width];
	for (int i = 0; i < cam_width; i++)
		camsPixels2_2d[i] = new vector<Tools::PointWithCode>[cam_height];

	int length = ceil(mapping_thresh);
	int length_ = length * 2 + 1;
	float nearestDistSquare = mapping_thresh * mapping_thresh;
	simplifyPointWithCodeVector_2(camsPixels1, camsPixels1_, shadowMask2);
	simplifyPointWithCodeVectorTo2d_2(camsPixels2, camsPixels2_2d, shadowMask1);
	int sz1 = camsPixels1_.size();
	//int sz2 = camsPixels2_.size();
	for (int i = 0; i < sz1; i++) {
		Tools::PointWithCode pwc1 = camsPixels1_[i];
		float nearestDist = nearestDistSquare;
		for (int j = 0; j < length_; j++) {
			for (int k = 0; k < length_; k++) {
				int xInt = pwc1.point.x - length + j;
				int yInt = pwc1.point.y - length + k;
				if (xInt >= 0 && xInt < cam_width && yInt >= 0 && yInt < cam_height && camsPixels2_2d[xInt][yInt].size() > 0) {
					vector<Tools::PointWithCode> pwcVec = camsPixels2_2d[xInt][yInt];
					for (Tools::PointWithCode pwc2 : pwcVec) {
						float dist = distanceOfTwoPoints_2(pwc1.point, pwc2.point);
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

void saveMatch(vector<Tools::PointWithCode> **camsPixels, int numOfProjectorGroup) {
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
		calcCodeMapOfTwoProjecterPosition_2(camsPixels[p][numOfImageGroup1-1], camsPixels[p+1][0], shadowMask1, shadowMask2, codeMap);
		cout << "End" << endl;
		int sizeOfLastOfProjectorGroup1 = camsPixels[p][numOfImageGroup1 - 1].size();
		int iMax = (p == numOfProjectorGroup - 2) ? numOfImageGroup1 + numOfImageGroup2 - 2 : numOfImageGroup1 - 1;
		for (int i = 0; i < iMax; i++) {
			int numI = i > numOfImageGroup1 - 2 ? 1 : 0;
			int ii = i - numI*(numOfImageGroup1 - 1);
			vector<Tools::PointWithCode> camPixels1 = camsPixels[p + numI][ii];
			for (int j = i + 1; j < numOfImageGroup1 + numOfImageGroup2 -1; j++) {
				int numJ = j > numOfImageGroup1 - 1 ? 1 : 0;
				int jj = j - numJ*(numOfImageGroup1 - 1);
				vector<int> matches[2];
				matches[0].resize(0);
				matches[1].resize(0);
				vector<Tools::PointWithCode> camPixels2 = camsPixels[p + numJ][jj];
				int sz = camPixels1.size();
				for (int k = 0; k < sz; k++) {
					Tools::PointWithCode pwc1 = camPixels1[k];
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
	}
	
	ouF.close();
	std::cout << "Save Match end" << endl;
}

void Sfm::executeMatching() {
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	// save feature points
	vector<Tools::PointWithCode>** avgCamsPixels = new vector<Tools::PointWithCode>*[numOfProjectorGroup];
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	int idx = 0;
	int numOfImageGroup;
	cv::String imagesDir1;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup, imagesDir1);
		avgCamsPixels[i] = new vector<Tools::PointWithCode>[numOfImageGroup];
		for (int j = 0; j < numOfImageGroup; j++) {
			idx = j==0? idx : 0;
			avgCamsPixels[i][j].resize(0);
			ostringstream numStr;
			numStr << j;
			Tools::loadCamsPixelsForReconstuction(avgCamsPixels[i][j], imagesDir1 + numStr.str() + decodefileType, idx);
		}
	}
	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup);
		for (int j = 0; j < numOfImageGroup; j++) {
			if (i == 0 || j != 0) {
				std::cout << "Saving Feature Points... (count " << count << ")" << endl;
				if (j == numOfImageGroup - 1 && i < numOfProjectorGroup - 1) {
					vector<Tools::PointWithCode> temp;
					temp.resize(0);
					temp.insert(temp.end(), avgCamsPixels[i][j].begin(), avgCamsPixels[i][j].end());
					temp.insert(temp.end(), avgCamsPixels[i + 1][0].begin(), avgCamsPixels[i + 1][0].end());
					saveFeaturePoints_2(temp, count);
				}
				else {
					saveFeaturePoints_2(avgCamsPixels[i][j], count);
				}
				count++;
			}
		}
	}

	// save match
	saveMatch(avgCamsPixels, numOfProjectorGroup);
}

void Sfm::simplifyMatchFile() {
	ifstream inF;
	inF.open((root_dir + sfm_dir + "match.txt").c_str());
	ofstream ouF;
	ouF.open((root_dir + sfm_dir + "match2.txt").c_str());
	while (!inF.eof()) {
		string fileA, fileB;
		int count, temp;
		inF >> fileA >> fileB >> count;
		int newMulti = count < matchFileSimplifyThresh ? count : matchFileSimplifyMultiple;
		int newCount = count * (newMulti - 1) / newMulti;
		ouF << fileA << " " << fileB << " " << newCount << "\n";
		for (int i = 0; i < count; i++) {
			inF >> temp;
			if (i % newMulti != 0) {
				ouF << temp << " ";
			}
		}
		ouF << "\n";
		for (int i = 0; i < count; i++) {
			inF >> temp;
			if (i % newMulti != 0) {
				ouF << temp << " ";
			}
		}
		ouF << "\n";
	}
	ouF.close();
	inF.close();
}