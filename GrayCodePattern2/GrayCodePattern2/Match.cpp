#include <io.h>
#include <direct.h>
#include <stdio.h>
#include <cmath>
#include "Match.h"
#include "Tools.h"
#include "Path.h"
#include "Const.h"
#include "FeaturePoints.h"
#include "Utilities.h"
#include <iostream>
#include <map>

void saveFeaturePoints(vector<Tools::PointWithCode>& camPixels, int num) {
	FeatureData::LocationData* ld = new FeatureData::LocationData;
	FeatureData::DescriptorData* dd = new FeatureData::DescriptorData;
	vector<float> ldData;
	ldData.resize(0);
	int sz = camPixels.size();
	int count = 0;
	for (int i = 0; i < sz; i++) {
		Tools::PointWithCode pwc = camPixels[i];
		if (pwc.indexInSiftFile != -1 && !pwc.isMerged) {
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

void getCamsPixels(vector<Tools::PointWithCode>** avgCamsPixels, int numOfProjectorGroup) {
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	int idx = 0;
	int numOfImageGroup;
	cv::String imagesDir1;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup, imagesDir1);
		avgCamsPixels[i] = new vector<Tools::PointWithCode>[numOfImageGroup];
		for (int j = 0; j < numOfImageGroup; j++) {
			idx = j == 0 ? idx : 0;
			avgCamsPixels[i][j].resize(0);
			ostringstream numStr;
			numStr << j;
			Tools::loadCamsPixelsForReconstuction(avgCamsPixels[i][j], imagesDir1 + numStr.str() + decodefileType, idx);
		}
	}
}

void simplifyPointWithCodeVector(vector<Tools::PointWithCode>& input, vector<Tools::PointWithCode>& output, cv::Mat shadowMask) {
	int sz = input.size();
	for (int i = 0; i < sz; i++) {
		Tools::PointWithCode pwc = input[i];
		if (pwc.indexInSiftFile != -1 && Utilities::matGet2D(shadowMask, pwc.point.x, pwc.point.y) == 1) {
			output.push_back(pwc);
		}
	}
}

void simplifyPointWithCodeVectorTo2d(vector<Tools::PointWithCode>& input, vector<Tools::PointWithCode*>** output, cv::Mat shadowMask) {
	int sz = input.size();
	for (int i = 0; i < sz; i++) {
		Tools::PointWithCode pwc = input[i];
		if (pwc.indexInSiftFile != -1 && Utilities::matGet2D(shadowMask, pwc.point.x, pwc.point.y) == 1) {
			int xInt = pwc.point.x;
			int yInt = pwc.point.y;
			output[xInt][yInt].resize(0);
			output[xInt][yInt].push_back(&input[i]);
		}
	}
}

float distanceOfTwoPoints(cv::Point2f point1, cv::Point2f point2) {
	float xx = point1.x - point2.x;
	float yy = point1.y - point2.y;
	return xx * xx + yy * yy;
}

Tools::PointWithCode* findNearestCode(Tools::PointWithCode& pwc1, vector<Tools::PointWithCode*>** camsPixels2_2d) {
	Tools::PointWithCode* nearestCode = NULL;
	float nearestDist = nearestDistSquare;
	int nearestXInt, nearestYInt;
	for (int j = 0; j < length_; j++) {
		for (int k = 0; k < length_; k++) {
			int xInt = pwc1.point.x - length + j;
			int yInt = pwc1.point.y - length + k;
			if (xInt >= 0 && xInt < cam_width && yInt >= 0 && yInt < cam_height && camsPixels2_2d[xInt][yInt].size() > 0) {
				vector<Tools::PointWithCode*> pwcVec = camsPixels2_2d[xInt][yInt];
				vector<Tools::PointWithCode*>::iterator itr = pwcVec.begin();
				while (itr != pwcVec.end()) {
					Tools::PointWithCode* pwc2 = *itr;
					float dist = distanceOfTwoPoints(pwc1.point, pwc2->point);
					if (dist < nearestDist) {
						nearestCode = pwc2;
						nearestDist = dist;
					}
					nearestXInt = xInt;
					nearestYInt = yInt;
					itr++;
				}
			}
		}
	}
	if (nearestCode != NULL) {
		vector<Tools::PointWithCode*>* nearestVals = &camsPixels2_2d[nearestXInt][nearestYInt];
		for (vector<Tools::PointWithCode*>::iterator nearestItr = nearestVals->begin(); nearestItr != nearestVals->end(); )
		{
			Tools::PointWithCode* pwc = *nearestItr;
			if (pwc->code == nearestCode->code)
			{
				nearestItr = nearestVals->erase(nearestItr);
				break;
			}
			else
			{
				++nearestItr;
			}
		}
	}
	return nearestCode;
}


void calcCodeMapOfTwoProjecterPosition(vector<Tools::PointWithCode>& camsPixels1, vector<Tools::PointWithCode>& camsPixels2, vector<Tools::PointWithCode>& mergingCamsPixels, cv::Mat shadowMask1, cv::Mat shadowMask2, map<int, int>& codeMap) {
	//vector<Tools::PointWithCode> camsPixels1_;
	vector<Tools::PointWithCode*>** camsPixels2_2d = new vector<Tools::PointWithCode*>*[cam_width];
	for (int i = 0; i < cam_width; i++)
		camsPixels2_2d[i] = new vector<Tools::PointWithCode*>[cam_height];

	mergingCamsPixels.resize(0);
	//simplifyPointWithCodeVector(camsPixels1, camsPixels1_, shadowMask2);
	simplifyPointWithCodeVectorTo2d(camsPixels2, camsPixels2_2d, shadowMask1);
	int sz1 = camsPixels1.size();
	//int sz2 = camsPixels2_.size();
	for (int i = 0; i < sz1; i++) {
		Tools::PointWithCode pwc1 = camsPixels1[i];
		if (pwc1.indexInSiftFile != -1 && Utilities::matGet2D(shadowMask2, pwc1.point.x, pwc1.point.y) == 1) {
			Tools::PointWithCode* pwc2 = findNearestCode(pwc1, camsPixels2_2d);
			if (pwc2 != NULL) {
				codeMap[pwc1.code] = pwc2->code;
				pwc1.point.x = pwc2->point.x = (pwc1.point.x + pwc2->point.x) / 2;
				pwc1.point.y = pwc2->point.y = (pwc1.point.y + pwc2->point.y) / 2;
				pwc2->indexInSiftFile = pwc1.indexInSiftFile;
				pwc2->isMerged = true;
				// delete the found point in the second vector
				//mergingCamsPixels[pwc2->code].indexInSiftFile = -1;
			}
		}
	}
	for (int i = 0; i < cam_width; i++)
	{
		delete[] camsPixels2_2d[i];
	}
	delete camsPixels2_2d;
	// Rewrite the index of the second camsPixels after merging.
	int idx, j;
	for (j = 0; j < sz1; j++) {
		Tools::PointWithCode pwc2 = camsPixels2[j];
		if (pwc2.indexInSiftFile != -1 && !pwc2.isMerged) {
			idx = pwc2.indexInSiftFile;
			j++;
			break;
		}
	}
	Tools::PointWithCode* pwc2;
	for (j; j < sz1; j++) {
		pwc2 = &camsPixels2[j];
		if (camsPixels2[j].indexInSiftFile != -1 && !camsPixels2[j].isMerged) {
			pwc2->indexInSiftFile = ++idx;
		}
	}
	mergingCamsPixels.insert(mergingCamsPixels.end(), camsPixels1.begin(), camsPixels1.end());
	mergingCamsPixels.insert(mergingCamsPixels.end(), camsPixels2.begin(), camsPixels2.end());
}

void savePoints(vector<Tools::PointWithCode>** avgCamsPixels, int numOfProjectorGroup, vector<map<int, int>>& codeMaps) {
	codeMaps.resize(0);
	int numOfImageGroup;
	int count = 0;
	for (int i = 0; i < numOfProjectorGroup; i++) {
		Utilities::readNumOfImageGroup(i, numOfImageGroup);
		for (int j = 0; j < numOfImageGroup; j++) {
			if (i == 0 || j != 0) {
				std::cout << "Saving Feature Points... (count " << count << ")" << endl;
				if (j == numOfImageGroup - 1 && i < numOfProjectorGroup - 1) {
					map<int, int> codeMap;
					vector<Tools::PointWithCode> temp;
					cv::Mat shadowMask1;
					cv::Mat shadowMask2;
					Utilities::readShadowMask(shadowMask1, i, j);
					Utilities::readShadowMask(shadowMask2, i + 1, 0);
					cout << "Calculating Code Map Of Two Projecter Position " << i << " and " << i + 1 << " ..." << endl;
					calcCodeMapOfTwoProjecterPosition(avgCamsPixels[i][j], avgCamsPixels[i + 1][0], temp, shadowMask1, shadowMask2, codeMap);
					saveFeaturePoints(temp, count);
					cout << "End" << endl;
					codeMaps.push_back(codeMap);
				}
				else {
					saveFeaturePoints(avgCamsPixels[i][j], count);
				}
				count++;
			}
		}
	}
}

void saveMatch(vector<Tools::PointWithCode> **camsPixels, int numOfProjectorGroup, vector<map<int, int>>& codeMaps) {
	std::cout << "Saving Match..." << endl;
	ofstream ouF;
	ouF.open((root_dir + sfm_dir + "match.txt").c_str());
	int count = 0;
	for (int p = 0; p < numOfProjectorGroup - 1; p++) {
		int numOfImageGroup1, numOfImageGroup2;
		Utilities::readNumOfImageGroup(p, numOfImageGroup1);
		Utilities::readNumOfImageGroup(p + 1, numOfImageGroup2);
	
		int sizeOfLastOfProjectorGroup1 = camsPixels[p][numOfImageGroup1 - 1].size();
		int iMax = (p == numOfProjectorGroup - 2) ? numOfImageGroup1 + numOfImageGroup2 - 2 : numOfImageGroup1 - 1;
		for (int i = 0; i < iMax; i++) {
			int numI = i > numOfImageGroup1 - 2 ? 1 : 0;
			int ii = i - numI*(numOfImageGroup1 - 1);
			vector<Tools::PointWithCode> camPixels1 = camsPixels[p + numI][ii];
			for (int j = i + 1; j < numOfImageGroup1 + numOfImageGroup2 - 1; j++) {
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
							iter = codeMaps[p].find(k);
							if (iter != codeMaps[p].end()) {
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

void Match::executeMatching() {
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	vector<Tools::PointWithCode>** avgCamsPixels = new vector<Tools::PointWithCode>*[numOfProjectorGroup];
	getCamsPixels(avgCamsPixels, numOfProjectorGroup);

	// save points and calculate code maps
	vector<map<int, int>> codeMaps;
	savePoints(avgCamsPixels, numOfProjectorGroup, codeMaps);
	// save match
	saveMatch(avgCamsPixels, numOfProjectorGroup, codeMaps);
}

void Match::simplifyMatchFile() {
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