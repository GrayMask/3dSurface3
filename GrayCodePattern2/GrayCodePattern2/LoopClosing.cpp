#include <ctime>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include "LoopClosing.h"
#include "Tools.h"
#include "Const.h"
#include "Path.h"
#include "Utilities.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

bool readNvmFile(pointcloud::Ptr cloud1, pointcloud::Ptr cloud2, vector<int>& featureIdxList1, vector<int>& featureIdxList2, int startImgIdx, int& endImgIdx) {
	ifstream in(root_dir + nvm_file);
	if (!in.is_open())
	{
		cout << "Error opening file";
		return false;
	}
	featureIdxList1.resize(0);
	featureIdxList2.resize(0);
	int imageMount, pointMount, featureMount, imageIdx, featureIdx;

	double x, y, z;
	if (Tools::goWithLine(in, 2)) {
		in >> imageMount;
		if (endImgIdx > imageMount -1 || endImgIdx < 0) {
			endImgIdx = imageMount - 1;
		}
		ostringstream startStr, endStr;
		startStr << startImgIdx;
		endStr << endImgIdx;
		cv::String startPathStr = startStr.str() + imgType;
		cv::String endPathStr = endStr.str() + imgType;
		string tempPathStr;
		int nvmStartImgIdx, nvmEndImgIdx;
		// get idx in nvm file
		for (int k = 0; k < imageMount; k++) {
			Tools::goWithLine(in, 1);
			in >> tempPathStr;
			if (startPathStr == tempPathStr) {
				nvmStartImgIdx = k;
			}
			else if (endPathStr == tempPathStr) {
				nvmEndImgIdx = k;
			}
		}
		if (Tools::goWithLine(in, 2)) {
			in >> pointMount;
			for (int i = 0; i < pointMount; i++) {
				in >> x >> y >> z;
				Tools::goWithStep(in, 3);
				in >> featureMount;
				for (int j = 0; j < featureMount; j++) {
					in >> imageIdx >> featureIdx;
					Tools::goWithStep(in, 2);
					if (imageIdx == nvmStartImgIdx) {
						cloud1->push_back(pcl::PointXYZ(x, y, z));
						featureIdxList1.push_back(featureIdx);
					}
					else if(imageIdx == nvmEndImgIdx) {
						cloud2->push_back(pcl::PointXYZ(x, y, z));
						featureIdxList2.push_back(featureIdx);
					}
				}
				Tools::goWithLine(in, 1);
			}
			return true;
		}
	}
	return false;
}

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	est_normal.compute(*point_normal);
	//fpfh 
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4); //4 core
									// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}

static void transformPointCloud(double t, pointcloud::Ptr input, pointcloud::Ptr output) {
	int sz = input->size();
	for (int i = 0; i < sz; i++) {
		pcl::PointXYZ point = input->at(i);
		point.z = point.z + t;
		output->push_back(point);
	}
}

void showCorrespondence(pointcloud::Ptr source, pointcloud::Ptr target, boost::shared_ptr<pcl::Correspondences> cru_correspondences) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
	int v1;

	view->createViewPort(0, 0.0, 1.0, 1.0, v1);
	view->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
	view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
	pointcloud::Ptr targetShow(new pointcloud);
	transformPointCloud(0.5, target, targetShow);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(targetShow, 0, 250, 0);
	view->addPointCloud(targetShow, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

	view->addCorrespondences<pcl::PointXYZ>(source, targetShow, *cru_correspondences, "correspondence", v1);
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void showCorrespondence(pointcloud::Ptr source, pointcloud::Ptr target) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
	int v1;

	view->createViewPort(0, 0.0, 1.0, 1.0, v1);
	view->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
	view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);
	view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

double getDistance(pcl::PointXYZ& pa, pcl::PointXYZ& pb) {
	double x_, y_, z_;
	x_ = pa.x - pb.x;
	y_ = pa.y - pb.y;
	z_ = pa.z - pb.z;

	return sqrt(x_*x_ + y_*y_ + z_*z_);
}

void rejectCorrespondence(pointcloud::Ptr source, pointcloud::Ptr target, boost::shared_ptr<pcl::Correspondences> cru_correspondences, boost::shared_ptr<pcl::Correspondences> output, double maxDistance) {
	int sz = cru_correspondences->size();
	for (int i = 0; i < sz; i++) {
		pcl::Correspondence temp = cru_correspondences->at(i);
		pcl::PointXYZ ps = source->at(temp.index_query);
		pcl::PointXYZ pt = target->at(temp.index_match);
		double d = getDistance(ps, pt);
		if (d < maxDistance) {
			output->push_back(temp);
		}
	}
}


int threeDpointMatching(pointcloud::Ptr source, pointcloud::Ptr target, boost::shared_ptr<pcl::Correspondences> cru_correspondences)
{
	clock_t start, end, time;
	start = clock();

	cout << source->width;
	cout << target->width;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;

	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	//  crude_cor_est.determineCorrespondences(cru_correspondences);
	boost::shared_ptr<pcl::Correspondences> initial_correspondences(new pcl::Correspondences);
	crude_cor_est.determineReciprocalCorrespondences(*initial_correspondences);
	cout << "crude size is:" << initial_correspondences->size() << endl;

	end = clock();
	cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

	double newThresh = initial_max_distance_thresh;
	int select;
	boost::shared_ptr<pcl::Correspondences> rejected_correspondences(new pcl::Correspondences);
	while (true) {
		rejectCorrespondence(source, target, initial_correspondences, rejected_correspondences, newThresh);
		cout << "selected correspondences size is:" << rejected_correspondences->size() << endl;
		cout << "The distance threshold is " << newThresh << endl;
		cout << "Would change the threshold? (input 1(yes) or 0(no))" << endl;
		showCorrespondence(source, target, rejected_correspondences);
		cin >> select;
		if (select == 1) {
			cout << "Input the new threshold: " << endl;
			cin >> newThresh;
		}
		else {
			break;
		}
	}
	//pcl::io::savePCDFile("crou_output.pcd", *align);
	//  pcl::io::savePCDFile ("final_align.pcd", *final);
	*cru_correspondences = *rejected_correspondences;
	return 0;
}

class MyICP : public pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>
{
public:
	pcl::CorrespondencesPtr getCorrespondences()
	{
		return correspondences_;
	}
};

int threeDpointMatchingICP(pointcloud::Ptr source, pointcloud::Ptr target, boost::shared_ptr<pcl::Correspondences> cru_correspondences)
{
	MyICP icp;
	pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
	// Set the input source and target
	icp.setInputSource(source);
	icp.setInputTarget(target);
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(0.005);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations(100);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon(1e-10);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon(1);
	// Perform the alignment
	icp.align(cloud_source_registered);
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	pcl::CorrespondencesPtr temp = icp.getCorrespondences();
	pointcloud endCloud;
	pcl::transformPointCloud(*source.get(), endCloud,transformation);
	int sz = temp->size();
	for (int i = 0; i < sz; i++) {
		cru_correspondences->push_back(temp->at(i));
	}
	pcl::PLYWriter writer;
	writer.write(root_dir + expr_dir + "source.ply", endCloud);
	writer.write(root_dir + expr_dir + "target.ply", *target);
	if (showICPResult) {
		pointcloud::Ptr endCloud_(&endCloud);
		showCorrespondence(endCloud_, target);
		showCorrespondence(source, target, cru_correspondences);
	}
	return 0;
}

void optimizeCodeList(vector<Tools::PointWithCode>& input, vector<Tools::PointWithCode>& output) {
	int sz = input.size();
	for (int i = 0; i < sz; i++) {
		Tools::PointWithCode temp = input[i];
		if (temp.indexInSiftFile != -1) {
			output.push_back(temp);
		}
	}
}

void outputFirstAndEndMatchesAndMakeCodeMap(boost::shared_ptr<pcl::Correspondences> cru_correspondences, 
		vector<int>& featureIdxList1, vector<int>& featureIdxList2, int startImgIdx, int endImgIdx, 
		map<int, int>& codeMap, vector<Tools::PointWithCode>** avgCamsPixels, int endGroupImageNum) {
	int sz = cru_correspondences->size();
	ofstream ouF(root_dir + sfm_dir + "match.txt", ios::app);
	ouF << startImgIdx << imgType << " " << endImgIdx << imgType << " " << sz << "\n";
	vector<int> matches[2];
	matches[0].resize(0);
	matches[1].resize(0);
	vector<Tools::PointWithCode> firstList;
	vector<Tools::PointWithCode> endList;
	optimizeCodeList(avgCamsPixels[0][0], firstList);
	optimizeCodeList(avgCamsPixels[1][endGroupImageNum - 1], endList);
	for (int i = 0; i < sz; i++) {
		int sourceI = cru_correspondences->at(i).index_query;
		int targetI = cru_correspondences->at(i).index_match;
		matches[0].push_back(featureIdxList1[sourceI]);
		matches[1].push_back(featureIdxList2[targetI]);
		int a = featureIdxList1[sourceI];
		int b = featureIdxList2[targetI];
		codeMap[firstList[a].code] = endList[b].code;
	}
	int matchSz = matches[0].size();
	for (int k = 0; k < 2; k++) {
		for (int l = 0; l < matchSz; l++) {
			ouF << matches[k][l] << " ";
		}
		ouF << "\n";
	}
}

void readTwoGroupCode(vector<Tools::PointWithCode>** avgCamsPixels, vector<int>& sz) {
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	// save feature points
	
	char* projectorGroupDirTemp = new char[projector_group_dir_length];
	int idx;
	int numOfImageGroup;
	cv::String imagesDir1;
	for (int i = 0; i < 2; i++) {
		Utilities::readNumOfImageGroup(i * (numOfProjectorGroup-1), numOfImageGroup, imagesDir1);
		avgCamsPixels[i] = new vector<Tools::PointWithCode>[numOfImageGroup];
		sz.push_back(numOfImageGroup);
		for (int j = 0; j < numOfImageGroup; j++) {
			idx = 0;
			if (j==0 && i ==1) {
				vector<Tools::PointWithCode> temp;
				cv::String imagesDirTemp;
				int numOfImageGroupTemp;
				Utilities::readNumOfImageGroup(i * numOfProjectorGroup - 2, numOfImageGroupTemp, imagesDirTemp);
				ostringstream numStrTemp;
				numStrTemp << numOfImageGroupTemp - 1;
				Tools::loadCamsPixelsForReconstuction(temp, imagesDirTemp + numStrTemp.str() + decodefileType, idx);
			}
			avgCamsPixels[i][j].resize(0);
			ostringstream numStr;
			numStr << j;
			Tools::loadCamsPixelsForReconstuction(avgCamsPixels[i][j], imagesDir1 + numStr.str() + decodefileType, idx);
		}
	}
}

void outputMatches(vector<Tools::PointWithCode>** avgCamsPixels, vector<int>& sz, map<int, int>& codeMap, int endImgIdx) {
	ofstream ouF;
	ouF.open((root_dir + sfm_dir + "match.txt").c_str(), ios::app);
	for (int i = 0; i < sz[0]; i++) {
		vector<Tools::PointWithCode> camPixels1 = avgCamsPixels[0][i];
		for (int j = 0; j < sz[1]; j++) {
			vector<Tools::PointWithCode> camPixels2 = avgCamsPixels[1][j];
			int size = camPixels1.size();
			vector<int> matches[2];
			matches[0].resize(0);
			matches[1].resize(0);
			for (int k = 0; k < size; k++) {
				Tools::PointWithCode pwc1 = camPixels1[k];
				if (pwc1.indexInSiftFile != -1) {
					map<int, int>::iterator iter;
					iter = codeMap.find(k);
					if (iter != codeMap.end()) {
						int codeFin = iter->second;
						int idx2 = camPixels2[codeFin].indexInSiftFile;
						if (idx2 != -1) {
							matches[0].push_back(pwc1.indexInSiftFile);
							matches[1].push_back(idx2);
						}
					}
				}
			}
			int matchSz = matches[0].size();
			ouF << i << imgType << " " << endImgIdx - sz[1] + 1 + j << imgType << " " << matchSz << "\n";
			for (int k = 0; k < 2; k++) {
				for (int l = 0; l < matchSz; l++) {
					ouF << matches[k][l] << " ";
				}
				ouF << "\n";
			}
		}
	}
	ouF.close();
}

void LoopClosing::loopClose()
{
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);
	vector<int> featureIdxList1;
	vector<int> featureIdxList2;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	int endImgIdx;
	int startImgIdx = 0;
	readNvmFile(source, target, featureIdxList1, featureIdxList2, startImgIdx, endImgIdx);
	//threeDpointMatching(source, target, cru_correspondences);
	threeDpointMatchingICP(source, target, cru_correspondences);
	map<int, int> codeMap;
	vector<Tools::PointWithCode>** avgCamsPixels = new vector<Tools::PointWithCode>*[2];
	vector<int> sz;
	sz.resize(0);
	readTwoGroupCode(avgCamsPixels, sz);
	outputFirstAndEndMatchesAndMakeCodeMap(cru_correspondences, featureIdxList1, featureIdxList2, startImgIdx, endImgIdx, codeMap, avgCamsPixels, sz[1]);
	//outputMatches(avgCamsPixels, sz, codeMap, endImgIdx);
}
