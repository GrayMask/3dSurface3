#include <ctime>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include "LoopClosing.h"
#include "Tools.h"
#include "Const.h"
#include "Path.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

bool readNvmFile(pointcloud::Ptr cloud1, pointcloud::Ptr cloud2, vector<int>& featureIdxList1, vector<int>& featureIdxList2, int& endImgIdx) {
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
		endImgIdx = imageMount - 1;
		if (Tools::goWithLine(in, imageMount + 2)) {
			in >> pointMount;
			for (int i = 0; i < pointMount; i++) {
				in >> x >> y >> z;
				Tools::goWithStep(in, 3);
				in >> featureMount;
				for (int j = 0; j < featureMount; j++) {
					in >> imageIdx >> featureIdx;
					Tools::goWithStep(in, 2);
					if (imageIdx == 0) {
						cloud1->push_back(pcl::PointXYZ(x, y, z));
						featureIdxList1.push_back(featureIdx);
					}
					else if(imageIdx == endImgIdx) {
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
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	est_normal.compute(*point_normal);
	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4); //指定4核计算
									// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}


int threeDpointMatching(pointcloud::Ptr source, pointcloud::Ptr target, boost::shared_ptr<pcl::Correspondences> cru_correspondences)
{
	clock_t start, end, time;
	start = clock();

	cout << source->width;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

	//pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	//sac_ia.setInputSource(source);
	//sac_ia.setSourceFeatures(source_fpfh);
	//sac_ia.setInputTarget(target);
	//sac_ia.setTargetFeatures(target_fpfh);
	//pointcloud::Ptr align(new pointcloud);
	////  sac_ia.setNumberOfSamples(20);  
	//sac_ia.setCorrespondenceRandomness(6); 
	//sac_ia.align(*align);

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;

	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	//  crude_cor_est.determineCorrespondences(cru_correspondences);
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	cout << "crude size is:" << cru_correspondences->size() << endl;

	end = clock();
	cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
	int v1;
	//int v2;

	view->createViewPort(0, 0.0, 1.0, 1.0, v1);
	//view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	//view->setBackgroundColor(0.05, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
	view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);
	view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(align, 255, 0, 0);
	//view->addPointCloud(align, aligend_cloud_color, "aligend_cloud_v2", v2);
	//view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "aligend_cloud_v2");
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

	view->addCorrespondences<pcl::PointXYZ>(source,target,*cru_correspondences,"correspondence",v1);
	while (!view->wasStopped())
	{
		// view->spin();
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));


	}
	//pcl::io::savePCDFile("crou_output.pcd", *align);
	//  pcl::io::savePCDFile ("final_align.pcd", *final);

	return 0;
}

void outputMatches(boost::shared_ptr<pcl::Correspondences> cru_correspondences, vector<int>& featureIdxList1, vector<int>& featureIdxList2, int endImgIdx) {
	int sz = cru_correspondences->size();
	ofstream ouF(root_dir + sfm_dir + "match.txt", ios::app);
	ouF << "0" << imgType << " " << endImgIdx << imgType << " " << sz << "\n";
	vector<int> matches[2];
	matches[0].resize(0);
	matches[1].resize(0);
	for (int i = 0; i < sz; i++) {
		int sourceI = cru_correspondences->at(i).index_query;
		int targetI = cru_correspondences->at(i).index_match;
		matches[0].push_back(featureIdxList1[sourceI]);
		matches[1].push_back(featureIdxList2[targetI]);
	}
	int matchSz = matches[0].size();
	for (int k = 0; k < 2; k++) {
		for (int l = 0; l < matchSz; l++) {
			ouF << matches[k][l] << " ";
		}
		ouF << "\n";
	}
}

void LoopClosing::loopClose()
{
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);
	vector<int> featureIdxList1;
	vector<int> featureIdxList2;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	int endImgIdx;
	readNvmFile(source, target, featureIdxList1, featureIdxList2, endImgIdx);
	threeDpointMatching(source, target, cru_correspondences);
	outputMatches(cru_correspondences, featureIdxList1, featureIdxList2, endImgIdx);
}

