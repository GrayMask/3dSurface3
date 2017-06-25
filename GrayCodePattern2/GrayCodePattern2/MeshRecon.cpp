#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include "MeshRecon.h"
#include "Const.h"
#include "Path.h"
#include "Tools.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;

void filterPointCloud(pointcloud::Ptr input, pointcloud::Ptr output) {
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	//sor.setInputCloud(input);
	//sor.setMeanK(70);
	//sor.setStddevMulThresh(0.9);
	//sor.filter(*output);
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	outrem.setInputCloud(input);
	outrem.setRadiusSearch(0.015);
	outrem.setMinNeighborsInRadius(30);
	// apply filter
	outrem.filter(*output);
}
//
//void MeshRecon::outputPlyFromNvm() {
//	
//
//	// trim points
//	pointcloud::Ptr trimedCloud(new pointcloud);
//	filterPointCloud(pointCloud, trimedCloud);
//
//	// compute normals
//	pointnormal::Ptr point_normal(new pointnormal);
//	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
//	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> est_normal;
//	est_normal.setInputCloud(trimedCloud);
//	est_normal.setSearchMethod(tree);
//	est_normal.setKSearch(10);
//	est_normal.compute(*point_normal);
//
//	// save ply file
//	Tools::savePointCloudInPly(trimedCloud, point_normal);
//}


void MeshRecon::poissonRecon()
{
	pointcloud::Ptr pointCloud(new pointcloud);
	Tools::readPointCloudFromNvm(pointCloud);
	//pointcloud::Ptr trimedCloud(new pointcloud);
	//filterPointCloud(pointCloud, trimedCloud);
	//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	//pointnormal::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//tree->setInputCloud(pointCloud);
	//n.setInputCloud(pointCloud);
	//n.setSearchMethod(tree);
	//n.setKSearch(20);
	//n.compute(*normals);

	//pcl::concatenateFields(*pointCloud, *normals, *cloud_with_normals);
	pcl::PLYWriter writer;
	//writer.write(root_dir + expr_dir + "point.ply", *cloud_with_normals);
	writer.write(root_dir + expr_dir + "point.ply", *pointCloud);

	//pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	//tree2->setInputCloud(cloud_with_normals);

	//pcl::Poisson<pcl::PointXYZRGBNormal> pn;
	//pn.setConfidence(false);
	//pn.setDegree(2);
	//pn.setDepth(8);
	//pn.setIsoDivide(8);
	//pn.setManifold(false);
	//pn.setOutputPolygons(false);
	//pn.setSamplesPerNode(3.0);
	//pn.setScale(0.25);
	//pn.setSolverDivide(8);
	////pn.setIndices();

	//pn.setSearchMethod(tree2);
	//pn.setInputCloud(cloud_with_normals);
	//pcl::PolygonMesh mesh;
	//pn.performReconstruction(mesh);

	//pcl::io::savePLYFile(root_dir + expr_dir + plyFileName, mesh);
}
