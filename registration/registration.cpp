// PCL.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <pcl/common/io.h>  
#include <pcl/io/io.h>  
#include <pcl/point_cloud.h>  
#include <pcl/io/ply_io.h>  
#include <pcl/io/ply/ply.h>  
#include <pcl/console/parse.h>  
#include <iostream>  
#include <string>  
#include <pcl/point_types.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/PolygonMesh.h>  
#include <pcl/io/ply_io.h>  
#include <pcl/io/vtk_lib_io.h>  
#include<pcl/common/transforms.h>
#include<pcl/common/centroid.h>
#include<mutex>
#include<thread>
//#include <pcl/io/vtk_lib_io.hpp>  
#include <pcl/io/vtk_io.h>  
#include <Eigen/SVD> 

using namespace std;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

void ply2pcd(string source, string des){
	pcl::PolygonMesh cloud;
	pcl::io::loadPolygonFilePLY(source, cloud);
	pcl::io::savePCDFile(des, cloud.cloud);
}
void transform(string source, string des){
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f transform_m = Eigen::Matrix4f::Identity();
	transform_m(0, 0) = -0.707;
	transform_m(0, 1) = -0.408;
	transform_m(0, 2) = 0.577;
	transform_m(1, 0) = 0.707;
	transform_m(1, 1) = -0.408;
	transform_m(1, 2) = 0.577;
	transform_m(2, 0) = 0;
	transform_m(2, 1) = 0.816;
	transform_m(2, 2) = 0.577;
	transform_m(0, 3) = -0.1;
	transform_m(1, 3) = 0.2;
	transform_m(2, 3) = -0.1;
	pcl::io::loadPCDFile(source, *source_cloud);
	int len = source_cloud->points.size();
	transformed_cloud->resize(len);
	for (int i = 0; i < len; i++){
		transformed_cloud->points[i].x = transform_m(0, 0)*source_cloud->points[i].x + transform_m(0, 1)*source_cloud->points[i].y +
			transform_m(0, 2)*source_cloud->points[i].z + transform_m(0, 3) + rand() % 100 * 0.00001;
		transformed_cloud->points[i].y = transform_m(1, 0)*source_cloud->points[i].x + transform_m(1, 1)*source_cloud->points[i].y +
			transform_m(1, 2)*source_cloud->points[i].z + transform_m(1, 3) + rand() % 100 * 0.00001;
		transformed_cloud->points[i].z = transform_m(2, 0)*source_cloud->points[i].x + transform_m(2, 1)*source_cloud->points[i].y +
			transform_m(2, 2)*source_cloud->points[i].z + transform_m(2, 3) + rand() % 100 * 0.00001;
	}
	pcl::io::savePCDFile(des, *transformed_cloud);
}
mutex mu;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix4f updatetranform(Eigen::Matrix3f R, Eigen::Vector3f t){
	Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
	res(0, 0) = R(0, 0), res(0, 1) = R(0, 1), res(0, 2) = R(0, 2), res(0, 3) = t(0);
	res(1, 0) = R(1, 0), res(1, 1) = R(1, 1), res(1, 2) = R(1, 2), res(1, 3) = t(1);
	res(2, 0) = R(2, 0), res(2, 1) = R(2, 1), res(2, 2) = R(2, 2), res(2, 3) = t(2);
	return res;
}
void match(){
	Eigen::Vector4f centroid_src, centroid_tra;
	Eigen::Vector3f p,p_dot,t;
	Eigen::Matrix4f transform_m=Eigen::Matrix4f::Identity();
	Eigen::Matrix3f H = Eigen::Matrix3f::Zero(), U, V, R;
	pcl::compute3DCentroid(*cloud, centroid_src);
	pcl::compute3DCentroid(*cloud_ori, centroid_tra);
	p(0) = centroid_src(0), p(1) = centroid_src(1), p(2) = centroid_src(2);
	p_dot(0) = centroid_tra(0), p_dot(1) = centroid_tra(1), p_dot(2) = centroid_tra(2);
	int len = cloud->points.size();
	for (int j = 0; j < 2000; j++){
		getchar();
		//计算H矩阵
		H = Eigen::Matrix3f::Zero();
		for (int i = 99; i < len; i += 100){
			H(0, 0) += (cloud_ori->points[i].x - p_dot(0))*(cloud->points[i].x - p(0));
			H(0, 1) += (cloud_ori->points[i].x - p_dot(0))*(cloud->points[i].y - p(1));
			H(0, 2) += (cloud_ori->points[i].x - p_dot(0))*(cloud->points[i].z - p(2));
			H(1, 0) += (cloud_ori->points[i].y - p_dot(1))*(cloud->points[i].x - p(0));
			H(1, 1) += (cloud_ori->points[i].y - p_dot(1))*(cloud->points[i].y - p(1));
			H(1, 2) += (cloud_ori->points[i].y - p_dot(1))*(cloud->points[i].z - p(2));
			H(2, 0) += (cloud_ori->points[i].z - p_dot(2))*(cloud->points[i].x - p(0));
			H(2, 1) += (cloud_ori->points[i].z - p_dot(2))*(cloud->points[i].y - p(1));
			H(2, 2) += (cloud_ori->points[i].z - p_dot(2))*(cloud->points[i].z - p(2));
		}
		//奇异值分解
		Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
		R = V*U.transpose();
		cout <<"R: "<< R << endl;
		cout << endl;
		t = p - R*p_dot;
		transform_m = updatetranform(R, t);
		mu.lock();
		pcl::transformPointCloud<pcl::PointXYZ>(*cloud_ori, *cloud_ori, transform_m);
		mu.unlock();
		p_dot = R*p_dot + t;
	}
}
void view(){
	int v1(0), v2(0);
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_ori, 255, 0, 0);
	viewer->setBackgroundColor(0, 0, 0);
//	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud 1");
//	viewer->addPointCloud<pcl::PointXYZ>(cloud_ori, color1, "cloud 2", v1);
//	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_ori, color1, "cloud 2");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		mu.lock();
		viewer->updatePointCloud(cloud_ori, color1, "cloud 2");
		viewer->spinOnce(100);
		mu.unlock();
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
int main(int argc, char** argv)
{
//	transform("../data/bunny.pcd", "../data/bunny_transform.pcd");
//	cout << "Done!" << endl;
	string filename = "../data/bunny.pcd";
	pcl::io::loadPCDFile(filename, *cloud);
	pcl::io::loadPCDFile("../data/bunny_transform.pcd", *cloud_ori);

	thread t1(match);
	thread t2(view);
	t1.join();
	t2.join();
	
	return (0);
}
