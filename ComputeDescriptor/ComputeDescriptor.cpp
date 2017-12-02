// ComputeDescriptor.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <KeyPointExtraction.h>
#include <PCL_io.h>
#include <Descriptor.h>
#include <RoughRegistration.h>
//#include <vector>
#include <time.h>

#include <vld.h>
#pragma comment(lib,"vld.lib")
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_track1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_track2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_track1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_track2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_key(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_key(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr model_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	std::vector<int> key_model,key_cloud;
	std::vector<pair<int, int>> corres_point;
	Myloadpcdfile(*model_track1, "../../PCL_Data/120101/track1.pcd");
	Myloadpcdfile(*model_track2, "../../PCL_Data/120101/track2.pcd");
	Myloadpcdfile(*cloud_track1, "../../PCL_Data/120102/track1.pcd");
	Myloadpcdfile(*cloud_track2, "../../PCL_Data/120102/track2.pcd");
	MyloadpcdfileN2(*model, *model_normal, "../../PCL_Data/111701/cloud_final.pcd");
	MyloadpcdfileN2(*cloud, *cloud_normal, "../../PCL_Data/111702/cloud_final.pcd");

	int t1, t2;
	t1 = clock();

	//pcl::RoughRegistration<pcl::PointXYZ> rr;
	//rr.setInputModel(model, model_track1, model_track2);
	//rr.setInputCloud(cloud, cloud_track1, cloud_track2, cloud_normal);
	//rr.applyRoughRegistration();

	pcl::KeyPointExtaction<pcl::PointXYZ> kp_model;
	kp_model.setInputCloud(model);
	kp_model.setNormal(model_normal);
	kp_model.setMeanK(50);
	kp_model.setStddevMulThresh(80);
	kp_model.setmindis(8);
	kp_model.setMinlambda(50);
	kp_model.filter(*model_key);
	kp_model.getKeyIndice(key_model);

	pcl::KeyPointExtaction<pcl::PointXYZ> kp_cloud;
	kp_cloud.setInputCloud(cloud);
	kp_cloud.setNormal(cloud_normal);
	kp_cloud.setMeanK(50);
	kp_cloud.setStddevMulThresh(80);
	kp_cloud.setmindis(8);
	kp_cloud.setMinlambda(50);
	kp_cloud.filter(*cloud_key);
	kp_cloud.getKeyIndice(key_cloud);
	
	pcl::Descriptor<pcl::PointXYZ> de;
	de.setInputCloud(model, cloud);
	de.setInputKey(key_model, key_cloud);
	de.setMeanK(100);
	de.setStddevMulThresh(80);
	de.buildModelIndex();
	de.matchKeyPoint();
	de.applytransform(output, 1,true);
	de.applytransform(output, 4,true);
	de.applytransform(output, 16,true);
	de.applytransform(output, 32,true);
	//de.applytransform(output, 256);
	de.getKeyPair(corres_point);

	t2 = clock();
	cout << (float)(t2 - t1) / CLOCKS_PER_SEC << endl;

	//可视化对应点
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Prgbcloud(cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Prgboutput(output, 255, 0, 0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addText("cloud1", 10, 10, "text");
	viewer->addPointCloud(model, "model");
	//viewer->addPointCloud(cloud, Prgbcloud, "cloud");
	viewer->addPointCloud(output, Prgboutput, "output");
	pcl::PointXYZ a, b;
	cout << "匹配关键点对" << corres_point.size() << endl;
	for (int i = 0; i < corres_point.size(); i++){
		a.x = model->points[corres_point[i].first].x;
		a.y = model->points[corres_point[i].first].y;
		a.z = model->points[corres_point[i].first].z;
		b.x = cloud->points[corres_point[i].second].x;
		b.y = cloud->points[corres_point[i].second].y;
		b.z = cloud->points[corres_point[i].second].z;
		char str[10];
		sprintf(str, "%d", i);
		viewer->addLine(a, b, 0, 255, 0, str);
	}
	viewer->initCameraParameters();
	std::cout << "显示完成..." << std::endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

