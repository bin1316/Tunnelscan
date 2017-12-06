#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <KeyPointExtraction.h>
#include<PCL_io.h>
#include <time.h>

#include <vld.h>
#pragma comment(lib,"vld.lib")
using namespace std;

//可视化
void view(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud2, 255, 0, 0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addText("cloud1", 10, 10, "v1 text");
	viewer->addPointCloud(cloud1, "cloud1");
	viewer->addPointCloud(cloud2, color1, "cloud2");
	viewer->initCameraParameters();
	std::cout << "显示完成..." << std::endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

//
int main(){
	double t1, t2;
	std::vector<int> key;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint(new pcl::PointCloud<pcl::PointXYZ>);

	
	MyloadpcdfileN2(*cloud, *normal, "../../PCL_Data/120102/cloud_final.pcd");
	t1 = clock();//计时器
	//提取关键点
	pcl::KeyPointExtaction<pcl::PointXYZ> kp;
	kp.setInputCloud(cloud);
	kp.setNormal(normal);
	kp.setMeanK(100);
	kp.setStddevMulThresh(80);
	kp.setmindis(6);
	kp.setMinlambda(100);
	kp.filter(*keypoint);
	kp.getKeyIndice(key);
	t2 = clock();//计时器
	cout << "用时 : " << (t2 - t1) / CLOCKS_PER_SEC << endl;
	cout << *keypoint << endl;
	view(cloud, keypoint);
	return 0;
}