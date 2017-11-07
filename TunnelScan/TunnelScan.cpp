// TunnelScan.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include"SamePointRemove.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <PointCloudSparse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include<pcl/io/pcd_io.h>
#include <time.h>

#include <pcl/io/vtk_lib_io.h>
using namespace std;

#define MAX_LINE 1024

void Myloadpcdfile(pcl::PointCloud<pcl::PointXYZ> &cloud, char* filename)
{
	cloud.width = 0;
	cloud.clear();
	cloud.height = 1;
	FILE *fp = fopen(filename, "r");
	char buff[MAX_LINE];
	int len, pn = 0;
	unsigned int data_idx;
	string line;
	if (fp == NULL)
	{
		cout << "打开文件失败！" << endl;
		fclose(fp);
		return;
	}
	while (!feof(fp))
	{
		line = "";
		fgets(buff, MAX_LINE, fp);
		len = strlen(buff);
		for (int i = 0; i < len; i++)
			line += buff[i];
		if (line == "")continue;
		boost::trim(line);
		if (line.substr(0, 1) == "#")
			continue;
		if (line.substr(0, 7) == "VERSION")
			continue;
		if ((line.substr(0, 6) == "FIELDS") || (line.substr(0, 7) == "COLUMNS"))
			continue;
		if (line.substr(0, 4) == "SIZE")
			continue;
		if (line.substr(0, 4) == "TYPE")
			continue;
		if (line.substr(0, 5) == "COUNT")
			continue;
		if (line.substr(0, 5) == "WIDTH")
		{
			cloud.width = atoi(line.substr(5, line.length() - 5).c_str());
			continue;
		}
		if (line.substr(0, 6) == "HEIGHT")
		{
			cloud.height = atoi(line.substr(6, line.length() - 6).c_str());
			continue;
		}
		if (line.substr(0, 9) == "VIEWPOINT")
			continue;
		if (line.substr(0, 6) == "POINTS")
		{
			if (cloud.height*cloud.width == atoi(line.substr(6, line.length() - 6).c_str()))
			{
				cloud.resize(cloud.width*cloud.height);
				continue;
			}
			cout << "PCD文件SIZE错误！" << endl;
			return;
		}
		if (line.substr(0, 4) == "DATA")
		{
			data_idx = static_cast<int>(ftell(fp));
			break;
		}
	}
	if (feof(fp))
	{
		cout << "PCD文件Head错误！" << endl;
		fclose(fp);
		return;
	}
	if (cloud.points.size() == 0)
	{
		cout << "PCD文件错误：size=0！" << endl;
		fclose(fp);
		return;
	}
	line.clear();
	long fsize;
	fseek(fp, data_idx, SEEK_END);
	fsize = ftell(fp);
	fseek(fp, data_idx-11, SEEK_SET);
	char *buf = (char*)malloc(sizeof(char)*fsize);
	if (buf == NULL)
	{
		cout << "不包括点云数据！" << endl;
		fclose(fp);
		return;
	}
	fread(buf, 1, fsize, fp);
	for (long i = 0; i < fsize; i++)
	{
		if (buf[i] != ' '&&buf[i] != '\t'&&buf[i] != '\n')
			line += buf[i];
		else if (line != "")
		{
			switch (pn % 3)
			{
			case 0:
			{
					  cloud.points[pn / 3].x = atof(line.c_str());
					  break;
			}
			case 1:
			{
					  cloud.points[pn / 3].y = atof(line.c_str());
					  break;
			}
			case 2:
			{
					  cloud.points[pn / 3].z = atof(line.c_str());
					  break;
			}
			default:
				break;
			}
			line = "";
			pn++;
		}
		if (pn/3==cloud.width)break;
	}
	//boost::split(st, line, boost::is_any_of("\t\n "), boost::token_compress_on);
	if ((pn / 3 ) != cloud.points.size())
	{
		cout << "文件中点云数量不匹配！" << cloud.points.size() << " " << pn / 3 << endl;
		cloud.clear();
		fclose(fp);
		return;
	}
	fclose(fp);
}

void savepcdfile2(pcl::PointCloud<pcl::PointXYZ> &cloud, char* filename)
{
	//char buf[1024 * 10];
	FILE *fp;
	fp = fopen(filename, "w");
	if (fp == NULL)
	{
		cout << "打开文件失败！" << endl;
		return;
	}
	fprintf(fp, "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n", cloud.points.size(), cloud.points.size());
	for (int i = 0; i < cloud.points.size(); i++)
	{
		fprintf(fp, "%.2f\t%.2f\t%.2f\n", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
	}
	fclose(fp);
}

void MyloadpcdfileN(pcl::PointCloud<pcl::PointXYZ> &cloud, char* filename)
{
	cloud.width = 0;
	cloud.clear();
	cloud.height = 1;
	FILE *fp = fopen(filename, "r");
	char buff[MAX_LINE];
	int len, pn = 0;
	unsigned int data_idx;
	string line;
	if (fp == NULL)
	{
		cout << "打开文件失败！" << endl;
		fclose(fp);
		return;
	}
	while (!feof(fp))
	{
		line = "";
		fgets(buff, MAX_LINE, fp);
		data_idx = ftell(fp);
		len = strlen(buff);
		for (int i = 0; i < len; i++)
			line += buff[i];
		if (line == "")continue;
		boost::trim(line);
		if (line.substr(0, 1) == "#")
			continue;
		if (line.substr(0, 7) == "VERSION")
			continue;
		if ((line.substr(0, 6) == "FIELDS") || (line.substr(0, 7) == "COLUMNS"))
			continue;
		if (line.substr(0, 4) == "SIZE")
			continue;
		if (line.substr(0, 4) == "TYPE")
			continue;
		if (line.substr(0, 5) == "COUNT")
			continue;
		if (line.substr(0, 5) == "WIDTH")
		{
			cloud.width = atoi(line.substr(5, line.length() - 5).c_str());
			continue;
		}
		if (line.substr(0, 6) == "HEIGHT")
		{
			cloud.height = atoi(line.substr(6, line.length() - 6).c_str());
			continue;
		}
		if (line.substr(0, 9) == "VIEWPOINT")
			continue;
		if (line.substr(0, 6) == "POINTS")
		{
			if (cloud.height*cloud.width == atoi(line.substr(6, line.length() - 6).c_str()))
			{
				cloud.resize(cloud.width*cloud.height);
				continue;
			}
			cout << "PCD文件SIZE错误！" << endl;
			return;
		}
		if (line.substr(0, 4) == "DATA")
		{
			data_idx = ftell(fp);
			break;
		}
	}
	if (feof(fp))
	{
		cout << "PCD文件Head错误！" << endl;
		fclose(fp);
		return;
	}
	if (cloud.points.size() == 0)
	{
		cout << "PCD文件错误：size=0！" << endl;
		fclose(fp);
		return;
	}
	line.clear();
	long fsize;
	fseek(fp, data_idx, SEEK_END);
	fsize = ftell(fp);
	fseek(fp, data_idx, SEEK_SET);
	char *buf = (char*)malloc(sizeof(char)*fsize);
	if (buf == NULL)
	{
		cout << "不包括点云数据！" << endl;
		fclose(fp);
		return;
	}
	fread(buf, 1, fsize, fp);
	for (long i = 0; i < fsize; i++)
	{
		if (buf[i] != ' '&&buf[i] != '\t'&&buf[i] != '\n')
			line += buf[i];
		else if (line != "")
		{
			switch (pn % 7)
			{
			case 0:
			{
					  cloud.points[pn++ / 7].x = atof(line.c_str());
					  break;
			}
			case 1:
			{
					  cloud.points[pn++ / 7].y = atof(line.c_str());
					  break;
			}
			case 2:
			{
					  cloud.points[pn++ / 7].z = atof(line.c_str());
					  break;
			}
			default:
				pn++;
				break;
			}
			line = "";
			if (pn / 7 == cloud.width)
				break;
		}

	}
	//boost::split(st, line, boost::is_any_of("\t\n "), boost::token_compress_on);
	if ((pn / 7) != cloud.points.size())
	{
		cout << "文件中点云数量不匹配！" << cloud.points.size() << " " << pn / 7 << endl;
		cloud.clear();
		fclose(fp);
		return;
	}
	fclose(fp);

}

int _tmain(int argc, _TCHAR* argv[])
{
	float t0, t1, t2,t3;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_SPR(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PCS(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_after_MLS(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_after_MLS2(new pcl::PointCloud<pcl::PointNormal>);

	pcl::PCDWriter writer;

	t0 = clock();
	Myloadpcdfile(*cloud, "../../PCdata/0802/RadarData.pcd");
	//pcl::io::savePCDFile("../PCdata/RadarData2.pcd", *cloud);
	t1 = clock();
	std::cerr << "加载用时" << (t1 - t0) / CLOCKS_PER_SEC << endl;
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;



	//点云简化
	pcl::PointCloudSparse<pcl::PointXYZ> PCS;
	PCS.setInputCloud(cloud);//##############修改
	PCS.setStddevMulThresh(20);//设置点云的最小距离
	PCS.setMeanK(40);//设置查询的邻近点数
	PCS.filter(*cloud_after_PCS);
	std::cerr << "cloud after PCS" << endl;
	std::cerr << *cloud_after_PCS << std::endl;
	cloud_filtered->clear();

	t2 = clock();
	std::cerr << "点云简化用时：" << (t2 - t1) / CLOCKS_PER_SEC << endl;//##############修改
	//pcl::io::savePCDFile("../../PCdata/0802/cloud_after_PCS.pcd", *cloud_after_PCS);
	pcl::io::savePCDFile("../../PCdata/0802/cloud_model.pcd", *cloud_after_PCS);

	// 去除离群点
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_after_PCS);
	sor2.setMeanK(20);//设置在进行统计时考虑查询点邻近点数
	sor2.setStddevMulThresh(3.0);//设置判断是否为离群点的阈值,如果一个点的距离超出平均距离一个标准差以上
	sor2.filter(*cloud_filtered);//执行滤波处理保存内点到cloud_filtered
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	cloud_after_PCS->clear();
	t3 = clock();
	std::cerr << "去除离群点2用时：" << (t3 - t2) / CLOCKS_PER_SEC << endl;
	//pcl::io::savePCDFile("../../PCdata/0802/cloud_filtered2.pcd", *cloud_filtered);

	//点云平滑
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// Create a KD-Tree
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud_filtered);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(50);
	mls.process(*cloud_after_MLS);
	std::cerr << "cloud after MLS" << endl;
	std::cerr << *cloud_after_MLS << std::endl;
	t1 = clock();
	std::cerr << "点云平滑用时：" << (t1 - t3) / CLOCKS_PER_SEC << endl;
	pcl::io::savePCDFile("../../PCdata/0802/cloud_track.pcd", *cloud_after_MLS);
	t2 = clock();
	std::cerr << "存储处理后的点云用时：" << (t2 - t1) / CLOCKS_PER_SEC << endl;

	//平滑
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree3(new pcl::search::KdTree<pcl::PointNormal>);// Create a KD-Tree
	pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls2;
	mls2.setInputCloud(cloud_after_MLS);
	mls2.setPolynomialFit(true);
	mls2.setSearchMethod(tree3);
	mls2.setSearchRadius(100);
	mls2.process(*cloud_after_MLS2);
	std::cerr << "cloud after MLS2" << endl;
	std::cerr << *cloud_after_MLS2 << std::endl;
	t1 = clock();
	std::cerr << "点云平滑用时：" << (t1 - t2) / CLOCKS_PER_SEC << endl;
	pcl::io::savePCDFile("../../PCdata/0802/cloud_final.pcd", *cloud_after_MLS2);
	t2 = clock();
	std::cerr << "存储处理后的点云用时：" << (t2 - t1) / CLOCKS_PER_SEC << endl;
	//贪婪三角形重建
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud_after_MLS2);                //利用点云构建搜索树
	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	//pcl::PolygonMesh triangles;                               //存储最终三角化的网格模型
	////设置参数
	//gp3.setSearchRadius(200);//K近邻搜索范围
	//gp3.setMu(4);//设置被样本点搜索其邻近点的最远距离为2.5，为了适应点云密度的变化
	//gp3.setMaximumNearestNeighbors(100); //设置样本点可搜索的邻域个数为100
	//gp3.setMaximumSurfaceAngle(M_PI / 2);   //设置某点法线方向偏离样本点法线方向的最大角度为45度
	//gp3.setMinimumAngle(M_PI / 36);         //设置三角化后得到三角形内角最小角度为20度
	//gp3.setMaximumAngle(5 * M_PI /6);        //设置三角化后得到三角形内角最大角度为120度
	//gp3.setNormalConsistency(false);       //设置该参数保证法线朝向一致
	//gp3.setInputCloud(cloud_after_MLS2);//设置输入点云为有向点云cloud_with_normals
	//gp3.setSearchMethod(tree2);           //设置搜索方式为tree2
	//gp3.reconstruct(triangles);           //重建提取三角化(error)

	//t1 = clock();
	//std::cerr << "贪婪三角形重建用时：" << (t1 - t2) / CLOCKS_PER_SEC << endl;
	//pcl::io::saveVTKFile("../../PCdata/0802/mesh.vtk", triangles);
	//t2 = clock();
	//std::cerr << "模型存储用时：" << (t2 - t1) / CLOCKS_PER_SEC << endl;
	//std::cerr << "总共用时：" << (t2 - t0) / CLOCKS_PER_SEC << endl;
	////可视化
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->addText("original", 10, 10, "v1 text");
	//viewer->addPolygonMesh(triangles, "mesh");
	//viewer->initCameraParameters();
	//cout << "显示完成..." << endl;
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	return 0;
}

