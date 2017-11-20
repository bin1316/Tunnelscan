#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <Key_Point.h>
#include<pcl/io/pcd_io.h>
#include <time.h>

using namespace std;
#define MAX_LINE 1024
//���ز����������ĵ���
void Myloadpcdfile(pcl::PointCloud<pcl::PointXYZ> &cloud, char* filename)
{
	cloud.width = 0;
	cloud.clear();
	cloud.height = 1;
	FILE *fp = fopen(filename, "r");
	char buff[MAX_LINE];
	size_t len, pn = 0;
	unsigned int data_idx;
	string line;
	if (fp == NULL)
	{
		cout << "���ļ�ʧ�ܣ�" << endl;
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
			cout << "PCD�ļ�SIZE����" << endl;
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
		cout << "PCD�ļ�Head����" << endl;
		fclose(fp);
		return;
	}
	if (cloud.points.size() == 0)
	{
		cout << "PCD�ļ�����size=0��" << endl;
		fclose(fp);
		return;
	}
	line.clear();
	long fsize;
	fseek(fp, data_idx, SEEK_END);
	fsize = ftell(fp);
	fseek(fp, data_idx - 11, SEEK_SET);
	char *buf = (char*)malloc(sizeof(char)*fsize);
	if (buf == NULL)
	{
		cout << "�������������ݣ�" << endl;
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
		if (pn / 3 == cloud.width)break;
	}
	//boost::split(st, line, boost::is_any_of("\t\n "), boost::token_compress_on);
	if ((pn / 3) != cloud.points.size())
	{
		cout << "�ļ��е���������ƥ�䣡" << cloud.points.size() << " " << pn / 3 << endl;
		cloud.clear();
		fclose(fp);
		return;
	}
	fclose(fp);
}
//���ش��з������ĵ���
void MyloadpcdfileN(pcl::PointCloud<pcl::PointXYZ> &cloud, char* filename)
{
	cloud.width = 0;
	cloud.clear();
	cloud.height = 1;
	FILE *fp = fopen(filename, "r");
	char buff[MAX_LINE];
	size_t len, pn = 0;
	unsigned int data_idx;
	string line;
	if (fp == NULL)
	{
		cout << "���ļ�ʧ�ܣ�" << endl;
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
			cout << "PCD�ļ�SIZE����" << endl;
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
		cout << "PCD�ļ�Head����" << endl;
		fclose(fp);
		return;
	}
	if (cloud.points.size() == 0)
	{
		cout << "PCD�ļ�����size=0��" << endl;
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
		cout << "�������������ݣ�" << endl;
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
		cout << "�ļ��е���������ƥ�䣡" << cloud.points.size() << " " << pn / 7 << endl;
		cloud.clear();
		fclose(fp);
		return;
	}
	fclose(fp);

}
//���ش��з������ĵ���
void MyloadpcdfileN2(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::Normal> &normal, char* filename)
{
	cloud.width = 0;
	cloud.clear();
	cloud.height = 1;
	normal.width = 0;
	normal.clear();
	normal.height = 1;
	FILE *fp = fopen(filename, "r");
	char buff[MAX_LINE];
	size_t len, pn = 0;
	unsigned int data_idx;
	string line;
	if (fp == NULL)
	{
		cout << "���ļ�ʧ�ܣ�" << endl;
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
			normal.width = cloud.width;
			continue;
		}
		if (line.substr(0, 6) == "HEIGHT")
		{
			cloud.height = atoi(line.substr(6, line.length() - 6).c_str());
			normal.height = normal.height;
			continue;
		}
		if (line.substr(0, 9) == "VIEWPOINT")
			continue;
		if (line.substr(0, 6) == "POINTS")
		{
			if (cloud.height*cloud.width == atoi(line.substr(6, line.length() - 6).c_str()))
			{
				cloud.resize(cloud.width*cloud.height);
				normal.resize(cloud.width*cloud.height);
				continue;
			}
			cout << "PCD�ļ�SIZE����" << endl;
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
		cout << "PCD�ļ�Head����" << endl;
		fclose(fp);
		return;
	}
	if (cloud.points.size() == 0)
	{
		cout << "PCD�ļ�����size=0��" << endl;
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
		cout << "�������������ݣ�" << endl;
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
			case 3:{
					   normal.points[pn++ / 7].normal_x = atof(line.c_str());
					   break;
			}
			case 4:{
					   normal.points[pn++ / 7].normal_y = atof(line.c_str());
					   break;
			}
			case 5:{
					   normal.points[pn++ / 7].normal_z = atof(line.c_str());
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
		cout << "�ļ��е���������ƥ�䣡" << cloud.points.size() << " " << pn / 7 << endl;
		cloud.clear();
		normal.clear();
		fclose(fp);
		return;
	}
	fclose(fp);

}
//���ӻ�
void view(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud2, 255, 0, 0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addText("cloud1", 10, 10, "v1 text");
	viewer->addPointCloud(cloud1, "cloud1");
	viewer->addPointCloud(cloud2,color1, "cloud2");
	viewer->initCameraParameters();
	std::cout << "��ʾ���..." << std::endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

//
int main(){
	double t1, t2;
	int dis, lam;
	cout << "����mindis��minlambda :" << endl;
	cin >> dis >> lam;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
 	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint(new pcl::PointCloud<pcl::PointXYZ>);

	char rootpath[50] = "../../PCL_Data/1102/cloud_final.pcd";
	MyloadpcdfileN2(*cloud,*normal, rootpath);
	t1 = clock();//��ʱ��
	//��ȡ�ؼ���
	pcl::KeyPoint<pcl::PointXYZ> kp;
	kp.setInputCloud(cloud);
	kp.setNormal(normal);
	kp.setMeanK(50);
	kp.setStddevMulThresh(80);
	kp.setmindis(dis);
	kp.setMinlambda(lam);
	kp.filter(*keypoint);
	t2 = clock();//��ʱ��
	cout << "��ʱ : " << (t2 - t1) / CLOCKS_PER_SEC << endl;
	cout << *keypoint << endl;
	view(cloud, keypoint);
	return 0;
}