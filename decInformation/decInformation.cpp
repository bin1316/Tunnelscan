// decInformation.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <WinSock2.h> 
#include <windows.h>
#include <boost/thread.hpp>
#include <boost/thread/win32/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/vtk_lib_io.h>
#pragma comment(lib, "ws2_32.lib") 

#define MAX_LINE 1024

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr stdcloud(new pcl::PointCloud<pcl::PointXYZ>);
boost::mutex cloud_mutex;
bool refresh = false;
bool stopread = false;

union tofloat
{
	unsigned char data[4];
	float coor;
}myfloat;

void Myloadpcdfile(pcl::PointCloud<pcl::PointXYZ> &cloud, char* filename)
{
	cloud.width = 0;
	cloud.clear();
	cloud.height = 1;
	FILE *fp = fopen(filename, "r");
	char buff[MAX_LINE];
	int len, pn = 0;
	unsigned int data_idx;
	std::string line;
	if (fp == NULL)
	{
		std::cout << "打开文件失败！" << std::endl;
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
		std::cout << "PCD文件Head错误！" << std::endl;
		fclose(fp);
		return;
	}
	if (cloud.points.size() == 0)
	{
		std::cout << "PCD文件错误：size=0！" << std::endl;
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
		std::cout << "不包括点云数据！" << std::endl;
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
		std::cout << "文件中点云数量不匹配！" << cloud.points.size() << " " << pn / 3 << std::endl;
		cloud.clear();
		fclose(fp);
		return;
	}
	fclose(fp);
}

void MyloadpcdfileN(pcl::PointCloud<pcl::PointXYZ> &cloud, char* filename)
{
	cloud.width = 0;
	cloud.clear();
	cloud.height = 1;
	FILE *fp = fopen(filename, "r");
	char buff[1024];
	int len, pn = 0;
	unsigned int data_idx;
	std::string line;
	if (fp == NULL)
	{
		cout << "打开文件失败！" << endl;
		fclose(fp);
		return;
	}
	while (!feof(fp))
	{
		line = "";
		fgets(buff, 1024, fp);
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
void receivedata()
{
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		printf("Failed to load Winsock");
		return;
	}
	//创建用于监听的套接字   
	SOCKET sockSrv = socket(AF_INET, SOCK_STREAM, 0);
	int port = 1507;
	SOCKADDR_IN addrSrv;
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(port); //1024以上的端口号   
	addrSrv.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

	int retVal = bind(sockSrv, (LPSOCKADDR)&addrSrv, sizeof(SOCKADDR_IN));
	if (retVal == SOCKET_ERROR){
		printf("Failed bind:%d\n", WSAGetLastError());
		return;
	}

	if (listen(sockSrv, 10) == SOCKET_ERROR){
		printf("Listen failed:%d", WSAGetLastError());
		return;
	}

	SOCKADDR_IN addrClient;
	int len = sizeof(SOCKADDR);
	int recnum;
	//等待客户请求到来     
	SOCKET sockConn = accept(sockSrv, (SOCKADDR *)&addrClient, &len);
	if (sockConn == SOCKET_ERROR){
		printf("Accept failed:%d", WSAGetLastError());
		return;
	}

	fd_set rfd;
	struct timeval timeout;
	timeout.tv_sec = 3;
	timeout.tv_usec = 0;

	char recvBuf[500000];
	while (!stopread)
	{
		memset(recvBuf, 0, sizeof(recvBuf));
		//接收数据   
		recnum = recv(sockConn, recvBuf, sizeof(recvBuf), 0);
		if (!recnum)
		{
			FD_ZERO(&rfd);
			if (select(sockSrv + 1, &rfd, 0, 0, &timeout) < 0)
			{
				stopread = true;
			}
			Sleep(10);
			continue;
		}
		printf("%d\n", recnum);

		if (recnum%12==0)
		{
			while (1)
			{
				if (cloud_mutex.try_lock())
				{
					//更新刷新标志及点云
					refresh = true;
					recnum /= 12;
					cloud->resize(recnum);
					cloud->width = recnum;
					cloud->height = 1;
					for (int i = 0; i < recnum * 3; i++)
					{
						for (int j = 0; j < 4; j++)
						{
							myfloat.data[j] = recvBuf[4 * i + 3 - j];
						}
						switch (i % 3)
						{
						case 0:
							cloud->points[i / 3].x = myfloat.coor;
							break;
						case 1:
							cloud->points[i / 3].y = myfloat.coor;
							break;
						case 2:
							cloud->points[i / 3].z = myfloat.coor;
							cloud->points[i / 3].r = 255;
							break;
						default:
							break;
						}
					}
					cloud_mutex.unlock();
					break;
				}
				Sleep(10);
			}

		}
	}
	printf("stopread!\n");
	closesocket(sockConn);
	closesocket(sockSrv);
	WSACleanup();
}

void view()
{
	int v1(0), v2(0),v3(0);
	pcl::PolygonMesh triangles;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPolygonFile("../../PCData/0608/mesh.vtk", triangles);
	Myloadpcdfile(*cloud1, "../../PCData/0608/RadarData0608.pcd");

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> Prgb;
	viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v1);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addText("cloud1", 10, 10, "v1 text");
	viewer->addPointCloud(cloud1, "model1");

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addText("cloud2", 10, 10, "v2 text", v2);
	viewer->addPointCloud(stdcloud, "model2", v2);


	viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v3);
	viewer->setBackgroundColor(0, 0, 0, v3);
	viewer->addText("after triangles", 10, 10, "v3 text", v3);
	viewer->addPolygonMesh(triangles, "mesh", v3);

	viewer->initCameraParameters();
	std::cout << "显示完成..." << std::endl;
	int count = 0;
	bool a = false;
	char cloudname[30];
	while (!viewer->wasStopped())
	{
		if (refresh&&cloud)
		{
			count %= 20;
			count++;
			sprintf(cloudname, "cloud%d", count);
			boost::mutex::scoped_lock lock(cloud_mutex);			
			Prgb.setInputCloud(cloud);
			if (cloud->points.size() == 1)
			{
				viewer->removePointCloud(cloudname);
			}
			else
			{
				if (!viewer->updatePointCloud(cloud, cloudname))
				{
					viewer->addPointCloud(cloud, cloudname);
					printf("addcloud%d!\n", count);
				}
				else
				{
					printf("updatecloud%d!\n", count);
				}
			}
			refresh = false;
		}
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	stopread = true;
}
void printnm()
{
	for (int i = 0; i < 10; i++)
	{
		Sleep(100);
		printf("nihao!\n");
	}
}
void printnm2()
{
	for (int i = 0; i < 10; i++)
	{
		Sleep(100);
		printf("buhao!\n");
	}
}
int _tmain(int argc, _TCHAR* argv[])
{
	MyloadpcdfileN(*stdcloud, "../../PCdata/0608/cloud_track.pcd");
	boost::thread trd(&receivedata);
	boost::thread trd2(&view);
	trd.join();
	trd2.join();
	printf("jieshu\n");

	//boost::function<void(void)>
	return 0;
}

