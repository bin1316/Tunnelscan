#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include<stdlib.h>
#include <WinSock2.h> 
#include <windows.h>
#include <thread>
#include <boost/thread/win32/mutex.hpp>
#include <boost/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#pragma comment(lib, "ws2_32.lib") 

bool refresh = false;

union tofloat
{
	char data[12];
	float coor[3];
}myfloat;

//接收雷达返回信息
class ReceiveData
{
public:
	ReceiveData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) :cloud(cloud_)
	{
	}
	void start();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::thread *read_packet_thread;

	void processpacket();
	void read_packet();

	
};

void
ReceiveData::start()
{
	receivedata();

}
void 
ReceiveData::read_packet()
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

	while (1)
	{
		char recvBuf[500000];
		memset(recvBuf, 0, sizeof(recvBuf));
		//接收数据   
		recnum = recv(sockConn, recvBuf, sizeof(recvBuf), 0);
		if (!recnum)
		{
			Sleep(10);
			continue;
		}
		printf("%d\n", recnum);

		while (true)
		{
			if (cloud_mutex.try_lock())
			{
				//更新刷新标志及点云
				refresh = true;
				recnum /= 12;
				cloud->resize(recnum);
				cloud->width = recnum;
				cloud->height = 1;
				for (int i = 0; i < recnum; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						myfloat.data[j] = recvBuf[12 * i + 4 - j];
						myfloat.data[4 + j] = recvBuf[12 * i + 8 - j];
						myfloat.data[8 + j] = recvBuf[12 * i + 12 - j];
					}
					cloud->points[i].x = myfloat.coor[0];
					cloud->points[i].y = myfloat.coor[1];
					cloud->points[i].z = myfloat.coor[2];
				}
				cloud_mutex.unlock();
				break;
			}
			Sleep(10);
		}

	}
	closesocket(sockConn);
	closesocket(sockSrv);
	WSACleanup();
	system("pause");
}