#include <iostream>
#include <fstream>
#include <windows.h>
#include<string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;

#define MAX_LINE 1024

//加载不带法向量的点云
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
	fseek(fp, data_idx, SEEK_SET);
	char *buf = (char*)malloc(sizeof(char)*fsize);
	if (buf == NULL)
	{
		cout << "不包括点云数据！" << endl;
		fclose(fp);
		delete buf;
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
					  cloud.points[pn++ / 3].x = atof(line.c_str());
					  break;
			}
			case 1:
			{
					  cloud.points[pn++ / 3].y = atof(line.c_str());
					  break;
			}
			case 2:
			{
					  cloud.points[pn++ / 3].z = atof(line.c_str());
					  break;
			}
			default:
				break;
			}
			line = "";
		}
		if (pn / 3 == cloud.width)break;
	}
	delete buf;
	//boost::split(st, line, boost::is_any_of("\t\n "), boost::token_compress_on);
	if ((pn / 3) != cloud.points.size())
	{
		cout << "文件中点云数量不匹配！" << cloud.points.size() << " " << pn / 3 << endl;
		cloud.clear();
		fclose(fp);
		return;
	}
	fclose(fp);
}
//加载带有法向量的点云
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
		delete buf;
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
	delete buf;
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
//加载带有法向量的点云
void MyloadpcdfileN(pcl::PointCloud<pcl::PointNormal> &cloud, char* filename)
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
		delete buf;
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
			case 3:
			{
					  cloud.points[pn++ / 7].normal_x = atof(line.c_str());
					  break;
			}
			case 4:
			{
					  cloud.points[pn++ / 7].normal_y = atof(line.c_str());
					  break;
			}
			case 5:
			{
					  cloud.points[pn++ / 7].normal_z = atof(line.c_str());
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
	delete buf;
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
//加载带有法向量的点云
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
		delete buf;
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
	delete buf;
	//boost::split(st, line, boost::is_any_of("\t\n "), boost::token_compress_on);
	if ((pn / 7) != cloud.points.size())
	{
		cout << "文件中点云数量不匹配！" << cloud.points.size() << " " << pn / 7 << endl;
		cloud.clear();
		normal.clear();
		fclose(fp);
		return;
	}
	fclose(fp);

}
