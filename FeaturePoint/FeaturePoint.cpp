// FeaturePoint.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "FeaturePoint.h"
#include<iostream>
#include <vector>
using namespace std;
void DataExtract(double* dist,  int* indices, int* isize)
{
	vector<vector<int>> Pcluster;
	vector<int> Pclm;
	*isize = 0;
	for (int k = 0; k < 16; k++)
	{
		Pcluster.clear();
		Pclm.clear();
		int fp = -1, ep = -1;//首末个非零点的indice
		int invalidPoint = 0;
		for (int i = 24*k; i < 24*(k+1); i++)
		{
			if (fp == -1 && *(dist + i) != 0)
			{
				fp = i;
			}
			if (*(dist + i) != 0)
			{
				ep = i;
			}
		}
		if (fp == -1)
		{
			continue;
		}
		if (fp == ep)
		{
			*indices = fp;
			*isize = 1;
			continue;
		}
		Pclm.push_back(fp);
		for (int i = fp + 1; i < ep + 1; i++)
		{
			if (*(dist + i) != 0)
			{
				if (abs(*(dist + i) - *(dist + i - invalidPoint - 1))>30)
				{
					Pcluster.push_back(Pclm);
					Pclm.clear();
					Pclm.push_back(i);
					invalidPoint = 0;
					continue;
				}
				Pclm.push_back(i);
				invalidPoint = 0;
				continue;
			}
			invalidPoint++;
		}
		if (Pclm.size() != 0)Pcluster.push_back(Pclm);
		double mindist = 0;
		int selectindex;
		for (unsigned int i = 0; i < Pcluster.size(); i++)
		{
			if (Pcluster[i].size() < 4)
			{
				*(indices + ((*isize)++)) = Pcluster[i][0];
				continue;
			}
			mindist = *(dist + Pcluster[i][0]);
			selectindex = Pcluster[i][0];
			for (unsigned int j = 1; j < Pcluster[i].size(); j++)
			{
				if (*(dist + Pcluster[i][j]) < mindist)
				{
					mindist = *(dist + Pcluster[i][j]);
					selectindex = Pcluster[i][j];
				}
			}
			*(indices + ((*isize)++)) = selectindex;
		}
	}
	return;
}