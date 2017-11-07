// NearSearch2D.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include <iostream>
#include <flann/flann.h>
#include "Nearsearch2D.h"
#include <math.h>
using namespace std;

void getflannindex2d(float* cloud1, int size1, float* cloud2, int size2)
{
	cloud2d1 = new float[2 * size1];
	cloud2d2 = new float[2 * size2];
	for (int i = 0; i < size1; i++)
	{
		cloud2d1[2 * i] = cloud1[3 * i];
		cloud2d1[2 * i + 1] = cloud1[3 * i + 1];
	}
	for (int i = 0; i < size2; i++)
	{
		cloud2d2[2 * i] = cloud2[3 * i];
		cloud2d2[2 * i + 1] = cloud2[3 * i + 1];
	}
	flannindex2d1 = new flann::Index<flann::L2_Simple<float>/* */>(flann::Matrix<float>(cloud2d1, size1, 2), flann::KDTreeSingleIndexParams(15));
	flannindex2d1->buildIndex();
	flannindex2d2 = new flann::Index<flann::L2_Simple<float>/* */>(flann::Matrix<float>(cloud2d2, size2, 2), flann::KDTreeSingleIndexParams(15));
	flannindex2d2->buildIndex();
	return;
}
int KNS2d(float* SP, int k, int* indices1, float* dist1, int* indices2, float* dist2)
{
	float gap = 1400;
	float maxdist = 1560;
	flann::Matrix<int> indices_mat1(indices1, k, 1);
	flann::Matrix<float> dist_mat1(dist1, k, 1);
	flannindex2d1->knnSearch(flann::Matrix<float>(SP, 1, 2), indices_mat1, dist_mat1, k, flann::SearchParams(-1, 0.0f));
	flann::Matrix<int> indices_mat2(indices2, k, 1);
	flann::Matrix<float> dist_mat2(dist2, k, 1);
	flannindex2d2->knnSearch(flann::Matrix<float>(SP, 1, 2), indices_mat2, dist_mat2, k, flann::SearchParams(-1, 0.0f));
	dist1[0] = sqrt(dist1[0]);
	dist2[0] = sqrt(dist2[0]);
	if (max(dist1[0], dist2[0]) - gap / 2 > maxdist)
		return -1;
	return max(dist1[0], dist2[0]) - gap / 2;
}
void deleteptr()
{
	if (!flannindex2d1)delete flannindex2d1;
	if (!cloud2d1)delete cloud2d1;
	if (!flannindex2d2)delete flannindex2d2;
	if (!cloud2d2)delete cloud2d2;
}
