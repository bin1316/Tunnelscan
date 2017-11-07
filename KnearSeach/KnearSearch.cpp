// KnearSeach.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include <iostream>
#include <flann/flann.h>
#include "KnearSearch.h"

using namespace std;

void getflannindex(float* cloud, int size)
{
	flann::Index<flann::L2_Simple<float>> flannindex_(flann::Matrix<float>(cloud, size, 3), flann::KDTreeSingleIndexParams(15));
	flannindex_.buildIndex();
	flannindex = flannindex_;
	return;
}
void KNS(float* SP, int k, int* indices, float* dist)
{
	flann::Matrix<int> indices_mat(indices, k, 1);
	flann::Matrix<float> dist_mat(dist, k, 1);
	flannindex.knnSearch(flann::Matrix<float>(SP, 1, 3), indices_mat, dist_mat, k, flann::SearchParams(-1, 0.0f));
	return;
}