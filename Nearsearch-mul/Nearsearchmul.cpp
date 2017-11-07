/*
 * Nearsearch-mul.cpp
 *
 *  Created on: 2017Äê5ÔÂ3ÈÕ
 *      Author: BIN
 */
#include "stdafx.h"
#include<flann/flann.h>
#include"Nearsearchmul.h"

void getflannindexmul(float* cloud, int size)
{
	flannindex=new flann::Index<flann::L2_Simple<float>/* */>(flann::Matrix<float>(cloud, size, 3), flann::KDTreeSingleIndexParams(15));
	flannindex->buildIndex();
	std::cout<<"built done!"<<std::endl;
	return;
}
int KNSmul(coordinate* SP, int conum,int k, int* indices, float* dist,float threshold)
{
	int outputnum=0;
	float s[3];
	flann::Matrix<int> indices_mat(indices, k, 1);
	flann::Matrix<float> dist_mat(dist, k, 1);
	for(int i=0;i<conum;i++)
	{
		s[0]=SP[i].x;
		s[1]=SP[i].y;
		s[2]=SP[i].z;
		flannindex->knnSearch(flann::Matrix<float>(s, 1, 3), indices_mat, dist_mat, k, flann::SearchParams(-1, 0.0f));
		if (indices[0]>threshold)
		{
			SP[outputnum].x = SP[i].x;
			SP[outputnum].y = SP[i].y;
			SP[outputnum++].z = SP[i].z;
		}
			
	}
	return outputnum;
}
void deleteptrmul()
{
	if(!flannindex)delete flannindex;
}


