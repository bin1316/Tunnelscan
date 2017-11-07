// TrackExtract.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <Eigen/src/Eigenvalues/EigenSolver.h>

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("../PCDData/0502/plane112.pcd", *cloud);
	Eigen::Matrix3f covariance_matrix;
	Eigen::Vector4f centroid;
	computeMeanAndCovarianceMatrix(*cloud, covariance_matrix, centroid);
	Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
	Eigen::Matrix3f A = es.pseudoEigenvalueMatrix();
	Eigen::Matrix3f B = es.pseudoEigenvectors();
	float max = A(0, 0), min = max, sec;
	int maxid = 0, minid = 0, secid = -1;
	for (int i = 1; i < 3; i++)
	{
		if (A(i, i)>max)
		{
			max = A(i, i);
			maxid = i;
		}
		if (A(i, i) < min)
		{
			min = A(i, i);
			minid = i;
		}
	}
	for (int i = 0; i<3; i++)
	{
		if (A(i, i) != min&&A(i, i) != max)
		{
			sec = A(i, i);
			secid = i;
		}
	}
	if (min>1200)
	{
		cout << "Not plane！" << endl;
		return 0;
	}
	if (secid != -1)
	{
		if (sec / max < 0.1)
		{
			cout << "this is a plane" << endl;
		}
		else
		{
			cout << "Not a rectangle" << endl;
			return 0;
		}
	}
	Eigen::Vector3f direction = B.col(maxid);
	cout << B.col(maxid) << endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ SearchPoint = { centroid(0), centroid(1), centroid(2) };
	//pcl::PointXYZ SearchPoint = { 1, 1, 1 };

	int searchnum = 1000;
	vector<int> indices(searchnum);
	vector<float> dist(searchnum);
	kdtree.nearestKSearch(SearchPoint, 1, indices, dist);
	SearchPoint = cloud->points[indices[0]];
	int nearnum = kdtree.radiusSearch(SearchPoint, 200, indices, dist);
	while (nearnum == searchnum&&nearnum < 10000)
	{
		searchnum = 2 * searchnum;
		nearnum = kdtree.radiusSearch(SearchPoint, 200, indices, dist);
	}
	vector<float> Ndist, Pdist;
	for (int i = 1; i < nearnum; i++)
	{
		Eigen::Vector3f p = { cloud->points[indices[i]].x - SearchPoint.x, cloud->points[indices[i]].y - SearchPoint.y, cloud->points[indices[i]].z - SearchPoint.z };
		float distdot = p.dot(direction);
		if (direction.cross(p).dot(B.col(minid))>0)
		{
			Pdist.push_back(dist[i] - distdot*distdot);
		}
		else
		{
			Ndist.push_back(dist[i] - distdot*distdot);
		}
	}
	float Pmax = 0, Nmax = 0;
	for (int i = 0; i < Pdist.size(); i++)
	{
		if (Pdist[i]>Pmax)
			Pmax = Pdist[i];
	}
	for (int i = 0; i < Ndist.size(); i++)
	{
		if (Ndist[i]>Nmax)
			Nmax = Ndist[i];
	}
	cout << "Plane witdh is " << sqrt(Pmax) + sqrt(Nmax) << endl;
	return 0;
}

