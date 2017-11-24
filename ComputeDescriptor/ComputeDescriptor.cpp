// ComputeDescriptor.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/centroid.h>
#include<Eigen/Eigenvalues>
#include <iostream>

namespace pcl{
	template<typename PointT>
	class Descriptor{
		Descriptor(){};
	public:
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

		inline double setStddevMulThresh(double stddev_mult){
			std_mul_=stddev_mult
		}
		inline void setMeanK(int nr_k){
			mean_k_ = nr_k;
		}
		inline void setInputCloud(PointCloud &model, PointCloud &cloud){
			input_model = model;
			input_cloud = cloud;
		}
		inline void setInputKey(std::vector<int> &model, std::vector<int> &cloud){
			key_model = model;
			key_cloud = cloud;
		}
		inline void setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr model, pcl::PointCloud<pcl::Normal>::Ptr cloud){
			normal_model = model;
			normal_cloud = cloud;
		}
		void compute();

	private:
		PointCloud input_model,input_cloud;
		SearcherPtr searcher_;
		pcl::PointCloud<pcl::Normal>::Ptr normal_model, normal_cloud;
		std::vector<int> key_model, key_cloud;
		int mean_k_, std_mul_;
	};
}
int _tmain(int argc, _TCHAR* argv[])
{
	
	return 0;
}

