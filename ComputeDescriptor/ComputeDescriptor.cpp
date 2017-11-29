// ComputeDescriptor.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/centroid.h>

#include <flann/flann.h>

#include <pcl/io/pcd_io.h>
#include <PCL_io.h>

#include<Eigen/Eigenvalues>
#include <vector>
#include <map>
#include<math.h>
#include <time.h>

#include <vld.h>
#pragma comment(lib,"vld.lib")

#include <iostream>
using namespace std;

namespace flann
{
	template <typename T> struct L2_Simple;
	template <typename T> class Index;
}
namespace pcl{
	template<typename PointT,typename Dist = ::flann::L2_Simple<float>>
	class Descriptor{
		
	public:
		Descriptor(){
			des_dim_ = 54;
		};
		typedef typename pcl::PointCloud<PointT> PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;
		typedef ::flann::Index<Dist> FLANNIndex;

		inline void setStddevMulThresh(double stddev_mult){
			std_mul_ = stddev_mult;
		}
		inline void setMeanK(int nr_k){
			mean_k_ = nr_k;
		}
		inline void setInputCloud(const PointCloudConstPtr &model, const PointCloudConstPtr &cloud){
			input_model = model;
			input_cloud = cloud;
			// Initialize the search class
			if (!searcher_)
			{
				if (input_model->isOrganized())
					searcher_.reset(new pcl::search::OrganizedNeighbor<PointT>());
				else
					searcher_.reset(new pcl::search::KdTree<PointT>(false));
			}
			searcher_->setInputCloud(input_model);
		}
		inline void setInputKey(std::vector<int> &model, std::vector<int> &cloud){
			key_model = model;
			key_cloud = cloud;
		}
		inline void setInputNormal(pcl::PointCloud<pcl::Normal>::Ptr model, pcl::PointCloud<pcl::Normal>::Ptr cloud){
			normal_model = model;
			normal_cloud = cloud;
		}
		std::vector<float> convFeatureMatrix(std::vector<Eigen::Matrix4i> &FeatureMatrix);

		void addvector(std::vector<float> &sum, std::vector<float> &vc);

		void dividevector(std::vector<float> &sum, int num);

		void computeModelDescriptor();

		void buildModelIndex();

	private:
		int des_dim_;//算子的维度;
		PointCloudConstPtr input_model, input_cloud;
		SearcherPtr searcher_;
		pcl::PointCloud<pcl::Normal>::Ptr normal_model, normal_cloud;
		std::vector<int> key_model, key_cloud;
		int mean_k_, std_mul_;
		map<int, std::vector<float>> mp_feature;//存储key_point的初始特征描述子
		std::vector<std::vector<float>> des;//存储key_point的合成特征描述子
		boost::shared_ptr<FLANNIndex> flann_index;
		boost::shared_array<float> des_;
	};

	template <typename PointT, typename Dist> std::vector<float>
		pcl::Descriptor<PointT, Dist>::convFeatureMatrix(std::vector<Eigen::Matrix4i> &FeatureMatrix){
			int t[8][3] = { { 0, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 }, { 0, 1, 1 }, { 1, 0, 0 }, { 1, 0, 1 }, { 1, 1, 0 }, { 1, 1, 1 } };
			float sum, allsum = 0;
			std::vector<float> res(27);
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++){
					for (int k = 0; k < 3; k++){
						sum = 0;
						for (int l = 0; l < 8; l++){
							sum += FeatureMatrix[i + t[l][0]](j + t[l][1], k + t[l][2]);
						}
						res[9 * i + 3 * j + k] = sum;
						allsum += sum;
					}
				}
			}
			allsum /= 27;
			for (int i = 0; i < 27; i++){
				res[i] = res[i] / allsum;
			}
			return res;
		}

		template <typename PointT, typename Dist> void
			pcl::Descriptor<PointT, Dist>::addvector(std::vector<float> &sum, std::vector<float> &vc){
			int size = sum.size();
			for (int i = 0; i < size; i++){
				sum[i] += vc[i];
			}
		}

		template <typename PointT, typename Dist> void
			pcl::Descriptor<PointT, Dist>::dividevector(std::vector<float> &sum, int num){
			int size = sum.size();
			for (int i = 0; i < size; i++){
				sum[i] /= num;
			}
		}

		template <typename PointT, typename Dist> void
			pcl::Descriptor<PointT, Dist>::computeModelDescriptor(){
			if (key_model.size() == 0)return;
			// The arrays to be used for knn
			std::vector<int> n_indices(mean_k_);
			std::vector<float> n_dists(mean_k_);
			int find_,tmp_;
			for (int l = 0; l < key_model.size();l++){//计算每个key_point的合成特征描述子
				tmp_ = key_model[l];
				find_ = searcher_->radiusSearch(tmp_, std_mul_, n_indices, n_dists, mean_k_);//合成特征描述子的计算域
				int tmp, findnum;
				// The arrays to be used for knn
				std::vector<int> nn_indices(mean_k_);
				std::vector<float> nn_dists(mean_k_);
				std::vector<float> ori_sum_descriptor(27, 0), ori_kp_descriptor, fea_vf;
				for (int i = 0; i < find_; i++){//计算每个key_point的初始特征描述子
					tmp = n_indices[i];
					map<int, std::vector<float>>::iterator it=mp_feature.find(tmp);
					if (it != mp_feature.end()){//计算过直接跳过
						if (n_dists[i] < 0.0001)
							ori_kp_descriptor = it->second;
						addvector(ori_sum_descriptor, it->second);
						continue;
					}
					findnum = searcher_->radiusSearch(tmp, std_mul_, nn_indices, nn_dists, mean_k_);//初始特征描述子的计算域
					std::vector<Eigen::Matrix4i> hist(4, Eigen::Matrix4i::Zero());//初始投影矩阵（4*4*4）
					for (int j = 0; j < findnum; j++){//每个邻近点计算各自的初始特征描述子（27维）
						if (nn_dists[j] < 0.001)continue;//自身
						float dz = input_model->points[nn_indices[j]].z - input_model->points[tmp].z,
							dx = input_model->points[nn_indices[j]].x - input_model->points[tmp].x,
							dy = input_model->points[nn_indices[j]].y - input_model->points[tmp].y,
							  angle = 0;
						//纵坐标不随旋转而改变
						int line=0,col=0;
						if (dz>0 && dz < 30)
							line = 1;
						else if (dz<0 && dz>-30)
							line = 2;
						else if (dz < 0 && dz < -30)
							line = 3;
						//横坐标随着旋转改变
						for (int k = 0; k < 4; k++){
							float ax = cos(angle), ay = sin(angle), inpro = ax*dx + ay*dy;
							if (inpro>0 && inpro < 30)
								col = 1;
							else if (inpro<0 && inpro>-30)
								col = 2;
							else if (inpro < 0 && inpro < -30)
								col = 3;
							hist[k](line,col)++;
							angle += 45*3.1415926/180;
						}
	
					}
					fea_vf = convFeatureMatrix(hist);
					mp_feature[tmp] = fea_vf;
					if (n_dists[i] < 0.0001)
						ori_kp_descriptor = fea_vf;
					addvector(ori_sum_descriptor, fea_vf);
				}
				//合成算子
				dividevector(ori_sum_descriptor, find_);
				for (int i = 0; i < 27; i++){
					ori_kp_descriptor.push_back(ori_sum_descriptor[i]);
				}
				des.push_back(ori_kp_descriptor);
			}
		}

		template <typename PointT, typename Dist> void
			pcl::Descriptor<PointT, Dist>::buildModelIndex(){
			int rows = key_model.size();
			des_.reset(new float[rows*des_dim_]);
			for (int i = 0; i < rows; i++){
				for (int j = 0; j < des_dim_; j++){
					*(des_.get() + i*des_dim_ + j) = des[i][j];
				}
			}
			//des.clear();//清空des，释放内存
			flann_index.reset(new FLANNIndex(::flann::Matrix<float>(des_.get(), rows, des_dim_), ::flann::KDTreeSingleIndexParams(15)));
			flann_index->buildIndex();
		}
}

int _tmain(int argc, _TCHAR* argv[])
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	MyloadpcdfileN(*model, "../../PCL_Data/111701/cloud_final.pcd");
	MyloadpcdfileN(*cloud, "../../PCL_Data/111702/cloud_final.pcd");
	std::vector<int> key;
	for (int i = 0; i < 10; i++){
		key.push_back(10 * i);
	}
	int t1, t2;
	t1 = clock();
	pcl::Descriptor<pcl::PointXYZ> de;
	de.setInputCloud(model, cloud);
	de.setInputKey(key, key);
	de.setMeanK(100);
	de.setStddevMulThresh(80);
	de.computeModelDescriptor();
	de.buildModelIndex();
	t2 = clock();
	cout << (float)(t2 - t1) / CLOCKS_PER_SEC << endl;
	return 0;
}

