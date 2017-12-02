#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include<Eigen/Eigenvalues>

namespace pcl{
	template<typename PointT>
	class RoughRegistration{
	public:
		RoughRegistration(){

		};
		typedef typename pcl::PointCloud<PointT> PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename pcl::PointCloud<pcl::Normal>::Ptr PointNormalPtr;

		inline void setInputModel(PointCloudPtr input_model_, PointCloudPtr model_track1_, PointCloudPtr model_track2_){
			input_model = input_model_;
			model_track1 = model_track1_;
			model_track2 = model_track2_;
		}
		inline void setInputCloud(PointCloudPtr input_cloud_, PointCloudPtr cloud_track1_, PointCloudPtr cloud_track2_, PointNormalPtr cloud_normal_){
			input_cloud = input_cloud_;
			cloud_track1 = cloud_track1_;
			cloud_track2 = cloud_track2_;
			cloud_normal = cloud_normal_;
		}


		void applyRoughRegistration();

	private:
		PointCloudConstPtr input_model, model_track1, model_track2;
		PointCloudPtr  input_cloud, cloud_track1, cloud_track2;
		PointNormalPtr cloud_normal;

		void adjustMatrix(Eigen::Matrix3f &fea_lambda, Eigen::Matrix3f &fea_vector);
	};

	template <typename PointT> void
		pcl::RoughRegistration<PointT>::adjustMatrix(Eigen::Matrix3f &fea_lambda, Eigen::Matrix3f &fea_vector){
			//第一列向前，第三竖直
			if (fea_lambda(0, 0) < fea_lambda(1, 1)){
				swap(fea_vector(0, 0), fea_vector(0, 1));
				swap(fea_vector(1, 0), fea_vector(1, 1));
				swap(fea_vector(2, 0), fea_vector(2, 1));
				swap(fea_lambda(0, 0), fea_lambda(1, 1));
			}
			if (fea_lambda(1, 1) < fea_lambda(2, 2)){
				swap(fea_vector(0, 2), fea_vector(0, 1));
				swap(fea_vector(1, 2), fea_vector(1, 1));
				swap(fea_vector(2, 2), fea_vector(2, 1));
				swap(fea_lambda(1, 1), fea_lambda(2, 2));
			}
			if (fea_lambda(0, 0) < fea_lambda(1, 1)){
				swap(fea_vector(0, 0), fea_vector(0, 1));
				swap(fea_vector(1, 0), fea_vector(1, 1));
				swap(fea_vector(2, 0), fea_vector(2, 1));
				swap(fea_lambda(0, 0), fea_lambda(1, 1));
			}
			//下面随便调都可以，只要法向量的方向都对应就好
			if (fea_vector(0, 0) > 0){
				fea_vector(0, 0) = -fea_vector(0, 0);
				fea_vector(1, 0) = -fea_vector(1, 0);
				fea_vector(2, 0) = -fea_vector(2, 0);
			}
			if (fea_vector(1, 1) < 0){
				fea_vector(0, 1) = -fea_vector(0, 1);
				fea_vector(1, 1) = -fea_vector(1, 1);
				fea_vector(2, 1) = -fea_vector(2, 1);
			}
			if (fea_vector(2, 2) < 0){
				fea_vector(0, 2) = -fea_vector(0, 2);
				fea_vector(1, 2) = -fea_vector(1, 2);
				fea_vector(2, 2) = -fea_vector(2, 2);
			}
		}

	template <typename PointT> void
		pcl::RoughRegistration<PointT>::applyRoughRegistration(){
			Eigen::Matrix3f cov_model_track, cov_cloud_track;
			Eigen::Vector4f cen_model_track1, cen_model_track2, cen_cloud_track1, cen_cloud_track2;
			pcl::compute3DCentroid(*model_track1, cen_model_track1);
			pcl::compute3DCentroid(*model_track2, cen_model_track2);
			pcl::compute3DCentroid(*cloud_track1, cen_cloud_track1);
			pcl::compute3DCentroid(*cloud_track2, cen_cloud_track2);
			if (cen_model_track1(1) > cen_model_track2(1)){
				pcl::computeMeanAndCovarianceMatrix(*model_track1, cov_model_track, cen_model_track1);
			}
			else{
				pcl::computeMeanAndCovarianceMatrix(*model_track2, cov_model_track, cen_model_track1);
			}
			if (cen_cloud_track1(1) > cen_cloud_track2(1)){
				pcl::computeMeanAndCovarianceMatrix(*cloud_track1, cov_cloud_track, cen_cloud_track1);
			}
			else{
				pcl::computeMeanAndCovarianceMatrix(*cloud_track2, cov_cloud_track, cen_cloud_track1);
			}
			Eigen::EigenSolver<Eigen::Matrix3f> sles_m(cov_model_track);
			Eigen::Matrix3f sla_m = sles_m.pseudoEigenvalueMatrix();
			Eigen::Matrix3f slb_m = sles_m.pseudoEigenvectors();
			Eigen::EigenSolver<Eigen::Matrix3f> sles_c(cov_cloud_track);
			Eigen::Matrix3f sla_c = sles_c.pseudoEigenvalueMatrix();
			Eigen::Matrix3f slb_c = sles_c.pseudoEigenvectors();
			adjustMatrix(sla_m, slb_m);
			adjustMatrix(sla_c, slb_c);
			Eigen::Matrix3f R = slb_m*slb_c.transpose();//修改
			Eigen::Vector3f cen_cloud_track1_, cen_model_track1_;
			cen_cloud_track1_(0) = cen_cloud_track1(0), cen_cloud_track1_(1) = cen_cloud_track1(1), cen_cloud_track1_(2) = cen_cloud_track1(2);
			cen_model_track1_(0) = cen_model_track1(0), cen_model_track1_(1) = cen_model_track1(1), cen_model_track1_(2) = cen_model_track1(2);
			Eigen::Vector3f T = cen_model_track1_ - R*cen_cloud_track1_;
			Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
			transform_matrix(0, 0) = R(0, 0), transform_matrix(0, 1) = R(0, 1), transform_matrix(0, 2) = R(0, 2), transform_matrix(0, 3) = T(0);
			transform_matrix(1, 0) = R(1, 0), transform_matrix(1, 1) = R(1, 1), transform_matrix(1, 2) = R(1, 2), transform_matrix(1, 3) = T(1);
			transform_matrix(2, 0) = R(2, 0), transform_matrix(2, 1) = R(2, 1), transform_matrix(2, 2) = R(2, 2), transform_matrix(2, 3) = T(2);
			transformPointCloud(*input_cloud, *input_cloud, transform_matrix);
			for (size_t i = 0; i < cloud_normal->points.size(); ++i){
				//cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
				Eigen::Matrix<float, 3, 1> pt(cloud_normal->points[i].normal_x, cloud_normal->points[i].normal_y, cloud_normal->points[i].normal_z);
				cloud_normal->points[i].normal_x = static_cast<float> (transform_matrix(0, 0) * pt.coeffRef(0) + transform_matrix(0, 1) * pt.coeffRef(1) + transform_matrix(0, 2) * pt.coeffRef(2));
				cloud_normal->points[i].normal_y = static_cast<float> (transform_matrix(1, 0) * pt.coeffRef(0) + transform_matrix(1, 1) * pt.coeffRef(1) + transform_matrix(1, 2) * pt.coeffRef(2));
				cloud_normal->points[i].normal_z = static_cast<float> (transform_matrix(2, 0) * pt.coeffRef(0) + transform_matrix(2, 1) * pt.coeffRef(1) + transform_matrix(2, 2) * pt.coeffRef(2));
			}
		}
}
