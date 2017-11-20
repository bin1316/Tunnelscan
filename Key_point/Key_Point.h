
#include <pcl/filters/filter_indices.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/centroid.h>
#include<Eigen/Eigenvalues>
#include <iostream>
using namespace std;
namespace pcl
{
	template<typename PointT>
	class KeyPoint : public FilterIndices<PointT>
	{
	protected:
		typedef typename FilterIndices<PointT>::PointCloud PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

	public:
		/** \brief Constructor.
		* \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
		*/
		KeyPoint(bool extract_removed_indices = false) :
			FilterIndices<PointT>::FilterIndices(extract_removed_indices),
			searcher_(),
			mean_k_(1),
			std_mul_(0.0)
		{
				filter_name_ = "KeyPoint";
			}

		/** \brief Set the number of distance to use for key point estimation.
		* \param[in] threshold The number of distance to use for key point estimation. distance for point to plane
		*/
		inline void setmindis(double mindis){
			mindis_ = mindis;
		}

		/** \brief Set the number of minimum lambda to use for key point estimation.
		* \param[in] threshold The number of minimum lambda to use for key point estimation. distance for point to plane
		*/
		inline void setMinlambda(double minlambda){
			minlambda_ = minlambda;
		}

		/** \brief Set the number of nearest neighbors to use for mean distance estimation.
		* \param[in] nr_k The number of points to use for mean distance estimation.
		*/
		inline void
			setMeanK(int nr_k)
		{
				mean_k_ = nr_k;
			}

		/** \brief Get the number of nearest neighbors to use for mean distance estimation.
		* \return The number of points to use for mean distance estimation.
		*/
		inline int
			getMeanK()
		{
				return (mean_k_);
			}

		/** \brief Set the standard deviation multiplier for the distance threshold calculation.
		* \details The distance threshold will be equal to: mean + stddev_mult * stddev.
		* Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
		* \param[in] stddev_mult The standard deviation multiplier.
		*/
		inline void
			setStddevMulThresh(double stddev_mult)
		{
				std_mul_ = stddev_mult;
			}

		/** \brief Set the Normal of input pointcloud.
		* \param[in] normal the Normal of input_.
		*/
		inline void setNormal(pcl::PointCloud<pcl::Normal>::Ptr normal){
			normal_ = normal;
		}

		/** \brief Get the standard deviation multiplier for the distance threshold calculation.
		* \details The distance threshold will be equal to: mean + stddev_mult * stddev.
		* Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
		* \param[in] stddev_mult The standard deviation multiplier.
		*/
		inline double
			getStddevMulThresh()
		{
				return (std_mul_);
			}

	protected:
		using PCLBase<PointT>::input_;
		using PCLBase<PointT>::indices_;
		using Filter<PointT>::filter_name_;
		using Filter<PointT>::getClassName;
		using FilterIndices<PointT>::negative_;
		using FilterIndices<PointT>::keep_organized_;
		using FilterIndices<PointT>::user_filter_value_;
		using FilterIndices<PointT>::extract_removed_indices_;
		using FilterIndices<PointT>::removed_indices_;

		/** \brief Filtered results are stored in a separate point cloud.
		* \param[out] output The resultant point cloud.
		*/
		void
			applyFilter(PointCloud &output);

		/** \brief Filtered results are indexed by an indices array.
		* \param[out] indices The resultant indices.
		*/
		void
			applyFilter(std::vector<int> &indices)
		{
				applyFilterIndices(indices);
			}

		/** \brief Filtered results are indexed by an indices array.
		* \param[out] indices The resultant indices.
		*/
		void
			applyFilterIndices(std::vector<int> &indices);

	private:
		/** \brief A pointer to the spatial search object. */
		SearcherPtr searcher_;

		/** \Normal of the input_. */
		pcl::PointCloud<pcl::Normal>::Ptr normal_;

		/** \brief The number of points to use for mean distance estimation. */
		int mean_k_;

		double mindis_;

		double minlambda_;

		/** \brief Standard deviations threshold (i.e., points outside of
		* \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as outliers). */
		double std_mul_;
	};
}

template <typename PointT> void
pcl::KeyPoint<PointT>::applyFilter(PointCloud &output)
{
	std::vector<int> indices;
	if (keep_organized_)
	{
		bool temp = extract_removed_indices_;
		extract_removed_indices_ = true;
		applyFilterIndices(indices);
		extract_removed_indices_ = temp;

		output = *input_;
		for (int rii = 0; rii < static_cast<int> (removed_indices_->size()); ++rii)  // rii = removed indices iterator
			output.points[(*removed_indices_)[rii]].x = output.points[(*removed_indices_)[rii]].y = output.points[(*removed_indices_)[rii]].z = user_filter_value_;
		if (!pcl_isfinite(user_filter_value_))
			output.is_dense = false;
	}
	else
	{
		applyFilterIndices(indices);
		copyPointCloud(*input_, indices, output);
	}
}

template <typename PointT> void
pcl::KeyPoint<PointT>::applyFilterIndices(std::vector<int> &indices)
{
	// Initialize the search class
	if (!searcher_)
	{
		if (input_->isOrganized())
			searcher_.reset(new pcl::search::OrganizedNeighbor<PointT>());
		else
			searcher_.reset(new pcl::search::KdTree<PointT>(false));
	}
	searcher_->setInputCloud(input_);
	// The arrays to be used
	std::vector<int> nn_indices(mean_k_);
	std::vector<float> nn_dists(mean_k_);
	std::vector<float> distances(indices_->size());
	removed_indices_->resize(indices_->size());
	int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

	//copy  to indice
	indices.resize(indices_->size());

	int RemaineddPointNum = 0;
	int findnum, searchnum;
	//init vector true for remained  and false for oppoisite
	std::vector<bool> NotVisited(indices_->size(), true);
	std::vector<int> isRemained;
	// First pass: Compute the mean distances for all points with respect to their k nearest neighbors
	for (int iii = 0; iii < indices_->size(); iii++)  // iii = input indices iterator
	{
		if (NotVisited[iii])
		{
			if (!pcl_isfinite(input_->points[(*indices_)[iii]].x) ||
				!pcl_isfinite(input_->points[(*indices_)[iii]].y) ||
				!pcl_isfinite(input_->points[(*indices_)[iii]].z))
			{
				continue;
			}
			searchnum = mean_k_;
			findnum = searcher_->radiusSearch((*indices_)[iii], std_mul_, nn_indices, nn_dists, searchnum);

			// Calculate the  distance to its neighbors' plane
			if (findnum != searchnum)
				nn_indices.resize(findnum);
			Eigen::Matrix3f cov;
			Eigen::Vector4f cent;
			computeMeanAndCovarianceMatrix(*input_, nn_indices, cov, cent);
			Eigen::EigenSolver<Eigen::Matrix3f> sles(cov);
			Eigen::Matrix3f sla = sles.pseudoEigenvalueMatrix();
			Eigen::Matrix3f slb = sles.pseudoEigenvectors();
			double minlambda = sla(0, 0);
			int minvectorIndex = 0;//最小特征值，与其对应的位置
			if (minlambda > sla(1, 1)){
				minlambda = sla(1, 1);
				minvectorIndex = 1;
			}
			if (minlambda > sla(2, 2)){
				minlambda = sla(2, 2);
				minvectorIndex = 2;
			}
			Eigen::Vector3f normal, i2cent;//拟合平面法向量，平面中心到点的向量
			normal(0) = slb(0, minvectorIndex), normal(1) = slb(1, minvectorIndex), normal(2) = slb(2, minvectorIndex);
			i2cent(0) = cent(0) - input_->points[iii].x, i2cent(1) = cent(1) - input_->points[iii].y, i2cent(2) = cent(2) - input_->points[iii].z;
			double dis_ = abs(normal.dot(i2cent));//点乘求点到平面的距离
			if (dis_ > mindis_)//距离大于阈值定为keypoint
				isRemained.push_back(iii);
			else if (minlambda < minlambda_){//小于阈值并且特征值小于阈值，剪枝
				for (int j = 0; j < findnum; j++){
					if (abs(normal(0)*normal_->points[nn_indices[j]].normal_x+
						normal(1)*normal_->points[nn_indices[j]].normal_y +
						normal(2)*normal_->points[nn_indices[j]].normal_z)>0.8)
						NotVisited[nn_indices[j]] = false;
				}
			}
			nn_indices.resize(mean_k_);
		}
	}
	indices = isRemained;
	cout << "Key Point Number :" << indices.size() << endl;
}