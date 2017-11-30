#include <pcl/filters/filter_indices.h>
#include <pcl/search/pcl_search.h>
#include <iostream>
namespace pcl
{
	template<typename PointT>
	class SamePointRemove : public FilterIndices<PointT>
	{
	protected:
		typedef typename FilterIndices<PointT>::PointCloud PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

	public:
		enum IndiceState
		{
			CHECKED = -1,
			UNCHECKED = 0
		};
		/** \brief Constructor.
		  * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
		  */
		SamePointRemove(bool extract_removed_indices = false) :
			FilterIndices<PointT>::FilterIndices(extract_removed_indices),
			searcher_(),
			mean_k_(1),
			std_mul_(0.0)
		{
				filter_name_ = "SamePointRemove";
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

		/** \brief The number of points to use for mean distance estimation. */
		int mean_k_;

		/** \brief Standard deviations threshold (i.e., points outside of
		  * \f$ \mu \pm \sigma \cdot std\_mul \f$ will be marked as outliers). */
		double std_mul_;

		std::vector<int> pointstate;
	};
}

template <typename PointT> void
pcl::SamePointRemove<PointT>::applyFilter(PointCloud &output)
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
pcl::SamePointRemove<PointT>::applyFilterIndices(std::vector<int> &indices)
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
	pointstate.resize(indices_->size(), UNCHECKED);

	int RemaineddPointNum = 0;
	int findnum;
	bool RP_isTrue;
	// First pass: Compute the mean distances for all points with respect to their k nearest neighbors
	for (int iii = 0; iii < indices_->size(); iii++)  // iii = input indices iterator
	{
		if (pointstate[iii] != UNCHECKED)continue;
		// Perform the nearest k search
		findnum = searcher_->radiusSearch((*indices_)[iii], 1, nn_indices, nn_dists,mean_k_);
		while(findnum==mean_k_)
		{
			findnum *= 2;
			findnum = searcher_->radiusSearch((*indices_)[iii], 0, nn_indices, nn_dists, findnum);

		}
		// Calculate the mean distance to its neighbors
		for (int k = 0; k < findnum; ++k)// k = 0 is the query point
		{
			pointstate[nn_indices[k]] = CHECKED;
		}
		indices[RemaineddPointNum]=iii;
		RemaineddPointNum++;
	}
	indices.resize(RemaineddPointNum);
	cout << "After filted:" << RemaineddPointNum << endl;
}