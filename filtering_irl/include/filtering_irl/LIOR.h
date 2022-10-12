#pragma once

#include <cmath>
#include <vector>
#include <omp.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/organized.h>

namespace pcl {
	template<typename PointT>
	class LIOR : public Filter<PointT> {
		using Filter<PointT>::input_;
		using Filter<PointT>::indices_;
		typedef typename Filter<PointT>::PointCloud PointCloud;
		typedef typename pcl::KdTreeFLANN<PointT> KdTree;

	private:
		/** \brief Angular resolution of LiDAR sensor, found in spec sheets or technical details */
		//double angle_resol_;
		/** \brief Constant multiplier for the calculated raduis */
		double detection_range;
		/** \brief Minimum number of points to be present in the search radius for not be an outlier */
		int min_nbrs_;
		/** \brief A pointer to the spatial search object, KdTreeFLANN for fast search methods. */
		KdTree kdTree_;
		int private_variable;
        /** \brief Radius multiplier for the calculated raduis */
		//double multi_const_ = tan(angle_resol_) * multi_;

		/** \brief Calculates radius depending on the location of search point.
		* Uses X, Y and Z coordinate to calculate dristance from the origin
		* and then multiplies to the radius multiplier to increase the search radius accordingly.
		* \param[in] Index of Search point
		*/
		/*inline double computeRadius(auto point) {
			return(sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z))) * tan(angle_resol_) * multi_;
		}*/
        inline double computeDistance(auto point){
			return (sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z)));
        }

	public:
		/** \brief Constructor.
		 * Sets \ref angle_resol_ to 0 and \ref multi_ to
		 * Sets angle_resol_ to 0 and \ref multi_ to MAXDBL
		 */
		LIOR() {
			//angle_resol_ = 0.0;
			min_nbrs_ = 1;
			detection_range = 70.235;
			private_variable=-1;
			//multi_const_ = multi_;
		}

		/** \brief Set the angular resolution of the LiDAR sensor, available in Spec Sheet
		* \param[in] Angular resolution in degrees
		*/
		/*inline void setAngularResolution(double angle_resolution) {
			angle_resol_ = (angle_resolution * M_PI) / 180.0;
		}*/

		/** \brief Get the angulular resolution in degrees as set by the user. */
		/*	inline double getAngularResolution() const {
			return(angle_resol_);
		}*/

		/** \brief Set the minimum number of points in the search radius
		* \param[in] Minimum number of points in search radius
		*/
	    inline void setPrivateVariable(int private_var) {
			private_variable = private_var;
		}

		/** \brief Get the private variable value as set by the user. */\
			inline int getPrivateVariable() const {
			return(private_variable);
			}
		
		inline void setMinNeighborsInRadius(double min_nbrs) {
			min_nbrs_ = min_nbrs;
		}

		/** \brief Get the Constant Multiplier for calculated radius as set by the user. */
		inline int getMinNeighborsInRadius() const {
			return(min_nbrs_);
		}

		/** \brief Set the Constant Multiplier for calculated radius.
		* \param[in] Constant Multiplier for the search radius
		*/
		inline void setDetection_range(double drange) {
			detection_range = drange;
		}

		/** \brief Get the Constant Multiplier for calculated radius as set by the user. */
		inline double getDetection_range() const {
			return(detection_range);
		}

		/** \brief Filter the input data and store the results into output
		 * \param[out] output the resultant point cloud message
		 */


		/** \brief Provide a pointer to the search object.
		 * \param[in] tree a pointer to the spatial search object.
		 */
		void setSearchMethod(const KdTree& kdtree) {
			kdTree_ = kdtree;
		}
		bool inline preCheck() {

			if (private_variable == -1) {
				PCL_ERROR("[pcl::CustomFilter] Private Variable cannot be -1.\n");
				return false;
			}

			return true;
		}
		void applyFilter(PointCloud& output);
	};
}