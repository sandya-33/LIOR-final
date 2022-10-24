
#include "LIOR.h"

#include <omp.h>
namespace pcl {

    template <typename PointT>
    void LIOR<PointT>::applyFilter(PointCloud& output) {
        (kdTree_).setInputCloud(input_);

        if (detection_range < 71.235) {
            PCL_ERROR("[pcl::DynamicRadiusOutlierRemoval::applyFilter] need a minimum snow detection range.\n");
            //return;
        }
        /*if (angle_resol_ <= 0.0) {
            PCL_ERROR("[pcl::DynamicRadiusOutlierRemoval::applyFilter] need a Angular Resolution value before continuing.\n");
            //break;
        }*/
        if (min_nbrs_ < 3) {
            PCL_ERROR("[pcl::DynamicRadiusOutlierRemoval::applyFilter] need a Minimum Neighbours value before continuing.\n");
            //break;
        }

        std::vector<int> k_indices;
        std::vector<float> k_distances;
        int k = 0;
        int Ithr;


        //Vector of boolean to check if we already visited certain point
        std::vector<bool> is_visited(input_->width * input_->height, false);

        //Vector of boolean for marking safe points
        std::vector<bool> is_outlier(input_->width * input_->height, false);
        for (std::size_t pid = 0; pid < input_->size(); ++pid) {

            k_indices.clear();
            k_distances.clear();
            double dp = computeDistance((*input_)[pid]);

            if(dp<detection_range)
            {
                Ithr=20.f;
            }
            else {
                Ithr=0.f;
            }

            if((*input_)[pid].intensity > Ithr) {
                is_outlier[pid] = false;
            }
            else{
                is_outlier[pid] = true;
                int nbrs = kdTree_.radiusSearch((*input_)[pid], 0.1, k_indices, k_distances);

                if (nbrs > min_nbrs_) {
                    is_outlier[pid]=false;
                }
            }
        }
            

        
        for (int s = 0; s < is_outlier.size(); s++) {
            if (!is_outlier[s]) {
                (output).push_back((*input_)[s]);
            }
        }
        
    }
}
