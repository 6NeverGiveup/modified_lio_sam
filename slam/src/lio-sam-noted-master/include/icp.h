#include "utility.h"

class ICP
{
public:
    ICP()
    {
    }
    void setInputSource(const pcl::PointCloud<PointType>::Ptr &source)
    {
        source_ = source;
    }

    void setInputTarget(const pcl::PointCloud<PointType>::Ptr &target)
    {
        target_ = target;
    }

    void setTransformationEpsilon(double transformation_epsilon)
    {
        trans_epsilon_ = transformation_epsilon;
    }

    void setMaxCorrespondenceDistance_reg(float max_corr_dist)
    {
        max_corr_dist_reg_ = max_corr_dist;
    }
    void setMaxCorrespondenceDistance_score(float max_corr_dist)
    {
        max_corr_dist_score_ = max_corr_dist;
    }

    void setMaximumIterations(int max_iterations)
    {
        max_iterations_ = max_iterations;
    }

    bool hasConverged()
    {
        return is_converged_;
    }

    void align(pcl::PointCloud<PointType>::Ptr reg_cloud)
    {
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(max_corr_dist_reg_);
        icp.setMaximumIterations(max_iterations_);

        icp.setTransformationEpsilon(trans_epsilon_);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(source_);
        icp.setInputTarget(target_);

        icp.align(*reg_cloud);

        if (icp.hasConverged())
        {
            is_converged_ = true;
            final_trans_ = icp.getFinalTransformation();
            fitness_score_ = icp.getFitnessScore(max_corr_dist_score_);

            right_pairs_ = 0;
            std::vector<int> nn_indices(1);
            std::vector<float> nn_dists(1);
            auto target_tree = icp.getSearchMethodTarget();
            for (std::size_t i = 0; i < reg_cloud->points.size(); ++i)
            {
                // Find its nearest neighbor in the target
                target_tree->nearestKSearch(reg_cloud->points[i], 1, nn_indices, nn_dists);

                // Deal with occlusions (incomplete targets)
                if (nn_dists[0] <= max_corr_dist_score_)
                {
                    right_pairs_++;
                }
            }
        }
    }

    double getFitnessScore()
    {
        return fitness_score_;
    }

    int getRightPairs()
    {
        return right_pairs_;
    }

    Eigen::Matrix4f getFinalTransformation()
    {
        return final_trans_;
    }

    bool resultCanBeUsed(float max_score, float overlap_percent_required)
    {
        if (right_pairs_ >= source_->points.size() * overlap_percent_required && fitness_score_ < max_score)
        {
            return true;
        }
        return false;
    }

private:
    pcl::PointCloud<PointType>::Ptr source_;
    pcl::PointCloud<PointType>::Ptr target_;

    double trans_epsilon_;
    int max_iterations_ = 50;
    float max_corr_dist_reg_ = 20;
    float max_corr_dist_score_ = 0.5;

    Eigen::Matrix4f final_trans_ = Eigen::Matrix4f::Identity();
    bool is_converged_ = false;
    double fitness_score_ = -1.0;
    int right_pairs_ = 0;
};