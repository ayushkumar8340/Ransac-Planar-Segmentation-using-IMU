#include "ransac.h"

/// @brief setup ransac plane segmentation class
/// @param max_itr 
/// @param dist_th 
/// @return 
bool RansacPlane::setup(const int& max_itr,const double& dist_th)
{
    inliers_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    outliers_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    max_itrs_ = max_itr;
    distance_threshold_ = dist_th;

    inliers_->clear();
    outliers_->clear();

    normal_.clear();
    plane_coeff.clear();

    return true;
}

/// @brief reset ransac
/// @return 0/1
bool RansacPlane::reset()
{
    inliers_->clear();
    outliers_->clear();
    normal_.clear();
    plane_coeff.clear();

    return true;
}

/// @brief set normal of the plane (X Y Z)
/// @param normal 
/// @return 
bool RansacPlane::setPlaneNormal(RANSAC_PLANES normal)
{
    switch (normal)
    {
        case RANSAC_PLANES::X_NORMAL:
            normal_.push_back(1);
            normal_.push_back(0);
            normal_.push_back(0);
            break;

        case RANSAC_PLANES::Y_NORMAL:
            normal_.push_back(0);
            normal_.push_back(1);
            normal_.push_back(0);
            break;  

        case RANSAC_PLANES::Z_NORMAL:
            normal_.push_back(0);
            normal_.push_back(0);
            normal_.push_back(1);
            break;

        default:
            normal_.push_back(0);
            normal_.push_back(1);
            normal_.push_back(0);
            break;
    }

    return true;
}

/// @brief set normal of the plane 
/// @param normal input normal vector
/// @return 0/1
bool RansacPlane::setPlaneNormal(std::vector<double>& normal)
{
    normal_ = normal;
    return true;
}

/// @brief compute inliers from ransac
/// @param cloud input point cloud
/// @return 0/1
bool RansacPlane::computeInliers(const PointCloudPtr& cloud)
{
    std::unordered_set<int> inliers_idx;
    std::vector<int> tmp_inliers;

    // Plane parameters
    double a,b,c,d = 0.0;

    a = normal_[0];
    b = normal_[1];
    c = normal_[2];

    for(int i = 0; i < max_itrs_; i++)
    {
        int count = 0;
        std::unordered_set<int> idx;
        std::vector<int> tmp_id;

        pcl::PointXYZ initial_guess;
        int guess_idx;
        guess_idx = (rand() % cloud->points.size());
        idx.insert(guess_idx);

        initial_guess = cloud->points[guess_idx];
        d = -((a * initial_guess.x) + (b * initial_guess.y) + (c * initial_guess.z));

        for(int j = 0; j < cloud->points.size(); j++)
        {
            if(j == guess_idx)
            {
                continue;
            }

            double perp_dist = 0;
            perp_dist = ((fabs((a * cloud->points[j].x) + (b * cloud->points[j].y) + (c * cloud->points[j].z) + d)) / (sqrt(a * a + b * b + c * c)));
            if(perp_dist <= distance_threshold_)
            {
                idx.insert(j);
            }
        }
        if(idx.size() > inliers_idx.size())
        {
            inliers_idx.clear();
            inliers_idx = idx;
            tmp_inliers.clear();
            tmp_inliers = tmp_id;
        }
    }

    
    for(int i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZ p = cloud->points[i];
        
        if(inliers_idx.count(i))
        {
            inliers_->points.push_back(p);
        }
        else
        {
            outliers_->points.push_back(p);
        }
    }

    plane_coeff.push_back(a);
    plane_coeff.push_back(b);
    plane_coeff.push_back(c);
    plane_coeff.push_back(d);

    return true;
}

/// @brief get output from ransac 
/// @param inliers 
/// @param outliers 
/// @return 0/1
bool RansacPlane::getOutput(PointCloudPtr& inliers,PointCloudPtr& outliers)
{
    inliers = inliers_;
    outliers = outliers_;

    return true;
}

/// @brief get coefficients of best fitting plane
/// @return 0/1
std::vector<double>& RansacPlane::getPlaneCoeffs()
{
    return plane_coeff;
}
