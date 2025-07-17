/**
 * @file detection_config.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief Detection configs and frames
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/

#ifndef SEGMENTATION_CONFIGS_H
#define SEGMENTATION_CONFIGS_H

#include "camera_frames.h"

/// @brief different planes for ransac
enum RANSAC_PLANES
{
    X_NORMAL,
    Y_NORMAL,
    Z_NORMAL
};

/// @brief ransac configurations
#define RANSAC_MAX_ITRS 50
#define RANSAC_DISTANCE_THRESHOLD 0.03
#define RANSAC_GROUND_PLANE RANSAC_PLANES::Y_NORMAL

/// @brief ransac configuration structure
typedef struct 
{
    int max_iterations;
    double distance_threshold;
    RANSAC_PLANES plane;

    void setDefault()
    {
        max_iterations = RANSAC_MAX_ITRS;
        distance_threshold = RANSAC_DISTANCE_THRESHOLD;
        plane = RANSAC_GROUND_PLANE;
    }

}RansacConfig;

typedef struct 
{
    PointCloudPtr inliers;
    PointCloudPtr outlier;
    std::vector<double> plane_coeff;

    RansacOutputFrame()
    {
        inliers = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        outliers = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    void reset()
    {
        inliers.clear();
        outliers.clear();
        plane_coeff.clear();
    }

}RansacOutputFrame;


#endif

