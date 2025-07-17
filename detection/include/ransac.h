/**
 * @file ransac.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief Ransac planar segmentation implementation
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/

#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include "camera_frames.h"
#include <unordered_set>
#include "detection_config.h"

/// @brief Ransac planar segmentation class
class RansacPlane
{
    public:


        /// @brief setup ransac planar segmentation
        bool setup(const int& max_itr,const double& dist_th);

        /// @brief set normal of the plane
        bool setPlaneNormal(std::vector<double>& normal);

        /// @brief set normal of the plane (overload X Y Z enums)
        bool setPlaneNormal(RANSAC_PLANES normal);
        
        /// @brief compute best plane
        bool computeInliers(const PointCloudPtr& cloud);
        
        /// @brief get inliers and outliers
        bool getOutput(PointCloudPtr& inliers,PointCloudPtr& outliers);
        
        /// @brief get plane coefficients
        std::vector<double>& getPlaneCoeffs();
        
        /// @brief reset ransac 
        bool reset();

    private:
        
        /// @brief max iterations to run 
        int max_itrs_;
        
        /// @brief max distance of a point from the plane to be treated as an inlier
        double distance_threshold_;

        /// @brief normal of the plane
        std::vector<double> normal_;
        
        /// @brief plane coefficients
        std::vector<double> plane_coeff;

        /// @brief inliers 
        PointCloudPtr inliers_;
        
        /// @brief outliers
        PointCloudPtr outliers_;
};

#endif