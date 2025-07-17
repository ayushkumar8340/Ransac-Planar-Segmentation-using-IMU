/**
 * @file pre_processing_interface.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief PreProcessing interface
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/
#ifndef PRE_PROCESSING_INTERFACE_H
#define PRE_PROCESSING_INTERFACE_H

#include "pre_processing_configs.h"
#include "camera_frames.h"
#include <pcl/filters/voxel_grid.h>

/// @brief pre processing interface 
class PreProcessingInterface
{
    public:
        /// @brief setup pre processing class
        virtual bool setupPreProcessing(PreProcessingMethods,const PreProcessConfig&) = 0;
        
        /// @brief pre process point cloud
        virtual bool preProcess(const PointCloudPtr&,PointCloudPtr&) = 0;
};

#endif