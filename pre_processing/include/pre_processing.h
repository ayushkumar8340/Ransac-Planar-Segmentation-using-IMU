/**
 * @file pre_processing.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief PreProcessing configs and frames
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/

#ifndef PRE_PROCESSING_H
#define PRE_PROCESSING_H

#include "pre_processing_interface.h"

/// @brief point cloud pre-processing class
class PCPreProcessing : public PreProcessingInterface
{
    public:

        /// @brief setup pre processing class 
        bool setupPreProcessing(PreProcessingMethods,const PreProcessConfig& setup);
        
        /// @brief pre process input point cloud
        bool preProcess(const PointCloudPtr& inp, PointCloudPtr& out);
    
    private:

        /// @brief setup frame
        PreProcessConfig setup_;
        
        /// @brief methods of pre processing 
        PreProcessingMethods method_;
};


#endif