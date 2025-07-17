#include "pre_processing.h"

/// @brief setup pre processing class
/// @param method pre processing methods to be used
/// @param setup setup frame
/// @return 0/1
bool PCPreProcessing::setupPreProcessing(PreProcessingMethods method,const PreProcessConfig& setup)
{
    setup_ = setup;
    method_ = method;
    return true;
}

/// @brief pre process input point cloud
/// @param inp input point cloud
/// @param out output point cloud
/// @return 0/1
bool PCPreProcessing::preProcess(const PointCloudPtr& inp,PointCloudPtr& out)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(inp);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(setup_.noise_config.x_min,setup_.noise_config.x_max);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-setup_.noise_config.x_max,setup_.noise_config.x_min);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-setup_.noise_config.x_max,setup_.noise_config.x_min);
    pass.filter(*out);

    if(method_ == PreProcessingMethods::NOISE_ELEMINATION)
    {
        return true;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(out);
    vox.setLeafSize(setup_.voxel_config.lx,setup_.voxel_config.ly,setup_.voxel_config.lz);
    vox.filter(*out); 

    return true;
}