/**
 * @file pre_processing_configs.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief PreProcessing configs and frames
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/

#ifndef PRE_PROCESSING_CONFIGS_H
#define PRE_PROCESSING_CONFIGS_H


/// @brief pre processing noises
#define NOISE_X_MIN 0
#define NOISE_X_MAX 1
#define NOISE_Y_MIN 0
#define NOISE_Y_MAX 1
#define NOISE_Z_MIN 0
#define NOISE_Z_MAX 1

/// @brief voxel leaf size
#define VOXEL_Lx 0.01
#define VOXEL_Ly 0.01
#define VOXEL_Lz 0.01

/// @brief pre processing methods
enum PreProcessingMethods
{
    NOISE_ELEMINATION,
    VOXEL
};

/// @brief noise elimination configuration structure
typedef struct
{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;

    void setDefault()
    {
        x_min = NOISE_X_MIN;
        x_max = NOISE_X_MAX;
        y_min = NOISE_Y_MIN;
        y_max = NOISE_Y_MAX;
        z_min = NOISE_Z_MIN;
        z_max = NOISE_Z_MAX;
    }

}NoiseEliminationConfig;

/// @brief Voxel confguration structure
typedef struct
{
    double lx;
    double ly;
    double lz;

    void setDefault()
    {
        lx = VOXEL_Lx;
        ly = VOXEL_Ly;
        lz = VOXEL_Lz;
    }
    
}VoxelConfig;

/// @brief pre processing configuration structure
typedef struct
{
    NoiseEliminationConfig noise_config;
    VoxelConfig voxel_config;

    void setDefault()
    {
        noise_config.setDefault();
        voxel_config.setDefault();
    }

}PreProcessConfig;


#endif