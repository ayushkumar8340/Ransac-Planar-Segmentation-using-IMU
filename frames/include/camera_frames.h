/**
 * @file camera_frames.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief Camera configs and frames
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/

#ifndef CAMERA_FRAMES_H
#define CAMERA_FRAMES_H

#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

/// @brief camera configurations
#define DEFAULT_CAM_HEIGHT 480
#define DEFAULT_CAM_WIDTH 640
#define FPS 30

#define REALSENE

/// @brief camera intrinsics
#ifndef REALSENSE
#define CAM_Fx 617.201
#define CAM_Fy 617.362
#define CAM_Cx 324.637
#define CAM_Cy 242.462
#define CAM_K1 0
#define CAM_K2 0
#define CAM_P1 0
#define CAM_P2 0
#define CAM_K3 0
#define DEPTH_MAP_FACTOR 1000
#endif

#ifdef TMP_CAM
#define CAM_Fx 0
#define CAM_Fy 0
#define CAM_Cx 0
#define CAM_Cy 0
#define CAM_K1 0
#define CAM_K2 0
#define CAM_P1 0
#define CAM_P2 0
#define CAM_K3 0
#endif

/// @brief input device configurations
typedef struct 
{
    int height;
    int width;
    int fps;
    double depth_map_factor;
    
    void setDefault()
    {

        height = DEFAULT_CAM_HEIGHT;
        width = DEFAULT_CAM_WIDTH;
        fps = FPS;
        depth_map_factor = 1.0 / DEPTH_MAP_FACTOR;
    }

}InputDevSetup;

/// @brief camera intrinsics and extrinsics
typedef struct 
{
    cv::Mat intrinsics = cv::Mat::zeros(cv::Size(3,3),CV_64FC1);
    cv::Mat dist_coeff = cv::Mat::zeros(cv::Size(5,1),CV_64FC1);
    
    
    void setDefault()
    {
        intrinsics.at<double>(0,0) = CAM_Fx;
        intrinsics.at<double>(0,1) = 0;
        intrinsics.at<double>(0,2) = CAM_Cx;
        intrinsics.at<double>(1,0) = 0;
        intrinsics.at<double>(1,1) = CAM_Fy;
        intrinsics.at<double>(1,2) = CAM_Cy;
        intrinsics.at<double>(2,0) = 0;
        intrinsics.at<double>(2,1) = 0;
        intrinsics.at<double>(2,2) = 1;

        dist_coeff.at<double>(0) = CAM_K1;
        dist_coeff.at<double>(1) = CAM_K2;
        dist_coeff.at<double>(2) = CAM_P1;
        dist_coeff.at<double>(3) = CAM_P2;
        dist_coeff.at<double>(4) = CAM_K3;
    }


}InputDevParams;

/// @brief camera frame datatype
typedef struct 
{
    cv::Mat colour_frame;
    cv::Mat gray_frame;

    void colour2Gray()
    {
        cv::cvtColor(colour_frame,gray_frame,cv::COLOR_BGR2GRAY);
    }

    void vis(std::string name)
    {
        cv::imshow(name,colour_frame);
        cv::waitKey(1);
    }

}ColourFrame;

/// @brief depth frame data type
typedef struct 
{
    cv::Mat depth_data;

    void computeDepth(double scale)
    {
        depth_data.convertTo(depth_data,CV_64F,scale);
    }

    double getDepth(int u,int v)
    {
        return depth_data.at<double>(u,v);
    }



}DepthFrame;

/// @brief point cloud pointer (pcl)
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;


#endif