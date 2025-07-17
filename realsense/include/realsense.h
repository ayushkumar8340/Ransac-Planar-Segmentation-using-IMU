/**
 * @file realsense.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief realsense camera
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/

#ifndef REALSENSE_H
#define REALSENSE_H

#include "input_device_interface.h"
#include "thread"
#include "mutex"
#include "atomic"

/// @brief realsense camera class
class Realsense : public InputDeviceInterface
{
    public:

        /// @brief constructor
        Realsense();

        /// @brief destructor
        ~Realsense();

        /// @brief setup input device
        bool setupInputDev(const InputDevSetup& setup);
        
        /// @brief start streaming data from camera
        bool startStreaming();
        
        /// @brief get point cloud data from camera
        bool getPointCloud(PointCloudPtr& p_cloud);
        
        /// @brief get coloured frame
        bool getColourFrame(ColourFrame& frame);
        
        /// @brief get depth frame 
        bool getDepthFrame(DepthFrame& frame);

        /// @brief set camera transformation (euler angles overload)
        bool setCameraTransformation(std::vector<double>& eu_angles);
        
        /// @brief set camera transformation 
        bool setCameraTransformation(cv::Mat& inp);

        /// @brief compute camera's rotation wrt to world
        bool computeCameraRotation();

        /// @brief get camera's rotation with respect to world (wRc)
        std::vector<double> getCameraRotation();
        
        /// @brief get camera calibration parameters
        InputDevParams& getParams();

        bool setOffsets(std::vector<double> offsets);
    private:

        /// @brief input device setup frame
        InputDevSetup setup_;
        
        /// @brief colour frame
        ColourFrame colour_frame_;
        
        /// @brief depth frame
        DepthFrame depth_frame_;
        
        /// @brief camera calibration parameters
        InputDevParams params_;

        /// @brief pipeline for camera stream
        rs2::pipeline pipe_;
        
        /// @brief realsense configurations
        rs2::config config_;
        
        /// @brief camera profile selector
        rs2::pipeline_profile selection_;
        
        /// @brief point cloud
        rs2::pointcloud pc_;
        
        /// @brief points of a point cloud
        rs2::points points_;

        /// @brief camera transformation
        cv::Mat wTc_;

        /// @brief gyroscope data
        std::vector<double> gyro_;

        /// @brief accelerometer data
        std::vector<double> accel_;

        /// @brief imu data mutex
        std::mutex imu_mutex_;

        /// @brief image data mutex
        std::mutex img_mutex_;

        /// @brief camera angle
        std::vector<double> angles_;

        /// @brief realsense frame set
        rs2::frameset frameset;

        /// @brief flag to check if image has arrived
        std::atomic<bool> img_arrived;

        /// @brief imu offset
        std::vector<double> offsets_;
};

#endif