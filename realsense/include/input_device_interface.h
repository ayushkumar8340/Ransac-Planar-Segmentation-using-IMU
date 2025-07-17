/**
 * @file input_device_interface.h
 * @author Ayush Kumar (akp8340135@gmail.com)
 * @brief Input device interface
 * @version 0.1
 * @date 2024-06-11
 * 
 * @copyright Copyright (c) 2023
 * 
**/

#ifndef INPUT_DEVICE_INTERFACE_H
#define INPUT_DEVICE_INTERFACE_H

#include <librealsense2/rs.hpp>
#include "camera_frames.h"

/// @brief input device interface 
class InputDeviceInterface
{
    public:

        /// @brief setup input device
        virtual bool setupInputDev(const InputDevSetup& setup) = 0;

        /// @brief start streaming data from input device
        virtual bool startStreaming() = 0;

        /// @brief get point cloud
        virtual bool getPointCloud(PointCloudPtr&) = 0;

        /// @brief get colour frames
        virtual bool getColourFrame(ColourFrame&) = 0;

        /// @brief get depth frames
        virtual bool getDepthFrame(DepthFrame&) = 0;

        /// @brief get camera calibration parameters
        virtual InputDevParams& getParams() = 0;

        /// @brief set camera transformation wrt to world frame (wTc)
        virtual bool setCameraTransformation(cv::Mat& inp) = 0;

        /// @brief set camera transformation (euler angles overload)
        virtual bool setCameraTransformation(std::vector<double>& eu_angles) = 0;

        /// @brief
        virtual bool computeCameraRotation() = 0;

        /// @brief
        virtual std::vector<double> getCameraRotation() = 0;

        /// @brief
        virtual bool setOffsets(std::vector<double> offsets) = 0;
};  


#endif