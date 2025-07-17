#include "realsense.h"

/// @brief constructor
Realsense::Realsense()
{
    gyro_ = std::vector<double>(3,0);
    accel_ = std::vector<double>(3,0);
    angles_ = std::vector<double>(3,0);
    offsets_ = std::vector<double>(3,0);
    img_arrived.store(false,std::memory_order_acquire);
}

/// @brief destructor
Realsense::~Realsense()
{

}

/// @brief setup device
/// @param setup setup frame
/// @return 0/1
bool Realsense::setupInputDev(const InputDevSetup& setup)
{
    setup_ = setup;
    config_.enable_stream(RS2_STREAM_COLOR,setup_.width,setup_.height,RS2_FORMAT_BGR8,setup_.fps);
    config_.enable_stream(RS2_STREAM_DEPTH,setup_.width,setup_.height,RS2_FORMAT_Z16,setup_.fps);
    config_.enable_stream(RS2_STREAM_ACCEL,RS2_FORMAT_MOTION_XYZ32F);
    config_.enable_stream(RS2_STREAM_GYRO,RS2_FORMAT_MOTION_XYZ32F);

    auto imuCallback_ = [&](rs2::frame frame)
    {
        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
            img_mutex_.lock();
            frameset = fs;
            img_mutex_.unlock();

            img_arrived.store(true,std::memory_order_acquire);

        }

        else if(rs2::motion_frame m_frame = frame.as<rs2::motion_frame>())
        {
            rs2::motion_frame motion = frame.as<rs2::motion_frame>();

            if(motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {

                rs2_vector accel_data = motion.get_motion_data();

                imu_mutex_.lock();
                accel_[0] = accel_data.x;
                accel_[1] = accel_data.y;
                accel_[2] = accel_data.z;
                imu_mutex_.unlock();

            }
            
            else if(motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {

                rs2_vector gyro_data = motion.get_motion_data();

                imu_mutex_.lock();
                gyro_[0] = gyro_data.x;
                gyro_[1] = gyro_data.y;
                gyro_[2] = gyro_data.z;
                imu_mutex_.unlock();

            }
        }
    };

    selection_ = pipe_.start(config_,imuCallback_);
    params_.setDefault();
    wTc_ = cv::Mat::eye(cv::Size(4,4),CV_64FC1);

    return true;

}

/// @brief start streaming data from device
/// @return 0/1
bool Realsense::startStreaming()
{
    // wait for new frames
    while(!img_arrived.load(std::memory_order_acquire));

    img_mutex_.lock();
    rs2::video_frame colour_frame = frameset.get_color_frame();
    rs2::depth_frame depth = frameset.get_depth_frame();
    img_mutex_.unlock();

    depth_frame_.depth_data = cv::Mat(cv::Size(setup_.width,setup_.height),CV_16U,(void*)depth.get_data(),cv::Mat::AUTO_STEP);
    colour_frame_.colour_frame = cv::Mat(cv::Size(setup_.width,setup_.height),CV_8UC3,(void*)colour_frame.get_data());
    pc_.map_to(colour_frame);
    points_ = pc_.calculate(depth);

    img_arrived.store(false,std::memory_order_acquire);

    return true;
}

/// @brief set camera transformation wrt world
/// @param inp transformation matrix
/// @return 0/1
bool Realsense::setCameraTransformation(cv::Mat& inp)
{
    wTc_ = inp;
    return true;
}

/// @brief set camera orientation wrt world (Euler angle overload)
/// @param eu_angles input angles
/// @return 0/1
bool Realsense::setCameraTransformation(std::vector<double>& eu_angles)
{
    return true;
}

/// @brief get colour frame form device
/// @param frame colour frame
/// @return 0/1
bool Realsense::getColourFrame(ColourFrame& frame)
{
    colour_frame_.colour2Gray();
    frame = colour_frame_;
    return true;
}

/// @brief get depth frame from device
/// @param frame depth frame
/// @return 0/1
bool Realsense::getDepthFrame(DepthFrame& frame)
{
    depth_frame_.computeDepth(setup_.depth_map_factor);
    frame = depth_frame_;
    return true;
}

/// @brief get point cloud from device
/// @param p_cloud point cloud pointer
/// @return 0/1
bool Realsense::getPointCloud(PointCloudPtr& p_cloud)
{
    p_cloud->width = setup_.width;
    p_cloud->height = setup_.height;
    p_cloud->is_dense = false;
    p_cloud->points.resize(points_.size());
    auto ptr = points_.get_vertices();
    for(auto& p : p_cloud->points)
    {
        p.x = wTc_.at<double>(0,0) * ptr->x - wTc_.at<double>(0,1) * ptr->y - wTc_.at<double>(0,2) * ptr->z + wTc_.at<double>(0,3);
        p.y = wTc_.at<double>(1,0) * ptr->x - wTc_.at<double>(1,1) * ptr->y - wTc_.at<double>(1,2) * ptr->z + wTc_.at<double>(1,3);
        p.z = wTc_.at<double>(2,0) * ptr->x - wTc_.at<double>(2,1) * ptr->y - wTc_.at<double>(2,2) * ptr->z + wTc_.at<double>(2,3);
        ptr++;
    }

    return true;
}

/// @brief compute camera rotation with respect to world using imu
/// @return 0/1
bool Realsense::computeCameraRotation()
{
    double x,z = 0.0;

    imu_mutex_.lock();
    z = atan2(accel_[1],accel_[2]);
    x = atan2(accel_[0],sqrt(accel_[1] * accel_[1] + accel_[2] * accel_[2]));
    imu_mutex_.unlock();

    angles_[2] = atan2(sin(x),cos(x));
    angles_[0] = atan2(sin(z),cos(z));

    return true;

}

/// @brief get camera rotation wrt world
/// @return euler angles (rpy)
std::vector<double> Realsense::getCameraRotation()
{
    std::vector<double> out{(angles_[0] + offsets_[0]) , (angles_[1] + offsets_[1]) , (angles_[2] + offsets_[2])};
    return out;
}

/// @brief get camera calibration parameters
/// @return camera paramteres
InputDevParams& Realsense::getParams()
{
    return params_;
}

/// @brief set imu offsets
/// @param offsets 
/// @return 
bool Realsense::setOffsets(std::vector<double> offsets)
{
    offsets_ = offsets;
    return true;
}
