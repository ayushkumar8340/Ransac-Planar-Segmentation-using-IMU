#include "segmentation_worker.h"

bool PlanarSegmentation::PlanarSegmentation()
{
    offsets_ = {CAMERA_OFFSETS_X,CAMERA_OFFSETS_Y,CAMERA_OFFSETS_Z};

    dev_ = std::make_unique<Realsense>();
    pre_process_ = std::make_unique<PCPreProcessing>();
    ransac_seg_ = std::make_unqiue<RansacPlane>();
    is_running_.store(false,std::memory_order_acquire);

}

bool PlanarSegmentation::setup()
{
    input_device_setup_.setDefault();
    pre_process_setup_.setDefault();
    ransac_config_.setDefault();

    if(!dev_->setupInputDev(input_device_setup_))
    {
        std::cout<<"[ERROR] Input device setup failed."<<std::endl;
        return false;
    }

    if(!pre_process_->setupPreProcessing(PreProcessingMethods::VOXEL,pre_prcoess_setup_))
    {
        std::cout<<"[ERROR] Pre-processing setup failed"<<std::endl;
        return false;
    }

    if(!ransac_seg_->setup(ransac_config_.max_iterations,ransac_config_.distance_threshold))
    {
        std::cout<<"[ERROR] Ransac planar segmentation setup failed"<<std::endl;
        return false;
    }

    dev_->setOffsets();

    return true;
}

bool PlanarSegmentation::start()
{
    if(is_running_.load(std::memory_order_acquire))
    {
        return false;
    }

    is_running_.store(true,std::memory_order_acquire)
    worker_th_ = std::thread(&PlanarSegmentation::work_,this);

    return true;
}

bool PlanarSegmentation::stop()
{
    is_running_.store(false,std::memory_order_acquire);
    while(worker_th_.joinable());
    worker_th_.join();
    return true;
}

bool PlanarSegmentation::isRunning()
{
    return is_running_.load(std::memory_order_acquire);
}

bool PlanarSegmentation::getPlaneData(RansacOutputFrame& output)
{
    if(!is_running_.load(std::memory_order_acquire))
    {
        return false;
    }

    mut_.lock();
    output = output_;
    mut_.unlock();

    return true;

}

void PlanarSegmentation::work_()
{
    while(!is_running_.load(std::memory_order_acquire))
    {

    }

    while(is_running_.load(std::memory_order_acquire))
    {
        ColourFrame frame;
        PointCloudPtr inp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        PointCloudPtr processed(new pcl::PointCloud<pcl::PointXYZ>);

        mut_.lock();
        output_.reset();
        mut_.unlock();

        dev_->startStreaming();
        dev_->getColourFrame(frame);
        dev_->getPointCloud(inp_cloud);
        dev_->computeCameraRotation();

        std::vector<double> angles = dev_->getCameraRotation();

        pre_process_->preProcess(inp_cloud,processed);

        std::vector<double> normal = computeNormals_(angles);

        ransac_seg_->setPlaneNormal(normal);
        ransac_seg_->computeInliers(processed);
        mut_.lock();
        ransac_seg_->getOuput(output_.inliers,output_.outliers);
        output_.plane_coeff = ransac_seg_->getPlaneCoeff();
        mut_.unlock();

        frame.vis("Camera Output");
        
    }
}

std::vector<double> PlanarSegmentation::computeNormals_(std::vector<double>& angles)
{
    
    std::vector<double> normal;
    normal.push_back((-cos(-angle[0]) * sin(-angle[2])));
    normal.push_back((cos(-angle[0]) * cos(-angle[2])));
    normal.push_back(sin(-angle[0]));

    return normal;  
}
