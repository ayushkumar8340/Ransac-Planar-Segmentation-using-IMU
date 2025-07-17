#ifndef SEGMENTATION_WORKER_H
#define SEGMENTATION_WORKER_H

#include "realsense.h"
#include "pre_processing.h"
#include "segmentation_config.h"
#include "thread"
#include "mutex"

class PlanarSegmentation
{
    public:
        PlanarSegmentation();
        ~PlanarSegmentation(){}
        bool setup();
        bool start();
        bool stop();
        bool getPlaneData(RansacOutputFrame& output);
        bool reset();
        bool isRunning();

    private:
        std::unique_ptr<InputDeviceInterface> dev_;
        std::unique_ptr<PreProcessingInterface> pre_process_;
        std::unique_ptr<RansacPlane> ransac_seg_;

        PreProcessConfig pre_process_setup_;
        InputDevSetup input_device_setup_;
        RansacConfig ransac_config_;

        RansacOutputFrame output_;

        std::thread worker_th_;
        std::atomic<bool> is_running_;

        std::mutex mut_;
        std::vector<double> offsets_;

        void work_();

        std::vector<double> computeNormals_(std::vector<double>& angle);

};



#endif