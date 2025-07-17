#include "segmentation_worker.h"

using namespace std;

int main()
{
    PlanarSegmentation plane;

    if(!plane.setup())
    {
        cout<<"Setup failed"<<std::endl;
        return -1;
    }

    if(!plane.start())
    {
        cout<<"Start failed"<<std::endl;
        return -1;
    }

    RansacOutputPlane output;

    while(plane.isRunning())
    {
        plane.getPlaneData(output);
        

        // Do something with the output data
        
    }
}