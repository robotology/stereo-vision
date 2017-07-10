
#include <yarp/os/LogStream.h>
#include "RGBD2PointCloud.hpp"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

int main(int argc,char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    RGBD2PointCloud module;

    rf.configure(argc,argv);
    return module.runModule(rf);
}
