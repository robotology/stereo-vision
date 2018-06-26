#ifndef RGBD_2_POINT_CLOUD_H
#define RGBD_2_POINT_CLOUD_H

#include <cmath>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Stamp.h>
#include <yarp/rosmsg/sensor_msgs/PointCloud2.h>

#include "RGBD2PointCloud_IDL.h"

class RGBD2PointCloud : public yarp::os::RFModule,
                        public RGBD2PointCloud_IDL
{
private:
    bool use_RGBD_client;
    yarp::dev::PolyDriver                       RGBD_Client;

    int  width, height;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>    colorImage;
    yarp::sig::ImageOf<yarp::sig::PixelFloat>  depthImage;
    yarp::os::Port                             imageFrame_inputPort;   // rgb input
    yarp::os::Port                             depthFrame_inputPort;   // depth input
    yarp::os::RpcServer                        rpcPort;

    yarp::os::Stamp colorStamp, depthStamp;

    int mirrorX;
    int mirrorY;
    int mirrorZ;

    bool publishTF;
    std::string rotation_frame_id;
    yarp::sig::Vector translation;
    yarp::sig::Vector rotation;

    double scaleFactor;

    // ROS stuff
    std::string                                          frame_id;
    yarp::os::Node                                      *rosNode;
    yarp::rosmsg::sensor_msgs::PointCloud2               rosPC_data;
    yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>    pointCloud_outTopic;

    // Publishing TF is useful only for debugging purposes. In real use case, someone else
    // robot-aware should publish the tf. Here I just need to publish the frame_id
    //     tf::TransformBroadcaster    *tf_broadcaster;

public:

    RGBD2PointCloud();
    ~RGBD2PointCloud();
    bool convertRGBD_2_XYZRGB();

    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    double getPeriod();
    bool interruptModule();
    bool close();
    yarp::os::Bottle get_3D_points(const std::vector<yarp::sig::Vector> &pixels,  bool color);
    bool attach(yarp::os::RpcServer &source);
};


#endif  //RGBD_2_POINT_CLOUD_H
