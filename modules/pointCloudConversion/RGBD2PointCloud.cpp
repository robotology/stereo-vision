#include <cmath>
#include <string>
#include <iostream>
#include <yarp/conf/numeric.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/conf/system.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include "RGBD2PointCloud.hpp"
#include <yarp/os/NetFloat32.h>

YARP_BEGIN_PACK
typedef struct {
    yarp::os::NetFloat32  x;
    yarp::os::NetFloat32  y;
    yarp::os::NetFloat32  z;
    char        rgba[4];
} PC_Point;
YARP_END_PACK

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

RGBD2PointCloud::RGBD2PointCloud()
{
    use_RGBD_client = false;
    width           = 0;
    height          = 0;
    rosPC_data.header.seq = 0;
    frame_id = "/depth_center";
    scaleFactor = 1;
    mirrorX = 1;
    mirrorY = 1;
    mirrorZ = 1;
    publishTF = false;
}

RGBD2PointCloud::~RGBD2PointCloud()
{

}

Bottle RGBD2PointCloud::get_3D_points(const vector<Vector> &pixels, bool color)
{
    int index;
    Bottle point_cloud;
    Bottle &points=point_cloud.addList();
    uint32_t c;

    for (size_t i=0; i<pixels.size(); i++)
    {
        Vector pixel=pixels[i];
        int u=pixel[0];
        int v=pixel[1];
        index=rosPC_data.width*(v) + u;

        char *pointer = (char*) &rosPC_data.data[0];
        PC_Point *iter = (PC_Point*) &pointer[index*sizeof(PC_Point)];

        Bottle &pp=points.addList();
        pp.addFloat64(iter->x);
        pp.addFloat64(iter->y);
        pp.addFloat64(iter->z);

        if (color)
        {
            uint8_t a = iter->rgba[2];
            pp.addInt32(a);
            a = iter->rgba[1];
            pp.addInt32(a);
            a = iter->rgba[0];
            pp.addInt32(a);
        }
    }

    return point_cloud;
}

bool RGBD2PointCloud::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

bool RGBD2PointCloud::configure(ResourceFinder& rf)
{
    //looking for ROS parameters
    string  topicName, nodeName;
    Bottle& rosParam = rf.findGroup("ROS");
//     if(!rosParam.isNull())
    {
        if (rosParam.check("ROS_topicName"))
        {
            topicName = rosParam.find("ROS_topicName").asString();
        }
        else
        {
            topicName = "/RGBD2PointCloud";
        }
        if (rosParam.check("ROS_nodeName"))
        {
            nodeName = rosParam.find("ROS_nodeName").asString();
        }
        else
        {
            nodeName = "/RGBD2PointCloudNode";
        }

    }
    
    
    if(rf.check("help"))
    {
        yInfo() << " Required parameters:";
        yInfo() << "\tremoteImagePort: remote port streaming rgb images";
        yInfo() << "\tremoteDepthPort: remote port streaming depth images";
        yInfo() << " Optional parameters:";
        yInfo() << "\tscale: scale factor to apply to depth data. For Gazebo has to be 1, for xtion sensor 0.001 in order to scale data to [m]";
        yInfo() << "\tmirrorX, mirrorY, mirrorZ: add it to mirror the resulting point cloud on the corresponding axes (no param, just '--mirrorx')";
        yInfo() << "\ttf: if present a tf will be published. This requires 7 parameters";
        yInfo() << "\t  reference frame name,\n\t  x,y,z translations [m], \n\t  roll, pitch, yaw rotations [rad]\n";
        yInfo() << "\t  For example --tf base_link 0.1 0.0 0.0   1.56 0.0 0.0   -- no parenthesis";
        return false;
    }

    if(rf.check("frame_id"))
    {
        frame_id = rf.find("frame_id").asString();
    }


    if(!rf.check("remoteImagePort") || !rf.check("remoteDepthPort"))
    {
        yError() << "Missing required parameters: remoteImagePort or remoteDepthPort";
        return false;
    }

    yInfo() << "Node  name is " << nodeName;
    yInfo() << "Topic name is " << topicName;

    rosNode = new yarp::os::Node(nodeName);
    if(!pointCloud_outTopic.topic(topicName))
    {
        yError() << " opening " << topicName << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }

    // TBD: This is for debug only. Implement later
//     tf_broadcaster  = new tf::TransformBroadcaster;

    std::string remoteImagePort_name = rf.find("remoteImagePort").asString();
    std::string remoteDepthPort_name = rf.find("remoteDepthPort").asString();

    // cannot use because OpenNI2DeviceServer does not uses RGBDSensorWrapper yet
    if(use_RGBD_client)
    {
        // TBD: make ports opened by this device to use custom names
        yarp::os::Property params;
        params.put("device", "RGBDSensorClient");
        params.put("localImagePort","/RGBD2PointCloud/rgb/in:i");
        params.put("localDepthPort","/RGBD2PointCloud/depth/in:i");
        params.put("remoteImagePort",remoteImagePort_name);
        params.put("remoteDepthPort",remoteDepthPort_name);
        params.put("watchdog","100");
        params.put("synchPolicy","latest");

        if(!RGBD_Client.open(params))
        {
            yError() << "Cannot open device";
            return false;
        }
    }
    else
    {
        bool ret = true;
        ret &= imageFrame_inputPort.open("/RGBD2PointCloud/rgb/in:i");
        ret &= depthFrame_inputPort.open("/RGBD2PointCloud/depth/in:i");
        ret &= rpcPort.open("/RGBD2PointCloud/rpc");

        if(!ret)
        {
            yError() << "Cannot open required ports";
            return false;
        }

        attach(rpcPort);

        // TBD: make ports opened by this device to use custom names
        ret &= yarp::os::Network::connect(remoteImagePort_name, "/RGBD2PointCloud/rgb/in:i");
        ret &= yarp::os::Network::connect(remoteDepthPort_name, "/RGBD2PointCloud/depth/in:i");

        if(!ret)
        {
            yError() << "Cannot connect to remote ports";
            return false;
        }
        else
            yInfo() << "Connection is done!";
    }

    //
    // TBD: This is for debug only. Implement later
    //
//     Bottle tf = rf.findGroup("tf");
//     if(!tf.isNull() )
//     {
//         if(tf.size() != 1+7)rosPC_data
//         {
//             yError() << "TF parameter must have 7 elements: reference frame name, x,y,z [m] translations, roll, pitch, yaw rotations [rad]\n" \
//             "For example --tf base_link 0.1 0.0 0.0     1.56 0.0 0.0   -- no parenthesis";
//             return false;
//         }
//
//         translation.resize(3);
//         rotation.resize(3);
//         rotation_frame_id   = tf.get(1).asString();
//         translation[0]      = tf.get(2).asFloat64();
//         translation[1]      = tf.get(3).asFloat64();
//         translation[2]      = tf.get(4).asFloat64();
//
//         rotation[0]         = tf.get(5).asFloat64();
//         rotation[1]         = tf.get(6).asFloat64();
//         rotation[2]         = tf.get(7).asFloat64();
//     }

    frame_id = rosParam.check("frame_id", Value(frame_id)).asString();
    rf.check("mirrorX") ? mirrorX = -1 : mirrorX = 1;
    rf.check("mirrorY") ? mirrorY = -1 : mirrorY = 1;
    rf.check("mirrorZ") ? mirrorZ = -1 : mirrorZ = 1;

    yInfo() << "Mirrors: x=" << mirrorX << " y=" << mirrorY << " z= " << mirrorZ;
    yInfo() << "Frame id: " << frame_id;
    scaleFactor = rf.check("scale",   Value(1)).asFloat64();


    // Initialize the ROS pointCloud data with const values (width & height still unknown here)
    rosPC_data.header.seq++;
    rosPC_data.header.frame_id = frame_id;

    rosPC_data.fields.resize(4);
    rosPC_data.fields[0].name       = "x";
    rosPC_data.fields[0].offset     = 0;    // offset in bytes from start of each point
    rosPC_data.fields[0].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[0].count      = 1;    // how many FLOAT32 used for 'x'

    rosPC_data.fields[1].name       = "y";
    rosPC_data.fields[1].offset     = 4;    // offset in bytes from start of each point
    rosPC_data.fields[1].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[1].count      = 1;    // how many FLOAT32 used for 'y'

    rosPC_data.fields[2].name       = "z";
    rosPC_data.fields[2].offset     = 8;    // offset in bytes from start of each point
    rosPC_data.fields[2].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[2].count      = 1;    // how many FLOAT32 used for 'z'

    rosPC_data.fields[3].name       = "rgb";
    rosPC_data.fields[3].offset     = 12;   // offset in bytes from start of each point
    rosPC_data.fields[3].datatype   = 6;    // 6 = UINT32
    rosPC_data.fields[3].count      = 1;    // how many UINT32 used for 'rgb'

#if defined(YARP_BIG_ENDIAN)
    rosPC_data.is_bigendian = true;
#elif defined(YARP_LITTLE_ENDIAN)
    rosPC_data.is_bigendian = false;
#else
    #error "Cannot detect endianness"
#endif

    yAssert(sizeof(PC_Point) == 16);
    rosPC_data.point_step = sizeof(PC_Point);
    rosPC_data.is_dense = true;   // what this field actually means?? When is it false??
    return true;
}

double RGBD2PointCloud::getPeriod()
{
    return 0.033;
}

bool RGBD2PointCloud::updateModule()
{
    static double start = yarp::os::Time::now();

    if(yarp::os::Time::now() - start > 3.0f)
    {
        yInfo() << "RGBD2PointCloud is running fine.";
        start = yarp::os::Time::now();
    }
    return convertRGBD_2_XYZRGB();
}

bool RGBD2PointCloud::convertRGBD_2_XYZRGB()
{
    bool ret = imageFrame_inputPort.read(colorImage);
    ret &= depthFrame_inputPort.read(depthImage);

    imageFrame_inputPort.getEnvelope(colorStamp);
    depthFrame_inputPort.getEnvelope(depthStamp);

    if(!ret)
    {
        yError() << "Cannot read from ports";
        return false;
    }

    int d_width  = depthImage.width();
    int d_height = depthImage.height();
    int c_width  = colorImage.width();
    int c_height = colorImage.height();

    if( (d_width != c_width) && (d_height != c_height) )
    {
        yError() << "Size does not match";
        return false;
    }

    double tmp_sec;
    width  = d_width;
    height = d_height;

    rosPC_data.width  = width;
    rosPC_data.height = height;
    rosPC_data.row_step   = rosPC_data.point_step*width;

    rosPC_data.header.stamp.nsec = (NetUint32) (modf(depthStamp.getTime(), &tmp_sec)*1000000000);
    rosPC_data.header.stamp.sec  = (NetUint32) tmp_sec;
    int index = 0;

    unsigned char* colorDataRaw = (unsigned char*)colorImage.getRawImage();
    float* depthDataRaw = (float*)depthImage.getRawImage();

    // This data should be got at runtime from device (cameraInfo topic or rpc)
    double hfov = 58 * 3.14 / 360;

    double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));

    double fovHorizontalDeg = 58.62;
    double halfFovHorizontalRad = fovHorizontalDeg*M_PI/360.0;
//     double fovVerticalDeg = 45.666;
//     double halfFovVerticalRad = fovVerticalDeg*M_PI/360.0;

    double f=(width/2)/sin(halfFovHorizontalRad)*cos(halfFovHorizontalRad); // / sqrt(2);

    PC_Point point;
    rosPC_data.data.resize(width*height*rosPC_data.point_step);

    PC_Point *iter = (PC_Point*) &rosPC_data.data[0];

    double u, v, depth;
    // convert depth to point cloud
    for (int32_t j = 0; j < height; j++)
    {
        for (int32_t i = 0; i < width; i++)
        {
            depth   = depthDataRaw[index++] * scaleFactor;
            u       = -(i - 0.5*(width-1));
            v       = (0.5*(height-1) -j);
            point.x = -(NetFloat32) depth * u/f;
            point.y = -(NetFloat32) depth * v/f;
            point.z = (NetFloat32) depth;
            int new_i;
//            if( depth > 0)
//                new_i = (i + (int) ( (0.03 *f/depth) + 0.5) );
//            else
                new_i = i;

//            if( (new_i >= width) || (new_i < 0))
//            {
//                point.rgba[2] = 0;
//                point.rgba[1] = 0;
//                point.rgba[0] = 0;
//            }
//            else
//            {
                point.rgba[2] = (uint8_t) colorDataRaw[(j*width*3) + new_i*3 +0];
                point.rgba[1] = (uint8_t) colorDataRaw[(j*width*3) + new_i*3 +1];
                point.rgba[0] = (uint8_t) colorDataRaw[(j*width*3) + new_i*3 +2];
//            }

            *iter = point;
            iter++;
        }
    }


    pointCloud_outTopic.write(rosPC_data);

    // TBD: for debugging purpose only
    /*
    if(publishTF)
    {
        tf::StampedTransform camera_base_tf(
                    tf::Transform(tf::createQuaternionFromRPY(rotation[0], rotation[1], rotation[2]),
                    tf::Vector3(translation[0], translation[1], translation[2])), ros::Time::now(),
                    rotation_frame_id.c_str(), frame_id);
        tf_broadcaster->sendTransform(camera_base_tf);
    }
    */
    return true;
}

bool RGBD2PointCloud::interruptModule()
{
    yDebug() << "Interrupting module";
    imageFrame_inputPort.interrupt();
    depthFrame_inputPort.interrupt();

    imageFrame_inputPort.close();
    depthFrame_inputPort.close();

    pointCloud_outTopic.close();
    rpcPort.close();
    return true;
}

bool RGBD2PointCloud::close()
{
    yTrace();
    interruptModule();
    return true;
}

