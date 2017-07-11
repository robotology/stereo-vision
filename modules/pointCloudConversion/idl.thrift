# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift

struct Vector
{
} (
   yarp.name = "yarp::sig::Vector"
   yarp.includefile="yarp/sig/Vector.h"
  )

struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )


/**
* RGBD2PointCloud_IDL
*
* IDL Interface to \ref RGBD2PointCloud services.
*/

service RGBD2PointCloud_IDL
{
    /**
    * Get the point cloud corresponding to a 2D blob of pixels
    * @param  pixels list of Vector of pixels
    * @return a bottle containing the 3D points
    */
    Bottle get_3D_points(1:list<Vector> pixels, 2:bool color);
}
