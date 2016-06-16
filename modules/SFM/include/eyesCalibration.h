/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <string>
#include <deque>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>


/**************************************************************************/
struct CalibrationData
{
    double vergence;
    yarp::sig::Matrix eye_kin_left;
    yarp::sig::Matrix eye_kin_right;
    yarp::sig::Matrix fundamental;

    CalibrationData() : vergence(0.0),
                        eye_kin_left(yarp::math::eye(4,4)),
                        eye_kin_right(yarp::math::eye(4,4)),
                        fundamental(yarp::math::eye(4,4)) { }
};


/**************************************************************************/
class EyesCalibration
{
    std::deque<CalibrationData> data;

public:
    CalibrationData &addData();
    yarp::sig::Vector calibrate(yarp::sig::Matrix &extrinsics_left,
                                yarp::sig::Matrix &extrinsics_right,
                                const std::string &logFile);
};

