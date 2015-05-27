/* 
 * Copyright (C) 2011 RobotCub Consortium
 * Author: Sean Ryan Fanello, Ilaria Gori
 * email:   sean.fanello@iit.it, ilaria.gori@iit.it
 * website: www.robotcub.org
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

#include "sceneFlowModule.h"

bool sceneFlowModule::configure(ResourceFinder &rf)
{
    sceneFlow=new SceneFlow(rf);
    if(sceneFlow->isOpen())
    {
        sceneFlow->start();
        Time::delay(0.5);
        return true;
    }
    else
    {
        delete sceneFlow;
        return false;
    }
    namedWindow("flowField");
    namedWindow("Module");
}

bool sceneFlowModule::close()
{
    delete sceneFlow;   
    return true;
}

bool sceneFlowModule::interruptModule()
{
    fprintf(stdout, "Closing Scene Flow...\n");
    sceneFlow->suspend();
    sceneFlow->close();     
    return true;
}
bool sceneFlowModule::updateModule()
{
    if(isStopping())
        return false;

    // Whole Flow Access
    int u=160;
    int v=120;
    Mat flow3D;
    sceneFlow->getSceneFlow(flow3D);
    Point3f Pflow3D;

    if(!flow3D.empty())
    {

        Pflow3D.x=flow3D.ptr<float>(v)[2*u]; // Flow DeltaX
        Pflow3D.y=flow3D.ptr<float>(v)[2*u+1]; // Flow DeltaY
        Pflow3D.z=flow3D.ptr<float>(v)[2*u+2]; // Flow DeltaZ
        
        fprintf(stdout,"3D Motion of Pixel (%i,%i): (%f, %f, %f) \n",u,v,Pflow3D.x,Pflow3D.y,Pflow3D.z);

        // Compute and show motion field and flow module

        IplImage* module=cvCreateImage(cvSize(sceneFlow->getImgWidth(),sceneFlow->getImgHeight()),8,3);
        IplImage* flowField=sceneFlow->draw2DMotionField();
        sceneFlow->drawFlowModule(module);
        
        if(flowField!=NULL)
            cvShowImage("flowField", flowField);
        cvShowImage("Module", module);
        cvWaitKey(5);

        cvReleaseImage(&flowField);
        cvReleaseImage(&module);
    }


    return true;
}

double sceneFlowModule::getPeriod()
{
    return 0.1;
}





int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("sceneFlowModule");
    rf.configure(argc,argv);
    sceneFlowModule mod;

    return mod.runModule(rf);
}
