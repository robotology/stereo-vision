#include "disparityModule.h"
#include <yarp/os/Stamp.h>



bool stereoModule::configure(yarp::os::ResourceFinder &rf)
{    


    moduleName            = rf.check("name", 
                           Value("stereoDisparity"), 
                           "module name (string)").asString();

    setName(moduleName.c_str());


    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();


    inputLeftPortName         = "/";
    inputLeftPortName        += getName(
                           rf.check("InputPortLeft",Value("/cam/left:i"),"Input image port (string)").asString());

    inputRightPortName        = "/";
    inputRightPortName       += getName(
                           rf.check("InputPortRight", 
                           Value("/cam/right:i"),
                           "Input image port (string)").asString()
                           );

    outputPortName        = "/";
    outputPortName       += getName(
                           rf.check("OutPort", 
                           Value("/disparity:o"),
                           "Output image port (string)").asString()
                           );


    handlerPortName        = "/";
    handlerPortName       += getName(
                           rf.check("CommandPort", 
                           Value("/rpc"),
                           "Output image port (string)").asString()
                           );



    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << ": unable to open port " << handlerPortName << endl;
        return false;
    }

    string calibPath=(rf.getContextPath()+"/").c_str();

    attach(handlerPort);


    dispThread = new disparityThread(inputLeftPortName, inputRightPortName,outputPortName, calibPath,&handlerPort);

    dispThread->start(); 

    return true ;

}


bool stereoModule::interruptModule()
{
   dispThread->stop();


    return true;
}


bool stereoModule::close()
{

    dispThread->stop();
    delete dispThread;

    return true;
}


bool stereoModule::respond(const Bottle& command, Bottle& reply) 
{
    if(command.size()==0)
        return false;

    if (command.get(0).asString()=="quit") {
        cout << "closing..." << endl;
        return false;
   }
    else if (command.get(0).asString()=="Point" || command.get(0).asString()=="Left" ) {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt(); 
        Point3f point = dispThread->get3DPoints(u,v);
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
   }
    else if (command.size()==2) {
        int u = command.get(0).asInt();
        int v = command.get(1).asInt(); 
        Point3f point = dispThread->get3DPoints(u,v);
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
   }
    else if (command.get(0).asString()=="Right") {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt(); 
        Point3f point = dispThread->get3DPoints(u,v,"RIGHT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
   }

    else if (command.get(0).asString()=="Root") {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt(); 
        Point3f point = dispThread->get3DPoints(u,v,"ROOT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
   }
    return true;
}


bool stereoModule::updateModule()
{
   return true;
}



double stereoModule::getPeriod()
{

    
   return 0.1;
}
