#include "disparityModule.h"
#include <yarp/os/Stamp.h>



bool stereoModule::configure(yarp::os::ResourceFinder &rf)
{    


   moduleName            = rf.check("name", 
                           Value("stereoDisparity"), 
                           "module name (string)").asString();
   
   setName(moduleName.c_str());


   robotName             = rf.check("robot", 
                           Value("icubSim"), 
                           "Robot name (string)").asString();

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

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
	
   
   output3DPointName= "/";
   output3DPointName       += getName(
                           rf.check("OutPointPort", 
                           Value("/worldpoint:o"),
                           "Output image port (string)").asString()
                           );


   inputFixationName= "/";
   inputFixationName       += getName(
                           rf.check("InputFixation", 
                           Value("/fixation:i"),
                           "Output image port (string)").asString()
                           );
	
   int useFixation= rf.check("useFixation", Value(0)).asInt();
	

   handlerPortName        = "/";
   handlerPortName       += getName(
                           rf.check("CommandPort", 
                           Value("/cmd"),
                           "Output image port (string)").asString()
                           );
	
    if (!handlerPort.open(handlerPortName.c_str())) {
      cout << ": unable to open port " << handlerPortName << endl;
      return false;
   }

    string calibPath=(rf.getContextPath()+"/").c_str();
 
   attach(handlerPort);


   dispThread = new disparityThread(inputLeftPortName, inputRightPortName,outputPortName,
       output3DPointName, inputFixationName, calibPath,&handlerPort,useFixation);


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
  if (command.get(0).asString()=="quit") {
       cout << "closing..." << endl;
       return false;     
   }
  if (command.get(0).asString()=="s") {
    
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
