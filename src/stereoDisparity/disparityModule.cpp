#include "disparityModule.h"
#include <yarp/os/Stamp.h>



bool stereoModule::configure(yarp::os::ResourceFinder &rf)
{    


   moduleName            = rf.check("name", 
                           Value("stereoModule"), 
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

    string calibPath=rf.getContextPath()+"/";
 
   attach(handlerPort);
   /* create the thread and pass pointers to the module parameters */

   dispThread = new disparityThread(inputLeftPortName, inputRightPortName,outputPortName, calibPath,&handlerPort);

   /* now start the thread to do the work */

   dispThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool stereoModule::interruptModule()
{
   dispThread->stop();


   return true;
}


bool stereoModule::close()
{

   /* stop the thread */

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


/* Called periodically every getPeriod() seconds */

bool stereoModule::updateModule()
{
   return true;
}



double stereoModule::getPeriod()
{

    
   return 0.1;
}
