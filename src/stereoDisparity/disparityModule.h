#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include "disparityThread.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  

class stereoModule:public RFModule
{
   /* module parameters */

   string moduleName;
   string robotName; 
   string inputLeftPortName;
   string inputRightPortName;
   string outputPortName;  
   string output3DPointName;
   string handlerPortName;
   int thresholdValue;

   /* class variables */

   Port handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   disparityThread *dispThread;


public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};

