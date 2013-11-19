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
   string handlerPortName;
   int thresholdValue;

   Port handlerPort;      


   disparityThread *dispThread;


public:
   
   bool configure(yarp::os::ResourceFinder &rf); 
   bool interruptModule();                       
   bool close();                                
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();

 

};

