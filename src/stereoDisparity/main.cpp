#include <yarp/dev/Drivers.h>
#include "disparityModule.h"

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char * argv[])
{

	YARP_REGISTER_DEVICES(icubmod)
   /* initialize yarp network */ 

   Network yarp;

   // create your module 

   stereoModule stereoModule; 

   // prepare and configure the resource finder 

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("Disparity.ini"); //overridden by --from parameter
   rf.setDefaultContext("stereoVision/conf");   //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   // run the module: runModule() calls configure first and, if successful, it then runs 

   stereoModule.runModule(rf);

    return 0;
}
