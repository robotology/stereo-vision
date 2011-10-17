#include "stereoCalibModule.h"
#include <yarp/os/Stamp.h>
#include <yarp/os/Os.h>



bool stereoCalibModule::configure(yarp::os::ResourceFinder &rf)
{    


    moduleName            = rf.check("name", 
                           Value("stereoCalib"), 
                           "module name (string)").asString();
   
    setName(moduleName.c_str());


    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();

 


    handlerPortName        = "/";
    handlerPortName       += getName(
                           rf.check("CommandPort", 
                           Value("/cmd"),
                           "Output image port (string)").asString()
                           );
    char dirName[255];
    bool proceed=true;


    string dir = rf.getContextPath().c_str();
    for (int i=1; proceed; i++)
        {
           sprintf(dirName,"%s/%s_%.5d",dir.c_str(),"calibImg",i);
		   proceed=!yarp::os::stat(dirName);
           sprintf(dirName,"%s/%s_%.5d/",dir.c_str(),"calibImg",i);
        }
    
    createFullPath(dirName);
    string outoutCalibPath=dirName;
    outoutCalibPath +="/";




    if (!handlerPort.open(handlerPortName.c_str())) {
      cout << ": unable to open port " << handlerPortName << endl;
      return false;
   }
   attach(handlerPort);

   myThread = new stereoCalibThread(rf,&handlerPort, dirName);



   myThread->start();

   return true;      
                      
}


bool stereoCalibModule::interruptModule()
{
   myThread->stop();


   return true;
}


bool stereoCalibModule::close()
{


   myThread->stop();
   delete myThread;

   return true;
}


bool stereoCalibModule::respond(const Bottle& command, Bottle& reply) 
{
   if (command.get(0).asString()=="start") {
        reply.addString("Show for 30 times the chess board pattern with different orientations and positions");
        myThread->startCalib();
        fprintf(stdout, "Show for 30 times the chess board pattern with different orientations and positions \n");
   }
  return true;
}

bool stereoCalibModule::updateModule()
{
   return true;
}



double stereoCalibModule::getPeriod()
{    
   return 0.1;
}

void stereoCalibModule::createFullPath(const char* path)
{
	if (yarp::os::stat(path))
    {
        string strPath=string(path);
        size_t found=strPath.find_last_of("/");
    
        while (strPath[found]=='/')
            found--;

        createFullPath(strPath.substr(0,found+1).c_str());
		yarp::os::mkdir(strPath.c_str());
    }
}
