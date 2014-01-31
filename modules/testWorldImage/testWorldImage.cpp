#include "testWorldImage.h"



bool TestWorldImage::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("testWorldImage")).asString().c_str();
    string pixel=rf.check("pixelPort",Value("/pixelPort")).asString().c_str();
    string imWorld=rf.check("worldPort",Value("/world:i")).asString().c_str();

    pixelPort.open(pixel.c_str());
    imageWorld.open(imWorld.c_str());

    return true;
}

bool TestWorldImage::close()
{
    
    imageWorld.interrupt();
    imageWorld.close();

    pixelPort.interrupt();
    pixelPort.close();

    return true;
}

bool TestWorldImage::interruptModule()
{
    imageWorld.interrupt();
    imageWorld.close();
    pixelPort.interrupt();
    pixelPort.close();
    return true;
}
bool TestWorldImage::updateModule()
{
    Bottle* b=NULL;
    ImageOf<PixelRgbFloat> *imgYarp=NULL;

    if(pixelPort.getInputCount()>0 && imageWorld.getInputCount()>0)
    {
        b=pixelPort.read(true);
        imgYarp= imageWorld.read(true);
    }

    if(b==NULL || imgYarp==NULL)
        return true;

    int u,v=0;

    u=b->get(0).asInt();
    v=b->get(1).asInt();

    worldImg=(IplImage*) imgYarp->getIplImage();

    if(u<0 || v<0 || u>worldImg->width || v>worldImg->height)
    {
        fprintf(stdout, "Non Valid Pixel Coordinates \n");
        return true;
    }

    float X=((float *)(worldImg->imageData + v*worldImg->widthStep))[u*worldImg->nChannels + 0];
    float Y=((float *)(worldImg->imageData + v*worldImg->widthStep))[u*worldImg->nChannels + 1];
    float Z=((float *)(worldImg->imageData + v*worldImg->widthStep))[u*worldImg->nChannels + 2];

    fprintf(stdout, "u: %d v: %d X: %f Y: %f Z %f \n",u,v,X,Y,Z);
    return true;
}

double TestWorldImage::getPeriod()
{
    return 0.1;
}





int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("testImageWorld");
    rf.configure(argc,argv);

    TestWorldImage mod;

    return mod.runModule(rf);
}

