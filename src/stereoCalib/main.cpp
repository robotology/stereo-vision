#include "stereoCalibModule.h"


int main(int argc, char * argv[])
{
   /* initialize yarp network */ 

   Network yarp;

   /* create your module */

   stereoCalibModule stereoModule; 

   /* prepare and configure the resource finder */

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("stereoCalib.ini"); //overridden by --from parameter
   rf.setDefaultContext("stereoVision/conf");   //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */

   stereoModule.runModule(rf);


   /* Camera left;
    left.calibrate("C:/Users/Utente/Desktop/320/left_calib.xml",9,6);
    Camera right;
    right.calibrate("C:/Users/Utente/Desktop/320/right_calib.xml",9,6);
    stereoCamera stereo(left,right);
    stereo.stereoCalibration("C:/Users/Utente/Desktop/320/stereo_calib.xml",9,6);
    stereo.saveCalibration("C:/Users/Utente/Desktop/320/extrinsics.yml","C:/Users/Utente/Desktop/320/intrinsics.yml");*/

    /*stereoCamera stereo("C:/Users/Utente/Desktop/320/intrinsics.yml", "C:/Users/Utente/Desktop/320/extrinsics.yml");

    IplImage * leftim =cvLoadImage("C:/Users/Utente/Desktop/320/tleft.png",1);
    IplImage * rightim =cvLoadImage("C:/Users/Utente/Desktop/320/tright.png",1);

    int64 t = getTickCount();
    stereo.setImages(leftim,rightim);
    stereo.undistortImages();
    stereo.findMatch();
  //  cout << stereo.reprojectionErrorAvg() << endl;
    stereo.estimateEssential();
    stereo.essentialDecomposition();

    stereo.optimization();

  //  cout << stereo.reprojectionErrorAvg() << endl;
    stereo.computeDisparity();
    t = getTickCount() - t;
     printf("Time elapsed: %fms\n", t*1000/getTickFrequency());


   namedWindow("Test",1);

    int64 t = getTickCount();

     t = getTickCount() - t;
     printf("Time elapsed: %fms\n", t*1000/getTickFrequency());


    namedWindow("Left",1);
    namedWindow("Right",1);

    imshow("Left",stereo.getImLeft());
    imshow("Right",stereo.getImRight());

    namedWindow("LeftUnd",1);
    namedWindow("RightUnd",1);

    imshow("LeftUnd",stereo.getImLeftUnd());
    imshow("RightUnd",stereo.getImRightUnd());
     Mat imkey;

    namedWindow("Disparity",1);     
    imshow("Disparity",stereo.getDisparity());
    drawKeypoints(stereo.getImLeftGray(), stereo.getKeyPointsLeft(), imkey, Scalar(255,0,0,0));
    namedWindow("KeypointsL",1);
    imshow("KeypointsL",imkey);

    drawKeypoints(stereo.getImRightGray(), stereo.getKeyPointsRight(), imkey, Scalar(255,0,0,0));
    namedWindow("KeypointsR",1);
    imshow("KeypointsR",imkey);
    cvWaitKey(0);*/
    return 1;
}
