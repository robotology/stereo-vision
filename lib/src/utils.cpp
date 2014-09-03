/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>
#include <yarp/os/Stamp.h>
#include <cv.h>
#include <highgui.h>

#include <iCub/stereoVision/utils.h>


using namespace std;
using namespace cv;

/************************************************************************/
Utilities::Utilities()
{
}

/************************************************************************/
Utilities::~Utilities()
{
}
/************************************************************************/
void Utilities::initSIFT_GPU()
{
    SiftGPU* (*pCreateNewSiftGPU)(int) = NULL;
    SiftMatchGPU* (*pCreateNewSiftMatchGPU)(int) = NULL;

    char * pPath;
    pPath = getenv ("SIFTGPU_DIR");
    printf ("\n\nThe current path is: %s\n\n",pPath);

    std::string str = pPath;
    str.append("/bin/libsiftgpu.so");
    hsiftgpu = dlopen(str.c_str(), RTLD_LAZY);

    pCreateNewSiftGPU = (SiftGPU* (*) (int)) GET_MYPROC(hsiftgpu, "CreateNewSiftGPU");
    pCreateNewSiftMatchGPU = (SiftMatchGPU* (*)(int)) GET_MYPROC(hsiftgpu, "CreateNewSiftMatchGPU");
    sift = pCreateNewSiftGPU(1);
    matcher = pCreateNewSiftMatchGPU(4096);


    char * argv[] = {(char*)"-fo", (char*)"-1", (char*) "-v",(char*) "1", (char*)"-winpos",(char*)"-maxd", (char*)"1024", (char*)"-cuda"};
    int argc = sizeof(argv)/sizeof(char*);

    sift->ParseParam(argc, argv);
    //verbose on sift to remove unwanted printouts (put 1 for data)
    sift->SetVerbose(0);

    if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        fprintf(stdout,"boh, some error\n");

    matcher->VerifyContextGL();
    writeS=false;
}

/************************************************************************/
void Utilities::extractMatch_GPU(Mat &leftMat, Mat &rightMat, Mat &matMatches,
                                 double displacement)
{
    sift->RunSIFT( leftMat.cols, leftMat.rows, leftMat.data, GL_RGB, GL_UNSIGNED_BYTE );
    num1 = sift->GetFeatureNum();
    keys1.resize(num1); descriptors1.resize(128*num1);
    sift->GetFeatureVector(&keys1[0], &descriptors1[0]);

    sift->RunSIFT( rightMat.cols, rightMat.rows, rightMat.data, GL_RGB, GL_UNSIGNED_BYTE );
    num2 = sift->GetFeatureNum();
    keys2.resize(num2); descriptors2.resize(128*num2);
    sift->GetFeatureVector(&keys2[0], &descriptors2[0]);

    matcher->SetDescriptors(0, num1, &descriptors1[0]);
    matcher->SetDescriptors(1, num2, &descriptors2[0]);

    int (*match_buf)[2] = new int[num1][2];
    int num_match = matcher->GetSiftMatch(num1, match_buf);
    //fprintf(stdout, "%d SIFT matches were found ", num_match); 

    pointsL.clear();
    pointsR.clear();
    //enumerate all the feature matches
    for (int i=0; i<num_match; i++)
    {
        SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];
        SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];
        //key1 in the first image matches with key2 in the second image
        if (fabs(key1.y-key2.y)<displacement)
        {
	    double x=key1.x;
	    double y=key1.y;
	    circle(matMatches,cvPoint(x,y),2,cvScalar(255,0,0),2);

	    double x2=leftMat.cols+key2.x;
	    double y2=key2.y;
	    circle(matMatches,cvPoint(x2,y2),2,cvScalar(255,0,0),2);
	    line(matMatches, cvPoint(x,y),cvPoint(x2,y2),cvScalar(255,255,255));
	    
	    Point2f p1(x,y);
	    Point2f p2(x2-leftMat.cols,y2);
	    
	    pointsL.push_back(p1);
	    pointsR.push_back(p2);
        }
    } 
}

/************************************************************************/
void Utilities::extractMatch_GPU(Mat &leftMat, Mat &rightMat, double displacement)
{
    sift->RunSIFT( leftMat.cols, leftMat.rows, leftMat.data, GL_RGB, GL_UNSIGNED_BYTE );
    num1 = sift->GetFeatureNum();
    keys1.resize(num1); descriptors1.resize(128*num1);
    sift->GetFeatureVector(&keys1[0], &descriptors1[0]);

    sift->RunSIFT( rightMat.cols, rightMat.rows, rightMat.data, GL_RGB, GL_UNSIGNED_BYTE );
    num2 = sift->GetFeatureNum();
    keys2.resize(num2); descriptors2.resize(128*num2);
    sift->GetFeatureVector(&keys2[0], &descriptors2[0]);

    matcher->SetDescriptors(0, num1, &descriptors1[0]);
    matcher->SetDescriptors(1, num2, &descriptors2[0]);

    int (*match_buf)[2] = new int[num1][2];
    int num_match = matcher->GetSiftMatch(num1, match_buf);
    //fprintf(stdout, "%d SIFT matches were found ", num_match); 
   
    pointsL.clear();
    pointsR.clear();
    //enumerate all the feature matches
    for(int i  = 0; i < num_match; ++i)
    {
        SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];
        SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];
        //key1 in the first image matches with key2 in the second image
        if(fabs(key1.y-key2.y)<displacement)
        {
            double x=key1.x;
            double y=key1.y;

            double x2=leftMat.cols+key2.x;
            double y2=key2.y;
                    
            Point2f p1(x,y);
            Point2f p2(x2-leftMat.cols,y2);
                    
            pointsL.push_back(p1);
            pointsR.push_back(p2);
        }
    } 
}

void Utilities::getMatches(std::vector<cv::Point2f> & points1, std::vector<cv::Point2f>  & points2)
{
    points1=pointsL;
    points2=pointsR;
}


void Utilities::writeSIFTs(std::string filePath, std::vector<float> &des, std::vector<SiftGPU::SiftKeypoint>  &points)
{
    

    string line;
    ofstream infile;
    infile.open (filePath.c_str());
   
    int cnt=0; 
    for (int i=0; i<points.size(); i++)
    {
      infile << points[i].x << " " << points[i].y << " ";
      for (int k=0; k<128; k++)
      {
         infile << des[cnt] << " ";
         cnt++;
      }
      infile << endl;
    
    }
    infile.close();

}

void Utilities::writeMatch(std::string filePath,std::vector<cv::Point2f>  &pointsL, std::vector<cv::Point2f>  &pointsR)
{
    

    string line;
    ofstream infile;
    infile.open (filePath.c_str());
   
    int cnt=0; 
    for (int i=0; i<pointsL.size(); i++)
    {
      infile << pointsL[i].x << " " << pointsL[i].y << " " << pointsR[i].x << " " << pointsR[i].y ;
      
      infile << endl;
    
    }
    infile.close();

}
