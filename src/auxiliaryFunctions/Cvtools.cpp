#include "Cvtools.h"

IplImage * Cvtools::readImage(string path) {
	IplImage *imgo = cvLoadImage(path.c_str(),1);
	return imgo;
}

IplImage * Cvtools::thresholdColor(IplImage * img) {
	CvSize size =cvGetSize(img);
	IplImage* res = cvCreateImage(size,img->depth,1);

	IplImage *upR= cvCreateImage(size,img->depth,1);
	IplImage * downR=cvCreateImage(size,img->depth,1);

	IplImage * upG=cvCreateImage(size,img->depth,1);
	IplImage * downG=cvCreateImage(size,img->depth,1);

	IplImage * upB=cvCreateImage(size,img->depth,1);
    IplImage * downB=cvCreateImage(size,img->depth,1);

	IplImage * Red = cvCreateImage(size,img->depth,1);
    IplImage * Blue = cvCreateImage(size,img->depth,1);
	IplImage * Green = cvCreateImage(size,img->depth,1);

	IplImage * RedSeg=cvCreateImage(size,img->depth,1);
    IplImage * GreenSeg=cvCreateImage(size,img->depth,1);
    IplImage * BlueSeg=cvCreateImage(size,img->depth,1);

	int threshold=70;
	int centerW= (int) size.width/2;
	int centerH= (int) size.height/2;
	CvScalar s = cvGet2D(img,centerH,centerW);

	cvSplit(img,Red,Green,Blue,NULL);

	cvThreshold(Red, upR, s.val[0]+threshold, 255, CV_THRESH_BINARY_INV);
	cvThreshold(Green, upG, s.val[1]+threshold, 255, CV_THRESH_BINARY_INV);
	cvThreshold(Blue, upB, s.val[2]+threshold, 255, CV_THRESH_BINARY_INV);

    cvThreshold(Red, downR, s.val[0]-threshold, 255, CV_THRESH_BINARY);
	cvThreshold(Green, downG, s.val[1]-threshold, 255, CV_THRESH_BINARY);
	cvThreshold(Blue, downB, s.val[2]-threshold, 255, CV_THRESH_BINARY);

	cvAnd(upR,downR,RedSeg,NULL);
	cvAnd(upG,downG,GreenSeg,NULL);
	cvAnd(upB,downB,BlueSeg,NULL);

	cvAnd(RedSeg,GreenSeg,res,NULL);
	cvAnd(res,BlueSeg,res,NULL);

    cvReleaseImage(&upR);
	cvReleaseImage(&upG);
	cvReleaseImage(&upB);
    cvReleaseImage(&downR);
    cvReleaseImage(&downG);
    cvReleaseImage(&downB);
    cvReleaseImage(&Red);
    cvReleaseImage(&Green);
    cvReleaseImage(&Blue);
    cvReleaseImage(&RedSeg);
    cvReleaseImage(&GreenSeg);
    cvReleaseImage(&BlueSeg);

	return res;
	
}

IplImage* Cvtools::binaryThresh(IplImage* img, IplImage* mask) {
CvSize size =cvGetSize(img);
IplImage * Red = cvCreateImage(size,img->depth,1);
IplImage * Blue = cvCreateImage(size,img->depth,1);
IplImage * Green = cvCreateImage(size,img->depth,1);
cvSplit(img,Red,Green,Blue,NULL);

cvAnd(Red,mask,Red,NULL);
cvAnd(Blue,mask,Blue,NULL);
cvAnd(Green,mask,Green,NULL);

IplImage* res = cvCreateImage(size,img->depth,3);
cvMerge(Red,Green,Blue,NULL,res);
cvReleaseImage(&Red);
cvReleaseImage(&Green);
cvReleaseImage(&Blue);
return res;

}

boolean Cvtools::checkTS(double TSLeft, double TSRight) {
    double diff=fabs(TSLeft-TSRight);
    if(diff <0.020)
        return true;
    else return false;

}
void Cvtools::saveStereoImage(const char * dir, IplImage* left, IplImage * right, int num) {
    char pathL[256];
    char pathR[256];
    preparePath(dir, pathL,pathR,num);
    
    printf("PathL: %s\n",pathL);
    printf("PathR: %s\n",pathR);

   cvSaveImage(pathL,left);
   cvSaveImage(pathR,right);
    
}

void Cvtools::preparePath(const char * dir, char* pathL, char* pathR, int count) {
            char num[5];
            sprintf(num, "%i", count); 


            strncpy(pathL,dir, strlen(dir));
			pathL[strlen(dir)]='\0';
            strcat(pathL,"left");
			strcat(pathL,num);
			strcat(pathL,".png");

            strncpy(pathR,dir, strlen(dir));
			pathR[strlen(dir)]='\0';
            strcat(pathR,"right");
			strcat(pathR,num);
			strcat(pathR,".png");
            
}

 void Cvtools::drawPoints(Mat& Img, vector<Point2f> Points) {

    for(int i=0; i<Points.size(); i++) 
        circle(Img,Points[i],2,cvScalar(255,0,0,0));

}