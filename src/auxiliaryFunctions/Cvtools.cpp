#include "Cvtools.h"

IplImage * Cvtools::readImage(string path) {
	IplImage *imgo = cvLoadImage(path.c_str(),1);
	return imgo;
}

IplImage * Cvtools::thresholdColor(IplImage * img, int threshold) {

       
  //  cvSmooth(img, img, 1, 3, 0, 0, 0);

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


	int centerW= (int) size.width/2;
	int centerH= (int) size.height/2;
	CvScalar s = cvGet2D(img,centerH,centerW);

    double val=70;
    //s.val[0]=val;
    //s.val[1]=val;
    //s.val[2]=val;

    if(s.val[0]==0) {
        s.val[0]=val;
        s.val[1]=val;
        s.val[2]=val;
    }

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

void Cvtools::binaryThresh(IplImage* img, IplImage* mask, IplImage * res) {
CvSize size =cvGetSize(img);
IplImage * Red = cvCreateImage(size,img->depth,1);
IplImage * Blue = cvCreateImage(size,img->depth,1);
IplImage * Green = cvCreateImage(size,img->depth,1);
cvSplit(img,Red,Green,Blue,NULL);

cvAnd(Red,mask,Red,NULL);
cvAnd(Blue,mask,Blue,NULL);
cvAnd(Green,mask,Green,NULL);

cvZero(res);
cvMerge(Red,Green,Blue,NULL,res);
cvReleaseImage(&Red);
cvReleaseImage(&Green);
cvReleaseImage(&Blue);
return;

}

bool Cvtools::checkTS(double TSLeft, double TSRight) {
    double diff=fabs(TSLeft-TSRight);
    if(diff <0.020)
        return true;
    else return false;

}
 void Cvtools::saveImgSegDisp(const char * dir, IplImage * img, IplImage* foreground, double norm, int num) {

    char path1[256];
    char path2[256];
    char path3[256];

    preparePath(dir,path1,path2,path3,num);

    cvSaveImage(path1,img);
    cvSaveImage(path2,foreground);

    ofstream myfile;
    myfile.open(path3,ios_base::app);
    myfile << norm << endl;
    myfile.close();


}
void Cvtools::saveStereoImage(const char * dir, IplImage* left, IplImage * right, int num) {
    char pathL[256];
    char pathR[256];
    preparePath(dir, pathL,pathR,num);
    
    printf("Saving images number %d\n",num);

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

void Cvtools::preparePath(const char * dir, char* path1, char* path2, char * path3, int count) {
            char num[5];
            sprintf(num, "%i", count); 
            strncpy(path1,dir, strlen(dir));
			path1[strlen(dir)]='\0';
            strcat(path1,"image");
			strcat(path1,num);
			strcat(path1,".png");

            strncpy(path2,dir, strlen(dir));
			path2[strlen(dir)]='\0';
            strcat(path2,"foreground");
			strcat(path2,num);
			strcat(path2,".png");
            
            strncpy(path3,dir, strlen(dir));
			path3[strlen(dir)]='\0';
            strcat(path3,"distance");
			strcat(path3,".txt");
}

 void Cvtools::drawPoints(Mat& Img, vector<Point2f> Points) {

    for(int i=0; i<Points.size(); i++) 
        circle(Img,Points[i],2,cvScalar(255,0,0,0));

}


void Cvtools::computeContrastandOrientation(IplImage* img, IplImage* arctan, IplImage* contrast) {
	double max =0;
	double min =0;
    IplImage *img_t = cvCreateImage(cvGetSize(img),img->depth,1);
	IplImage *img_f = cvCreateImage(cvGetSize(img),32,1);

	IplImage * derivativeX= cvCreateImage(cvGetSize(img),32,1);
	IplImage * derivativeY= cvCreateImage(cvGetSize(img),32,1);
    if(img->nChannels>1)
        cvCvtColor(img,img_t,CV_RGB2GRAY);
    else
        cvCopy(img,img_t,0);
    cvMinMaxLoc(img_t,&min,&max,NULL,NULL,NULL);
	cvConvertScale(img_t,img_f,1.0/max,0);

	cvSobel(img_f,derivativeX,1,0,1);
	cvSobel(img_f,derivativeY,0,1,1);

	cvZero(arctan);
	cvZero(contrast);
	cvCartToPolar(derivativeX, derivativeY, contrast, arctan, 0);
	//cvThreshold(contrast,contrast,0.7,1,CV_THRESH_BINARY); // puoi thresholdare sul contrasto se vuoi


	cvReleaseImage(&img_f);
	cvReleaseImage(&derivativeX);
	cvReleaseImage(&derivativeY);
    cvReleaseImage(&img_t);

}

void Cvtools::computeHOG(IplImage* image, CvHistogram* histTemp) {
	IplImage * contrastTemplate= cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage * arctanTemplate= cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage * maskTemplate= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	computeContrastandOrientation(image,arctanTemplate,contrastTemplate);


	cvConvertScale(contrastTemplate,maskTemplate,255,0);
    cvNamedWindow("Contrasto",1);
	cvShowImage("Contrasto", maskTemplate);
	cvCalcHist(&arctanTemplate, histTemp, 0, maskTemplate);
	cvNormalizeHist(histTemp,1);
	stampaIstogrammi1D(histTemp, 36,  25, "Hist Temp"); // per visualizzazione

	cvReleaseImage(&contrastTemplate);
	cvReleaseImage(&arctanTemplate);
	cvReleaseImage(&maskTemplate);
}

void Cvtools::getContour(IplImage* mask, IplImage* maschera) {

	cvMorphologyEx(maschera, maschera, 0, 0, CV_MOP_OPEN, 1);

	CvMemStorage* storage=cvCreateMemStorage();
	CvSeq* contour = NULL;

	IplImage* temp=(IplImage*)cvClone(maschera);

	CvContourScanner sc = cvStartFindContours(temp, storage, sizeof(CvContour), CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE );

	contour = cvFindNextContour(sc);
	while (contour!=NULL) {
		CvRect rect = cvBoundingRect(contour, 0);
		if (rect.x<160&&rect.y<120&&(rect.x+rect.width)>160&&(rect.y+rect.height)>120) {
			break;
		}
    contour = cvFindNextContour(sc);

	} 


	if (contour==NULL) {
		cvReleaseImage(&temp);
		mask=maschera;
		cvReleaseMemStorage(&storage);
		return;
	}

	cvSetZero(mask);
	cvDrawContours(mask, contour, cvScalar(255, 0, 0, 0), cvScalar(0, 0, 0, 0), 1, CV_FILLED, 8, cvPoint(0, 0));
	cvAnd(mask,maschera,mask);
	cvEndFindContours(&sc);
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&temp);
}


void Cvtools::stampaIstogrammi1D(CvHistogram* hist, int n_bins, int scale, char* nameWindow) {

	IplImage* hist_img=cvCreateImage(cvSize(n_bins*scale, 255), 8, 3);
	hist_img->origin=IPL_ORIGIN_BL;
	float max_value=0;
	cvGetMinMaxHistValue(hist, 0, &max_value, 0, 0);
	int i=0;
	for (i=0; i<n_bins; i++) {
		float bin_value=cvQueryHistValue_1D(hist, i);
		int normalizzo=cvRound(bin_value*255/max_value);
		CvPoint point1=cvPoint(i*scale, 0);
		CvPoint point2=cvPoint((i+1)*scale, normalizzo);
		cvRectangle(hist_img, point1, point2, cvScalar(255, 0, 0, 0), -1, 8, 0);
	}
    cvNamedWindow(nameWindow,1);
	cvShowImage(nameWindow, hist_img);
	cvReleaseImage(&hist_img);

}
