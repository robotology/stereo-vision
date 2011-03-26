#include "stereoCamera.h"

void printMatrix(Mat &matrix) {
    int row=matrix.rows;
    int col =matrix.cols;
        cout << endl;
   /* for(int i=0; i<row; i++) {
        for(int j=0; j<col; j++) {
            cout << matrix(Range(i,i),Range(j,j));
        }
        cout << endl;
    }*/
    for(int i = 0; i < matrix.rows; i++)
    {
        const double* Mi = matrix.ptr<double>(i);
        for(int j = 0; j < matrix.cols; j++)
            cout << Mi[j] << " ";
        cout << endl;
    }
        cout << endl;
}

Point3f triangulation(Point2f& pointleft, Point2f& pointRight, Mat Camera1, Mat Camera2) {

      Point3f point3D;
      Mat J=Mat(4,4,CV_64FC1);
      for(int j=0; j<4; j++) {

            int rowA=0;
            int rowB=2;

            J.at<double>(0,j)=(pointleft.x*Camera1.at<double>(rowB,j))- (Camera1.at<double>(rowA,j));
            J.at<double>(2,j)=(pointRight.x*Camera2.at<double>(rowB,j))- (Camera2.at<double>(rowA,j));

            rowA=1;
            
            J.at<double>(1,j)=(pointleft.y*Camera1.at<double>(rowB,j))- (Camera1.at<double>(rowA,j));
            J.at<double>(3,j)=(pointRight.y*Camera2.at<double>(rowB,j))- (Camera2.at<double>(rowA,j));
        }
        SVD decom(J);
        Mat V= decom.vt;

        point3D.x=V.at<double>(3,0)/V.at<double>(3,3);
        point3D.y=V.at<double>(3,1)/V.at<double>(3,3);     
        point3D.z=V.at<double>(3,2)/V.at<double>(3,3);
        return point3D;

}


void reprojection(Mat& Rot, Mat& Tras, Mat Kleft, Mat Kright, vector<Point2f> PointsL, vector<Point2f> PointsR, double * err) {

    Point2f pointL, pointR;
    vector<Point3f> WorldPoints;
        Mat A = Mat::eye(3, 4, CV_64F);
        Mat P1=Kleft*A;

        for(int i = 0; i < Rot.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = Rot.ptr<double>(i);
            for(int j = 0; j < Rot.cols; j++)
                 Mi[j]=MRi[j];
         }
        for(int i = 0; i < Tras.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = Tras.ptr<double>(i);
                 Mi[3]=MRi[0];
         }
        Mat P2=Kright*A;
        Point3f point3D;


     for(int i =0; i<PointsL.size(); i++) {

        pointL=PointsL[i];
        pointR=PointsR[i];

        point3D=triangulation(pointL,pointR,P1,P2);
        WorldPoints.push_back(point3D);

    }

    vector<Point2f> reprojectionL, reprojectionR;

    projectPoints(Mat(WorldPoints), Mat::eye(3,3,CV_64FC1), Mat::zeros(3,1,CV_64FC1), Kleft, Mat(), reprojectionL);

    projectPoints(Mat(WorldPoints), Rot, Tras, Kright, Mat(), reprojectionR);
    int j=0;
    for(int i =0; i<WorldPoints.size(); i++) {
        err[j]=reprojectionL[i].x;
        err[j+1]=reprojectionL[i].y;
        err[j+2*WorldPoints.size()]=reprojectionR[i].x;
        err[j+2*WorldPoints.size()+1]=reprojectionR[i].y;
        j=j+2;
        
    }
 /*j=0;
    for (int i=0; i<WorldPoints.size(); i++) {
        cout << "XL: " << err[j] << " " << reprojectionL[i].x << " YL: " << err[j+1] << " " << reprojectionL[i].y << endl;
        cout << "XR: " << err[j+2*WorldPoints.size()] << " " << reprojectionR[i].x << " YL: " << err[j+1+2*WorldPoints.size()] << " " << reprojectionR[i].y << endl;
        j=j+2;
   
    }*/



}


void getSolutionfromVariables(Mat& Rot, Mat& Tras, vector<Point3f>& WorldPoints, double * vars) {
    Rot=Mat(3,3,CV_64FC1);
    Mat angles(3,1,CV_64FC1);
    Tras=Mat(3,1,CV_64FC1);

    for(int i=0; i<3; i++) {
        angles.at<double>(i,0)=vars[i];
    }

    for(int i=0; i<3; i++) {
        Tras.at<double>(i,0)=vars[i+3];
    }
    Rodrigues(angles,Rot);

    int j=6;
    for(int i=0; i<WorldPoints.size(); i++) {
     
        WorldPoints[i].x=vars[j+i];
        j++;
        WorldPoints[i].y=vars[j+i];
        j++;
        WorldPoints[i].z=vars[j+i];
    }


}

void funcMeas(double *p, double *x, int m, int n, void *data) {
// p -> 6 parametri da minimizzare
 // x -> 4*npoints output riproiezione

    stereoCamera * camera = (stereoCamera*) data;
    Mat Rot;
    Mat Tras;
    
    getSolutionfromVariables(Rot,Tras, vector<Point3f>(), p);
    if(m==3)
        Tras=camera->getTranslation();


    reprojection(Rot,Tras,camera->getKleft(), camera->getKright(), camera->getMatchLeft(), camera->getMatchRight(),x);


}
const Mat stereoCamera::getKleft() {
    return this->Kleft;
}
const Mat stereoCamera::getKright() {
    return this->Kright;
}

const vector<Point2f> stereoCamera::getMatchLeft() {
    return this->InliersL;
}
const vector<Point2f>  stereoCamera::getMatchRight() {

    return this->InliersR;
}

stereoCamera::stereoCamera(std::string intrinsicPath, std::string exstrinsicPath) {

     FileStorage fs(intrinsicPath.c_str(), CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsicPath.c_str());
            return;
        }
        fs["M1"] >> Kleft;
        fs["D1"] >> DistL;
        fs["M2"] >> Kright;
        fs["D2"] >> DistR;

        fs.open(exstrinsicPath.c_str(), CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", exstrinsicPath.c_str());
            return ;
        }
        fs["R"] >> R;
        fs["T"] >> T;
        fs["Q"] >> Q;
        Mat A = Mat::eye(3, 4, CV_64F);
        Pleft=Kleft*A;

        for(int i = 0; i < R.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = R.ptr<double>(i);
            for(int j = 0; j < R.cols; j++)
                 Mi[j]=MRi[j];
         }
        for(int i = 0; i < T.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = T.ptr<double>(i);
                 Mi[3]=MRi[0];
         }
        Pright=Kright*A;




}

stereoCamera::stereoCamera(Camera Left, Camera Right) {
    this->Kleft=Left.getIntrinsic();
    this->DistL=Left.getDistVector();

    this->Kright=Right.getIntrinsic();
    this->DistR=Right.getDistVector();
}

void stereoCamera::setImages(IplImage * left, IplImage * right) {
       this->imleft=left;
       this->imright=right;
}


void stereoCamera::printStereoIntrinsic() {
    if(this->Kleft.empty())
        printf("Stereo Cameras are not calibrated, run calibration method before..\n");
    else 
    {
     printf("Left Intrinsic Parameters: \n");
     printMatrix(this->Kleft);
     printf("Right Intrinsic Parameters: \n");
     printMatrix(this->Kright);
    }
}

void stereoCamera::stereoCalibration(vector<string> imagelist, int boardWidth, int boardHeight) {
    Size boardSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;
    runStereoCalib(imagelist, boardSize);

}

void stereoCamera::stereoCalibration(string imagesFilePath, int boardWidth, int boardHeight) {
    Size boardSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;

    vector<string> imagelist;
    bool ok = readStringList(imagesFilePath, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << imagesFilePath << " or the string list is empty" << endl;
        return;
    }
    runStereoCalib(imagelist, boardSize);




}

bool stereoCamera::readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}
    
void stereoCamera::runStereoCalib(const vector<string>& imagelist, Size boardSize)
{
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }
    
    bool displayCorners = true;
    const int maxScale = 2;
    const float squareSize = 3.f;  // Set this to your actual square size
    // ARRAY AND VECTOR STORAGE:
    
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;
    
    int i, j, k, nimages = (int)imagelist.size()/2;
    
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;
    
    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners, 
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, CV_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                imshow("corners", cimg);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
           // if(imageSize.width==640)          
          //      cornerSubPix(img, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }
    
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }
    
    cout << "Running stereo calibration ...\n";
    
    Mat cameraMatrix[2], distCoeffs[2];
    Mat E, F;
    
    if(this->Kleft.empty() || this->Kleft.empty())
    {
        double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                        this->Kleft, this->DistL,
                        this->Kright, this->DistR,
                        imageSize, this->R, this->T, E, F,
                        TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                        CV_CALIB_FIX_ASPECT_RATIO +
                        CV_CALIB_ZERO_TANGENT_DIST +
                        CV_CALIB_SAME_FOCAL_LENGTH +
                        CV_CALIB_RATIONAL_MODEL +
                        CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
        cout << "done with RMS error=" << rms << endl;
    } else
    {
        double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                this->Kleft, this->DistL,
                this->Kright, this->DistR,
                imageSize, this->R, this->T, E, F,
                TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),CV_CALIB_FIX_INTRINSIC);
        cout << "done with RMS error=" << rms << endl;
    }
// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    cameraMatrix[0] = this->Kleft;
    cameraMatrix[1] = this->Kright;
    distCoeffs[0]=this->DistL;
    distCoeffs[1]=this->DistR;
    Mat R, T;
    T=this->T;
    R=this->R;
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
   Mat R1,R2,P1,P2;
   Rect roi1, roi2;
   stereoRectify( this->Kleft, this->DistL, this->Kright, this->DistR, imageSize, this->R, this->T, R1, R2, P1, P2, this->Q, -1, imageSize, &roi1, &roi2 );
    cout << "average reprojection err = " <<  err/npoints << endl;
}

void stereoCamera::saveCalibration(string extrinsicFilePath, string intrinsicFilePath) {
// save intrinsic parameters
    FileStorage fs(intrinsicFilePath, CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << Kleft << "D1" << DistL << "M2" << Kright << "D2" << DistR;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

     fs.open(extrinsicFilePath, CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T <<"Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

}



const Mat stereoCamera::getImLeft() {

    return this->imleft;
}

const Mat stereoCamera::getImRight() {
    return this->imright;
}

const Mat stereoCamera::getDisparity() {
    return this->Disparity;
}
const Mat stereoCamera::getQ() {
    return this->Q;
}
void stereoCamera::computeDisparity() {
    if(this->Kleft.empty() || this->DistL.empty() || this->Kright.empty() || this->DistR.empty()) {
        cout <<" Cameras are not calibrated! Run the Calibration first!" << endl;
        return;
    }
    if(this->imleft.empty() || this->imright.empty()) {
          cout << "Images are not set! set the images first!" << endl;
          return;
    }
        enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2 };
        int alg = STEREO_SGBM;
        int SADWindowSize = 0, numberOfDisparities = 0;
        int color_mode = alg == STEREO_BM ? 0 : -1;
        Size img_size = this->imleft.size();
        Mat R1, P1, R2, P2;
        Rect roi1, roi2;
        Mat Q;

       // StereoBM bm;
        StereoSGBM sgbm;
        stereoRectify( this->Kleft, this->DistL, this->Kright, this->DistR, img_size, this->R, this->T, R1, R2, P1, P2, Q, -1, img_size, &roi1, &roi2 );


        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(this->Kleft, this->DistL, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(this->Kright,  this->DistR, R2, P2, img_size, CV_16SC2, map21, map22);
        
        Mat img1r, img2r;
        remap(this->imleft, img1r, map11, map12, INTER_LINEAR);
        remap(this->imright, img2r, map21, map22, INTER_LINEAR);

        /*namedWindow("RectL",1);
        namedWindow("RectR",1);
        imshow("RectL",img1r);
        imshow("RectR",img2r);
      cvWaitKey(15);*/

        if(numberOfDisparities > 0)
            numberOfDisparities = numberOfDisparities;
        else if(img_size.width==640)
            numberOfDisparities= img_size.width/8;
        else  
            numberOfDisparities= img_size.width/8 +8;
    
        numberOfDisparities=64;
       /* bm.state->roi1 = roi1;
        bm.state->roi2 = roi2;
        bm.state->preFilterCap = 31;
        bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
        bm.state->minDisparity = 0;
        bm.state->numberOfDisparities = numberOfDisparities;
        bm.state->textureThreshold = 10;
        bm.state->uniquenessRatio = 15;
        bm.state->speckleWindowSize = 100;
        bm.state->speckleRange = 32;
        bm.state->disp12MaxDiff = 1;*/
        
        sgbm.preFilterCap = 63; //63
        sgbm.SADWindowSize = 1; //SADWindowSize > 0 ? SADWindowSize : 1;
        
        int cn = this->imleft.channels();
        
        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity =-15;
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = 22; //10
        sgbm.speckleWindowSize = 50; //100
        sgbm.speckleRange = 16; //32
        sgbm.disp12MaxDiff = 1;
        sgbm.fullDP = 1; // alg == STEREO_HH
       
        
        Mat disp, disp8;


        sgbm(img1r, img2r, disp);


        //disp = dispp.colRange(numberOfDisparities, img1p.cols);
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        this->Disparity=disp8;        

}


void stereoCamera::findMatch() {

    if(this->imleftund.empty() || this->imrightund.empty()) {
          cout << "Images are not set and undistorted! set the images first and call undistortImages()!" << endl;
          return;
    }
    int region=100;
    int off=15;
    vector<DMatch> matches; // Match matches[i].QueryIdx=keypointLeft, matches[i].trainIdx=keypointRight. QueryIdx=TrainIdx=0 -> No Match
    this->PointsL.clear();
    this->PointsR.clear();
    
    this->InliersL.clear();
    this->InliersR.clear();

    SURF FeaturesExtractionSurf;

    bool draw=false;

    Size dim =imleft.size();


    Rect roiL((dim.width/2)-region,(dim.height/2)-region,2*region,2*region);
    Mat maskL(dim.height,dim.width,CV_8U);
    maskL.setTo(Scalar(0,0,0,0));
    maskL(roiL).setTo(Scalar(255,255,255,255));

    Rect roiR((dim.width/2)-(region+off),(dim.height/2)-(region),2*region,2*region);
    Mat maskR(dim.height,dim.width,CV_8U);
    maskR.setTo(Scalar(0,0,0,0));
    maskR(roiR).setTo(Scalar(255,255,255,255));

    cvtColor(this->imleftund,this->imleftgray,CV_BGR2GRAY,1);
    cvtColor(this->imrightund,this->imrightgray,CV_BGR2GRAY,1);

    vector<float> descL;
    vector<float> descR;
    
    vector<KeyPoint> keypointsL; // Features on Left Image
    vector<KeyPoint> keypointsR; // Features on Right Image
    FeaturesExtractionSurf(this->imleftgray,maskL,keypointsL,descL);
    FeaturesExtractionSurf(this->imrightgray,maskR,keypointsR,descR);
    
    if(draw)
         matches.resize(keypointsL.size(),DMatch(0,0,0));

    Mat MatdescL=Mat(descL).reshape(1,keypointsL.size());
    Mat MatdescR=Mat(descR).reshape(1,keypointsR.size());

    getMatch(MatdescL,keypointsL,MatdescR,keypointsR,matches,draw);


    if(draw) {
        Mat matchImg;
        vector<char> matchMask(keypointsL.size(),1);
        for (int i=0; i<matches.size(); i++) {
            if(matches[i].queryIdx==0 || matches[i].trainIdx==0) {
                matchMask[i]=0;
            }
        }
         
        drawMatches(this->imleftund, keypointsL, this->imrightund, keypointsR,matches,matchImg,Scalar(0,0,255,0), Scalar(0,0,255,0),matchMask,2);
        namedWindow("Match",1);
        imshow("Match",matchImg); 
        cvWaitKey(15);
    }
    

}

/*
void stereoCamera::findMatch() {
    if(this->imleftund.empty() || this->imrightund.empty()) {
          cout << "Images are not set and undistorted! set the images first and call undistortImages()!" << endl;
          return;
    }
    bool draw=true;
    this->PointsL.clear();
    this->PointsR.clear();
    double Ymax=4;

    vector<Point2f> cornerL, cornerR;
    vector<uchar> status;
    vector <float> diff;
    cvtColor(this->imleftund,this->imleftgray,CV_BGR2GRAY,1);
    cvtColor(this->imrightund,this->imrightgray,CV_BGR2GRAY,1);

    goodFeaturesToTrack(this->imleftgray,cornerL,1000,0.001, 15);
    calcOpticalFlowPyrLK(this->imleftgray,this->imrightgray,cornerL,cornerR,status,diff,Size(3,3),2);

    for(int i=0; i< cornerL.size(); i++) {
        if(status[i]==1 && (abs(cornerL[i].y-cornerR[i].y))<Ymax) {
            PointsL.push_back(cornerL[i]);
            PointsR.push_back(cornerR[i]);
        }
    }
    
    Cvtools::drawPoints((Mat)this->getImLeft(),PointsL);
    Cvtools::drawPoints((Mat)this->getImRight(),PointsR);

}*/
const Mat stereoCamera::getImLeftGray() {
    return this->imleftgray;
}

const Mat stereoCamera::getImRightGray() {
    return this->imrightgray;
}


void stereoCamera::getMatch(const Mat& descL, const vector<KeyPoint>& keypointsL, const Mat& descR, const  vector<KeyPoint>& keypointsR,  vector<DMatch>& matches, bool draw) {
    int sizeL=keypointsL.size();
    int sizeR=keypointsR.size();
    int pixelsX=50; //80
    int pixelsXMin=-200; //25
    int pixelsY=5; // 5

    
    for (int i=0; i<sizeL; i++) {

        const float *dL=descL.ptr<float>(i);
        int index1=getBestDistance(dL,descR);
        
        const float *dR=descR.ptr<float>(index1);
        int index2=getBestDistance(dR,descL);
     
        if(i!=index2){

            continue;
        }
           
        double X=keypointsL[i].pt.x-keypointsR[index1].pt.x;
        double maxY=abs(keypointsL[i].pt.y-keypointsR[index1].pt.y);
        bool cond=X> pixelsX || maxY> pixelsY || X< pixelsXMin;
 
        if(cond) {
            continue;
        }
        if(draw) {
           matches[i].queryIdx=i;
           matches[i].trainIdx=index1;
        }
           Point2f pointL=keypointsL[i].pt;
           Point2f pointR=keypointsR[index1].pt;
           this->PointsR.push_back(pointR);
           this->PointsL.push_back(pointL);
    }

}

int stereoCamera::getBestDistance(const float *di, const Mat& allDesc) {
    double min=10000;
    int index=-1;
    double dist=0;


    for(int i=0; i<allDesc.rows; i++) 
    {
        const float* Mi = allDesc.ptr<float>(i);

        for(int j = 0; j < allDesc.cols; j++) {
            dist=dist+pow(di[j]-Mi[j],2);

        }
        dist=sqrt(dist);
        if(dist<min) {
            min=dist;
            index=i;
        }
        dist=0;
    }



return index;

}

double stereoCamera::reprojectionErrorAvg() {
    if(PointsL.empty() || PointsR.empty()) {
        cout << "No Matches Found" << endl;
        return -1;
    }

    if(InliersL.empty()) {
        InliersL=PointsL;
        InliersR=PointsR;

    }
    double errAvg=0;

    Point2f pointL, pointR;

    vector<Point3f> WorldPoints;
    Mat J=Mat(4,4,CV_64FC1);
    Point3f point3D;

    for(int i =0; i<this->InliersL.size(); i++) {

        pointL=InliersL[i];
        pointR=InliersR[i];

        point3D=triangulation(pointL,pointR,this->Pleft,this->Pright);

        WorldPoints.push_back(point3D);

    }
    vector<Point2f> reprojection;

    projectPoints(Mat(WorldPoints), Mat::eye(3,3,CV_64FC1), Mat::zeros(3,1,CV_64FC1), this->Kleft, Mat(), reprojection);
    double errorLeft = norm(Mat(InliersL), Mat(reprojection), CV_L2)/InliersL.size();

    projectPoints(Mat(WorldPoints), this->R, this->T, this->Kright, Mat(), reprojection);
    double errorRight = norm(Mat(InliersR), Mat(reprojection), CV_L2)/InliersR.size();


    errAvg=(errorLeft+errorRight)/2;
    return errAvg;

}


Point3f stereoCamera::triangulation(Point2f& pointleft, Point2f& pointRight) {
      
      Point3f point3D;
      Mat J=Mat(4,4,CV_64FC1);
      for(int j=0; j<4; j++) {

            int rowA=0;
            int rowB=2;

            J.at<double>(0,j)=(pointleft.x*this->Pleft.at<double>(rowB,j))- (this->Pleft.at<double>(rowA,j));
            J.at<double>(2,j)=(pointRight.x*this->Pright.at<double>(rowB,j))- (this->Pright.at<double>(rowA,j));

            rowA=1;
            
            J.at<double>(1,j)=(pointleft.y*this->Pleft.at<double>(rowB,j))- (this->Pleft.at<double>(rowA,j));
            J.at<double>(3,j)=(pointRight.y*this->Pright.at<double>(rowB,j))- (this->Pright.at<double>(rowA,j));
        }
        SVD decom(J);
        Mat V= decom.vt;

        point3D.x=V.at<double>(3,0)/V.at<double>(3,3);
        point3D.y=V.at<double>(3,1)/V.at<double>(3,3);     
        point3D.z=V.at<double>(3,2)/V.at<double>(3,3);
        return point3D;

}

void stereoCamera::estimateEssential() {
    if(this->PointsL.size()<10 || this->PointsL.size()<10 ) {
        cout << "Not enough matches in memory! Run findMatch first!" << endl;
        this->E=NULL;
        return;
    }
    this->E=NULL;
    vector<uchar> status;
    this->E=findFundamentalMat(Mat(PointsL), Mat(PointsR),status, CV_FM_RANSAC, 1, 0.999);

    for(int i=0; i<PointsL.size(); i++) {
        if(status[i]==1) {
            InliersL.push_back(PointsL[i]);
            InliersR.push_back(PointsR[i]);
        }

    }
   

   // cout << "Matches: " << PointsL.size() << " Inliers: " << InliersL.size() << endl;
    this->E=this->Kright.t()*this->E*this->Kleft;

}

void stereoCamera::essentialDecomposition() {

    if(E.empty() ) {
        cout << "Essential Matrix is Empy! Run the estimateEssential first!" << endl;
        return;
    }
    Mat W=Mat(3,3,CV_64FC1);

    W.at<double>(0,0)=0;
    W.at<double>(0,1)=-1;
    W.at<double>(0,2)=0;

    W.at<double>(1,0)=1;
    W.at<double>(1,1)=0;
    W.at<double>(1,2)=0;

    W.at<double>(2,0)=0;
    W.at<double>(2,1)=0;
    W.at<double>(2,2)=1;

    /*Mat S=Mat(3,3,CV_64FC1);

    S.at<double>(0,0)=0;
    S.at<double>(0,1)=-T.at<double>(2,0);
    S.at<double>(0,2)=T.at<double>(1,0);

    S.at<double>(1,0)=T.at<double>(2,0);
    S.at<double>(1,1)=0;
    S.at<double>(1,2)=-T.at<double>(0,0);

    S.at<double>(2,0)=-T.at<double>(1,0);
    S.at<double>(2,1)=T.at<double>(0,0);
    S.at<double>(2,2)=0;

    this->E=S*this->R;*/


    SVD dec(E);

    Mat V=dec.vt;
    Mat U=dec.u;


    Mat R1=U*W*V;
    Mat R2=U*W.t()*V;



    if(determinant(R1)<0 || determinant(R2)<0) {
        E=-E;
        SVD dec2(E);

         V=dec2.vt;
         U=dec2.u;
        
         R1=U*W*V;
         R2=U*W.t()*V;
    }


    Mat t1=U(Range(0,3),Range(2,3));
    Mat t2=-t1;
    
    /*cout << "Rotation Estimated:" << endl;
    printMatrix(R1);
    printMatrix(R2);
    cout << "Translation Estimated:" << endl;
    printMatrix(t1);
    printMatrix(t2);*/

   /* cout << "True Rotation and Traslation :" << endl;
    printMatrix(this->R);
    printMatrix(this->T);*/

  Mat Rnew=Mat(3,3,CV_64FC1);
  Rnew.setTo(cvScalar(0,0,0,0));
    Mat tnew=Mat(3,1,CV_64FC1);

    chierality(R1,R2,t1,t2,Rnew,tnew);

   /*   cout << "Chierality Test:" << endl;
    printMatrix(Rnew);
    printMatrix(tnew);

    Mat diffR=R-Rnew;
    Mat diffT=T/norm(T)-tnew/norm(tnew);
   
    cout << "Rotation Difference :" << norm(diffR) <<endl;
   cout << "Translation Difference :" << norm(diffT) <<endl;*/
  //  cout << "Derminant: " << determinant(Rnew) << endl;
    if(determinant(Rnew)!=0) {
        this->R=Rnew;
        this->updatePMatrix();
    }

printMatrix(R1);
printMatrix(R2);
printMatrix(Rnew);
   // this->T=(tnew/norm(tnew))*norm(T);

}

void stereoCamera::optimization() {
    if(this->PointsL.size()<10 || this->PointsR.size()<10) {
        cout << "No matches found! Run findMatch fist!" << endl;
        return;
    }


    if(this->Kleft.empty() || this->Kright.empty() || this->R.empty() || this->T.empty()) {
        cout << "Cameras are empty, run Calibration first" << endl;
        return;
    }

    if(InliersL.empty()) {
        InliersL=PointsL;
        InliersR=PointsR;
    }


    vector<Point3f> WorldPoints;

    Point3f point3D;
    for(int i =0; i<this->InliersL.size(); i++) {
        point3D=triangulation(InliersL[i],InliersR[i]);
        WorldPoints.push_back(point3D);

    }

    //int numVarStruct=PointsL.size()*3;
    int numVarMoto=3; // 6 -> optimization both rotation and translation. 3 -> only rotation is optimized
    int measNum=4*InliersL.size();
   
    double* Allvars=prepareVariables(this->R,this->T,WorldPoints);
   // double* structVars= (Allvars+6);
    double* meas=(double *)malloc(measNum*sizeof(double));

    int j=0;
    for(int i =0; i<WorldPoints.size(); i++) {
        meas[j]=InliersL[i].x;
        meas[j+1]=InliersL[i].y;
        meas[j+2*WorldPoints.size()]=InliersR[i].x;
        meas[j+2*WorldPoints.size()+1]=InliersR[i].y;
        j=j+2;
        
    }
    
    double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    opts[0]=LM_INIT_MU; opts[1]=1E-12; opts[2]=1E-12; opts[3]=1E-15;
    opts[4]= LM_DIFF_DELTA;

    void (*err)(double *p, double *hx, int m, int n, void *adata);
    err=funcMeas;

   /* printMatrix(this->R);
    printMatrix(this->T);*/

    int ret=dlevmar_dif(err, Allvars, meas, numVarMoto, measNum, 1000, opts, info, NULL, NULL, this); 

   // printf("LM algorithm iterations: %f \n", info[5]);
    Mat Rot;
    Mat Tras;

    getSolutionfromVariables(Rot,Tras,vector<Point3f>(),Allvars);

    this->R=Rot;
    this->T=(Tras/norm(Tras))*norm(T);
    this->updatePMatrix();

    free(meas);
    free(Allvars);

    //printMatrix(Rot);
    //printMatrix(Tras);

}
void stereoCamera::chierality( Mat& R1,  Mat& R2,  Mat& t1,  Mat& t2, Mat& R, Mat& t) {

        Mat A= Mat::eye(3,4,CV_64FC1);
        Mat P1 = this->Kleft*Mat::eye(3, 4, CV_64F);

        for(int i = 0; i < R1.rows; i++)
         {
             double* Mi = A.ptr<double>(i);
             double* MRi = R1.ptr<double>(i);
                for(int j = 0; j < R1.cols; j++)
                     Mi[j]=MRi[j];
         }
        for(int i = 0; i < t1.rows; i++)
         {
             double* Mi = A.ptr<double>(i);
             double* MRi = t1.ptr<double>(i);
             Mi[3]=MRi[0];
         }

        Mat P2=this->Kright*A;

        for(int i = 0; i < R2.rows; i++)
         {
             double* Mi = A.ptr<double>(i);
             double* MRi = R2.ptr<double>(i);
                for(int j = 0; j < R2.cols; j++)
                     Mi[j]=MRi[j];
         }
        for(int i = 0; i < t2.rows; i++)
         {
             double* Mi = A.ptr<double>(i);
             double* MRi = t2.ptr<double>(i);
             Mi[3]=MRi[0];
         }
         Mat P3=this->Kright*A;


        for(int i = 0; i < R1.rows; i++)
         {
             double* Mi = A.ptr<double>(i);
             double* MRi = R1.ptr<double>(i);
             for(int j = 0; j < R1.cols; j++)
                     Mi[j]=MRi[j];
         }
        for(int i = 0; i < t1.rows; i++)
         {
             double* Mi = A.ptr<double>(i);
             double* MRi = t2.ptr<double>(i);
             Mi[3]=MRi[0];
         }
        Mat P4=this->Kright*A;


            for(int i = 0; i < R2.rows; i++)
     {
         double* Mi = A.ptr<double>(i);
         double* MRi = R2.ptr<double>(i);
            for(int j = 0; j < R2.cols; j++)
                 Mi[j]=MRi[j];
     }
    for(int i = 0; i < t2.rows; i++)
     {
         double* Mi = A.ptr<double>(i);
         double* MRi = t1.ptr<double>(i);
         Mi[3]=MRi[0];
     }
     Mat P5=this->Kright*A;

     int passed=0;
         for(int i=0; i<InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(InliersL[i],InliersR[i],P1,P2);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }

         if(passed==InliersL.size()) {
            R=R1;
            t=t1;
            return;
         }


        for(int i=0; i<InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(InliersL[i],InliersR[i],P1,P3);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }
         if(passed==InliersL.size()) {
            R=R2;
            t=t2;
            return;
         }

        for(int i=0; i<InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(InliersL[i],InliersR[i],P1,P4);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }
      if(passed==InliersL.size()) {
            R=R1;
            t=t2;
            return;
         }

        for(int i=0; i<InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(InliersL[i],InliersR[i],P1,P5);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }
      if(passed==InliersL.size()) {
            R=R2;
            t=t1;
            return;
         }

}


Point3f stereoCamera::triangulation(Point2f& pointleft, Point2f& pointRight, Mat Camera1, Mat Camera2) {

      Point3f point3D;
      Mat J=Mat(4,4,CV_64FC1);
      for(int j=0; j<4; j++) {

            int rowA=0;
            int rowB=2;

            J.at<double>(0,j)=(pointleft.x*Camera1.at<double>(rowB,j))- (Camera1.at<double>(rowA,j));
            J.at<double>(2,j)=(pointRight.x*Camera2.at<double>(rowB,j))- (Camera2.at<double>(rowA,j));

            rowA=1;
            
            J.at<double>(1,j)=(pointleft.y*Camera1.at<double>(rowB,j))- (Camera1.at<double>(rowA,j));
            J.at<double>(3,j)=(pointRight.y*Camera2.at<double>(rowB,j))- (Camera2.at<double>(rowA,j));
        }
        SVD decom(J);
        Mat V= decom.vt;

        point3D.x=V.at<double>(3,0)/V.at<double>(3,3);
        point3D.y=V.at<double>(3,1)/V.at<double>(3,3);     
        point3D.z=V.at<double>(3,2)/V.at<double>(3,3);
        return point3D;

}






double* stereoCamera::prepareVariables(Mat& R, Mat& T,vector<Point3f>& WorldPoints) {
    int size = 6+WorldPoints.size()*3;

    double * vars= (double *) malloc(sizeof(double)*size);

    Mat angles;
    Rodrigues(R,angles);

    for(int i=0; i<3; i++) {
        vars[i]=angles.at<double>(i,0);
    }

    for(int i=0; i<3; i++) {
        vars[i+3]=T.at<double>(i,0);
    }

    int j=6;
    for(int i=0; i<WorldPoints.size(); i++) {
     
        vars[j+i]=WorldPoints[i].x;
        j++;
        vars[j+i]=WorldPoints[i].y;
        j++;
        vars[j+i]=WorldPoints[i].z;

    }

    return vars;

}



double * stereoCamera::reprojectionError(Mat& Rot, Mat& Tras) {
    if(PointsL.empty()) {
        cout << "No keypoints found! Run findMatch first!" << endl;
        return 0;
    }

    if(InliersL.empty()) {
    InliersL=PointsL;
    InliersR=PointsR;
    }

    Point2f pointL, pointR;
    vector<Point3f> WorldPoints;
        Mat A = Mat::eye(3, 4, CV_64F);
        Mat P1=this->Kleft*A;

        for(int i = 0; i < Rot.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = Rot.ptr<double>(i);
            for(int j = 0; j < Rot.cols; j++)
                 Mi[j]=MRi[j];
         }
        for(int i = 0; i < Tras.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = Tras.ptr<double>(i);
         Mi[3]=MRi[0];
         }
    Mat P2=this->Kright*A;
    Point3f point3D;
    for(int i =0; i<this->InliersL.size(); i++) {
        pointL=InliersL[i];
        pointR=InliersR[i];

        point3D=triangulation(pointL,pointR);

        WorldPoints.push_back(point3D);

    }
    double * err =(double *) malloc(sizeof(double)*2);
    vector<Point2f> reprojectionL, reprojectionR;

    projectPoints(Mat(WorldPoints), Mat::eye(3,3,CV_64FC1), Mat::zeros(3,1,CV_64FC1), this->Kleft, Mat(), reprojectionL);


    projectPoints(Mat(WorldPoints), this->R, this->T, this->Kright, Mat(), reprojectionR);
        double errorLeft=0;
        double errorRight=0;
    for(int i =0; i<WorldPoints.size(); i++) {
        errorLeft += pow(sqrt(pow(InliersL[i].x-reprojectionL[i].x,2)+ pow(InliersL[i].y-reprojectionL[i].y,2)),2);
        //cvSet2D(err,i,0,cvScalar(errorLeft,0,0,0));


        errorRight += pow(sqrt(pow(InliersR[i].x-reprojectionR[i].x,2)+ pow(InliersR[i].y-reprojectionR[i].y,2)),2);
        //cvSet2D(err,i+WorldPoints.size(),0,cvScalar(errorRight,0,0,0));

    }
    err[0]=errorLeft;
    err[1]=errorRight;
    return err;

}

void stereoCamera::undistortImages() {
    if(this->Kleft.empty() || this->DistL.empty() || this->Kright.empty() || this->DistR.empty()) {
        cout <<" Cameras are not calibrated! Run the Calibration first!" << endl;
        return;
    }
    if(this->imleft.empty() || this->imright.empty()) {
          cout << "Images are not set! set the images first!" << endl;
          return;
    }
        
    undistort(this->imleft,this->imleftund,this->Kleft,this->DistL);
    undistort(this->imright,this->imrightund,this->Kright,this->DistR);
}


const Mat stereoCamera::getImLeftUnd() {
    return this->imleftund;
}
const Mat stereoCamera::getImRightUnd() {
    return this->imrightund;
}

void stereoCamera::setRotation(Mat& Rot, int mul) {

    if(mul==0)
        this->R=Rot;
    else
        this->R=Rot*R;
    this->updatePMatrix();
}
void stereoCamera::setTranslation(Mat& Tras, int mul) {

    if(mul==0)
        this->T=Tras;
    else
        this->T=T+Tras;
    this->updatePMatrix();
}

void stereoCamera::printExtrinsic() {
    printMatrix(this->R);
    printMatrix(this->T);
}

void stereoCamera::updatePMatrix() {


        Mat A = Mat::eye(3, 4, CV_64F);
   
        for(int i = 0; i < this->R.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = this->R.ptr<double>(i);
            for(int j = 0; j < this->R.cols; j++)
                 Mi[j]=MRi[j];
         }
        for(int i = 0; i < this->T.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = this->T.ptr<double>(i);
         Mi[3]=MRi[0];
         }
        Pright=Kright*A;

}

const Mat stereoCamera::getTranslation() {
    return this->T;
}