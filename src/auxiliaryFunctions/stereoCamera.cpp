#include "stereoCamera.h"

void printMatrix(Mat &matrix) {
    int row=matrix.rows;
    int col =matrix.cols;
        cout << endl;
    for(int i = 0; i < matrix.rows; i++)
    {
        const double* Mi = matrix.ptr<double>(i);
        for(int j = 0; j < matrix.cols; j++)
            cout << Mi[j] << " ";
        cout << endl;
    }
        cout << endl;
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

void stereoCamera::stereoCalibration(vector<string> imagelist, int boardWidth, int boardHeight,float sqsize) {
    Size boardSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;
    runStereoCalib(imagelist, boardSize,sqsize);

}

void stereoCamera::stereoCalibration(string imagesFilePath, int boardWidth, int boardHeight,float sqsize) {
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
    runStereoCalib(imagelist, boardSize,sqsize);




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
    
void stereoCamera::runStereoCalib(const vector<string>& imagelist, Size boardSize, const float squareSize)
{
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }
    
    bool displayCorners = false;
    const int maxScale = 2;
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

const Mat stereoCamera::getDisparity16() {
    return this->Disparity16;
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
        Mat Q1;

       // StereoBM bm;
        StereoSGBM sgbm;
        stereoRectify( this->Kleft, this->DistL, this->Kright, this->DistR, img_size, this->R, this->T, R1, R2, P1, P2, Q1, -1, img_size, &roi1, &roi2 );


        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(this->Kleft, this->DistL, R1, P1, img_size, CV_32FC1, map11, map12);
        initUndistortRectifyMap(this->Kright,  this->DistR, R2, P2, img_size, CV_32FC1, map21, map22);
        
        Mat img1r, img2r;
        this->Mapl1=map21;
        this->Mapl2=map22;

        remap(this->imleft, img1r, map11, map12, INTER_LINEAR);
        remap(this->imright, img2r, map21, map22, INTER_LINEAR);



        /*namedWindow("RectL",1);
        namedWindow("RectR",1);
        imshow("RectL",img1r);
        imshow("RectR",img2r);
      cvWaitKey(15);*/

        /*if(numberOfDisparities > 0)
            numberOfDisparities = numberOfDisparities;
        else if(img_size.width==640)
            numberOfDisparities= img_size.width/8;
        else  
            numberOfDisparities= img_size.width/8 +8;*/
    
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
        sgbm.minDisparity =-10; //-15
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = 15; //22
        sgbm.speckleWindowSize = 50; //100
        sgbm.speckleRange = 16; //32
        sgbm.disp12MaxDiff = 1;
        sgbm.fullDP = 1; // alg == STEREO_HH
       
        
        Mat disp, disp8, map, dispTemp,dispTemp16;


        sgbm(img1r, img2r, disp);


        //disp = dispp.colRange(numberOfDisparities, img1p.cols);

        disp.convertTo(map, CV_32FC1, 255/(numberOfDisparities*16.));
        Mat inverseMap(map.rows*map.cols,1,CV_32FC2);
        for( int y = 0; y < map.rows; y++ )

           for( int x = 0; x < map.cols; x++ )
           {
            inverseMap.ptr<float>(y*map.cols+x)[0]= (float)x;
            inverseMap.ptr<float>(y*map.cols+x)[1] = (float)y;

           }


           undistortPoints(inverseMap,inverseMap,this->Kleft,this->DistL,R1,P1);
           Mat mapper= inverseMap.reshape(2,map.rows);
           Mat x;
           remap(map,dispTemp,mapper,x,INTER_LINEAR);
           remap(disp,dispTemp16,mapper,x,INTER_LINEAR);
           dispTemp.convertTo(disp8, CV_8U); 

        this->Disparity=disp8;        
        this->Disparity16=dispTemp16;
        this->Q=Q1;

}


/*
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
*/

void stereoCamera::findMatch() {
    if(this->imleftund.empty() || this->imrightund.empty()) {
              cout << "Images are not set and undistorted! set the images first and call undistortImages()!" << endl;
              return;
        }

    this->PointsL.clear();
    this->PointsR.clear();
    
    this->InliersL.clear();
    this->InliersR.clear();

    Ptr<FeatureDetector> detector = FeatureDetector::create( "SIFT" );
    Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create("SIFT");
    Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create("BruteForce" );

    vector<KeyPoint> keypoints1;
    detector->detect( this->imleftund, keypoints1 );
    Mat descriptors1;
    descriptorExtractor->compute(this->imleftund, keypoints1, descriptors1 );

    vector<KeyPoint> keypoints2;
    detector->detect( this->imrightund, keypoints2 );
    Mat descriptors2;
    descriptorExtractor->compute( this->imrightund, keypoints2, descriptors2 );

    vector<DMatch> filteredMatches;
    crossCheckMatching( descriptorMatcher, descriptors1, descriptors2, filteredMatches, 1 );

    vector<char> matchMask(filteredMatches.size(),1);
    for(int i=0; i<filteredMatches.size(); i++) {
        Point2f pointL=keypoints1[filteredMatches[i].queryIdx].pt;
        Point2f pointR=keypoints2[filteredMatches[i].trainIdx].pt;

        if(abs(pointL.y-pointR.y)<5) {
            this->PointsR.push_back(pointR);
            this->PointsL.push_back(pointL);
        } else
            matchMask[i]=0;
    }



 //   Mat matchImg;
    
 //   drawMatches(this->imleftund, keypoints1, this->imrightund, keypoints2,filteredMatches,matchImg,Scalar(0,0,255,0), Scalar(0,0,255,0),matchMask);
 //   namedWindow("Match",1);
 //   imshow("Match",matchImg); 


}
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
        this->E=Mat(3,3,CV_64FC1);
        return;
    }

    vector<uchar> status;
    this->E=findFundamentalMat(Mat(PointsL), Mat(PointsR),status, CV_FM_RANSAC, 1, 0.999);

    for(int i=0; i<PointsL.size(); i++) {
        if(status[i]==1) {
            InliersL.push_back(PointsL[i]);
            InliersR.push_back(PointsR[i]);
        }

    }
   

//    cout << "Matches: " << PointsL.size() << " Inliers: " << InliersL.size() << endl;
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

const Mat stereoCamera::getRotation() {
    return this->R;
}


void stereoCamera::crossCheckMatching( Ptr<DescriptorMatcher>& descriptorMatcher,
                         const Mat& descriptors1, const Mat& descriptors2,
                         vector<DMatch>& filteredMatches12, int knn )
{
    filteredMatches12.clear();
    vector<vector<DMatch> > matches12, matches21;
    descriptorMatcher->radiusMatch( descriptors1, descriptors2, matches12, 0.15 );
    descriptorMatcher->radiusMatch( descriptors2, descriptors1, matches21, 0.15 );
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}







void stereoCamera::hornRelativeOrientations() {

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

    Mat Rot=this->R.clone();
    Mat Tras=this->T.clone();
    horn(this->Kleft,this->Kright,this->InliersL,this->InliersR,Rot,Tras);

    

    double thetaNew=acos((Rot.at<double>(0,0)+Rot.at<double>(1,1)+Rot.at<double>(2,2)-1)/2);
    double theta=acos((this->R.at<double>(0,0)+this->R.at<double>(1,1)+this->R.at<double>(2,2)-1)/2);
    double relTheta=(thetaNew*180.0/CV_PI)-(theta*180.0/CV_PI);
    double relNorm=norm(Tras-(this->T/norm(this->T)));
 //   cout << "Angolo: " << relTheta << " Tras: " << relNorm << endl;

/*   Mat vettore(3,1,CV_64FC1);
    double coso=1/(2*sin(thetaNew));
    vettore.at<double>(0,0)=coso*(Rot.at<double>(2,1)-Rot.at<double>(1,2));
    vettore.at<double>(1,0)=coso*(Rot.at<double>(0,2)-Rot.at<double>(2,0));
    vettore.at<double>(2,0)=coso*(Rot.at<double>(1,0)-Rot.at<double>(0,1));
    vettore=vettore/norm(vettore);*/

//    printMatrix(Tras);
 //   printMatrix(Rot);
  //  Mat tm=this->T/norm(this->T);
 //   printMatrix(tm);

    this->R=Rot;
    if(relNorm<0.1)
        this->T=Tras/norm(Tras)*norm(T);
    this->updatePMatrix();
}


void stereoCamera::horn(Mat & K1,Mat & K2, vector<Point2f> & PointsL,vector<Point2f> & PointsR,Mat & Rot,Mat & Tras) {
    double prevres = 1E40;
    double res = 1E39;
    double vanishing = 1E-16;
    Tras=Tras/norm(Tras);

    normalizePoints(K1,K2,PointsL,PointsR);
   // savePoints("C:/Users/Utente/Desktop/PointsNL.txt","C:/Users/Utente/Desktop/PointsNR.txt", PointsL,PointsR);
    int iters=0;
    Mat B(3,3,CV_64FC1);
    Mat C(3,3,CV_64FC1);
    Mat D(3,3,CV_64FC1);
    Mat cs(3,1,CV_64FC1);
    Mat ds(3,1,CV_64FC1);
    Mat r1(3,1,CV_64FC1);
    Mat r2(3,1,CV_64FC1);

    while ( (prevres  - res  >  vanishing) ) {
        iters = iters+1;
        
        B.setTo(cvScalar(0));
        C.setTo(cvScalar(0));
        D.setTo(cvScalar(0));
        cs.setTo(cvScalar(0));
        ds.setTo(cvScalar(0));
       
        prevres=res;
        res=0;
        for(int i=0; i<PointsL.size(); i++) {
            
            r1.at<double>(0,0)=PointsL[i].x;
            r1.at<double>(1,0)=PointsL[i].y;
            r1.at<double>(2,0)=1;
            r1=r1/norm(r1);

            r2.at<double>(0,0)=PointsR[i].x;
            r2.at<double>(1,0)=PointsR[i].y;
            r2.at<double>(2,0)=1;
            r2=r2/norm(r2);


            Mat r1p= Rot*r1;

            Mat ci=r1p.cross(r2);
            Mat di=r1p.cross(r2.cross(Tras));
            Mat si=Tras.t()*ci;

          
            B=B+(ci*di.t());
            D=D+(di*di.t());
            C=C+(ci*ci.t());
            
            cs=cs+ (si.at<double>(0,0)*ci);
            ds=ds+ (si.at<double>(0,0)*di);

            Mat residual=Tras.t()*ci*ci.t()*Tras;
            res=res+residual.at<double>(0,0);

        }

        Mat L(7,7,CV_64FC1);
        L.setTo(0);
        
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                L.at<double>(i,j)=C.at<double>(i,j);

        for(int i=0; i<3; i++)
            for(int j=3; j<6; j++)
                L.at<double>(i,j)=B.at<double>(i,j-3);
    
        for(int i=0; i<3; i++)
                L.at<double>(i,6)=Tras.at<double>(i,0);


        for(int i=3; i<6; i++)
            for(int j=0; j<3; j++) {
                Mat Bt=B.t();
                L.at<double>(i,j)=Bt.at<double>(i-3,j);

            }


        for(int i=3; i<6; i++)
            for(int j=3; j<6; j++)
                L.at<double>(i,j)=D.at<double>(i-3,j-3);

        for(int j=0; j<3; j++) {
                Mat Trast=Tras.t();
                L.at<double>(6,j)=Trast.at<double>(0,j);
        }


        Mat Y(7,1,CV_64FC1);
        Y.setTo(0);

        for(int j=0; j<3; j++)
                Y.at<double>(j,0)=-cs.at<double>(j,0);

        for(int j=3; j<6; j++)
                Y.at<double>(j,0)=-ds.at<double>(j-3,0);

        Mat Linv=L.inv();
        Mat result=Linv*Y;
        Tras=Tras+result(Range(0,3),Range(0,1));
        Tras=Tras/norm(Tras);

        Mat q(4,1,CV_64FC1);

        Mat temp=result(Range(3,6),Range(0,1));       
        q.at<double>(0,0)= sqrt(1-(0.25* norm(temp)*norm(temp)));
        q.at<double>(1,0)= 0.5*result.at<double>(3,0);
        q.at<double>(2,0)= 0.5*result.at<double>(4,0);
        q.at<double>(3,0)= 0.5*result.at<double>(5,0);


        Mat deltaR(3,3,CV_64FC1);
        getRotation(q,deltaR);

        Rot=deltaR*Rot;

        SVD dec(Rot);

        Mat Id = Mat::eye(3, 3, CV_64F);

        Mat Vt=dec.vt;
        Mat U=dec.u;

        Rot=U*Id*Vt;
    }



}

void stereoCamera::getRotation(Mat & q, Mat & Rot) {

    double n=q.at<double>(0,0);
    double e1=q.at<double>(1,0);
    double e2=q.at<double>(2,0);
    double e3=q.at<double>(3,0);


    Rot.at<double>(0,0)= (2*((n*n) + (e1*e1))) - 1;
    Rot.at<double>(0,1)= 2*(e1*e2-n*e3); 
    Rot.at<double>(0,2)= 2*(e1*e3+n*e2);

    Rot.at<double>(1,0)= 2*(e1*e2+n*e3);
    Rot.at<double>(1,1)= 2*(n*n+e2*e2)-1;
    Rot.at<double>(1,2)= 2*(e2*e3-n*e1);

    Rot.at<double>(2,0)= 2*(e1*e3-n*e2);
    Rot.at<double>(2,1)= 2*(e2*e3+n*e1);
    Rot.at<double>(2,2)=2*(n*n+e3*e3)-1;
}

void stereoCamera::normalizePoints(Mat & K1, Mat & K2, vector<Point2f> & PointsL, vector<Point2f> & PointsR) {

   
   Mat Point(3,1,CV_64FC1);
   for (int i=0; i<PointsL.size(); i++) {

        Point.at<double>(0,0)=PointsL[i].x;
        Point.at<double>(1,0)=PointsL[i].y;
        Point.at<double>(2,0)=1;

        Mat pnorm=K1.inv()*Point;
        pnorm.at<double>(0,0)=pnorm.at<double>(0,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(1,0)=pnorm.at<double>(1,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(2,0)=1;
      
        
        PointsL[i].x=pnorm.at<double>(0,0);
        PointsL[i].y=pnorm.at<double>(1,0);

        Point.at<double>(0,0)=PointsR[i].x;
        Point.at<double>(1,0)=PointsR[i].y;
        Point.at<double>(2,0)=1;

        pnorm=K2.inv()*Point;
        pnorm.at<double>(0,0)=pnorm.at<double>(0,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(1,0)=pnorm.at<double>(1,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(2,0)=1;

        
        PointsR[i].x=pnorm.at<double>(0,0);
        PointsR[i].y=pnorm.at<double>(1,0);
    }

}

void stereoCamera::savePoints(string pointsLPath,string pointsRPath, vector<Point2f>  PointL, vector<Point2f>  PointR) {
// save intrinsic parameters
  ofstream myfile;
  myfile.open (pointsLPath.c_str());
  for(int i=0; i<PointL.size(); i++)
       myfile << PointL[i].x << " " << PointL[i].y << "\n";
  myfile.close();

  myfile.open (pointsRPath.c_str());
  for(int i=0; i<PointR.size(); i++)
     myfile << PointR[i].x << " " << PointR[i].y << "\n";
  myfile.close();

}

const Mat stereoCamera::getMapL1() {
 return this->Mapl1;
}

const Mat stereoCamera::getMapL2() {
 return this->Mapl2;
}
