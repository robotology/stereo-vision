#include "iCub/stereoVision/stereocamera.h"

Mat StereoCamera::buildRotTras(Mat &R, Mat &T) {
    Mat A = Mat::eye(4, 4, CV_64F);
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
    return A;
}

const Mat StereoCamera::getKleft() {
    return this->Kleft;
}
const Mat StereoCamera::getKright() {
    return this->Kright;
}

const vector<Point2f> StereoCamera::getMatchLeft() {
    return this->InliersL;
}
const vector<Point2f>  StereoCamera::getMatchRight() {

    return this->InliersR;
}

StereoCamera::StereoCamera(std::string intrinsicPath, std::string exstrinsicPath) {

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
        fs["R"] >> Rinit;
        fs["T"] >> T;
        fs["T"] >> Tinit;
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
        this->mutex= new Semaphore(1);
        this->cameraChanged=true;
}

StereoCamera::StereoCamera(Camera Left, Camera Right) {
    this->Kleft=Left.getCameraMatrix();
    this->DistL=Left.getDistVector();

    this->Kright=Right.getCameraMatrix();
    this->DistR=Right.getDistVector();
    this->mutex=new Semaphore(1);
    this->cameraChanged=true;
}

void StereoCamera::setImages(IplImage * left, IplImage * right) {
       this->imleft=left;
       this->imright=right;
}


void StereoCamera::printStereoIntrinsic() {
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

void StereoCamera::stereoCalibration(vector<string> imagelist, int boardWidth, int boardHeight,float sqsize) {
    Size boardSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;
    runStereoCalib(imagelist, boardSize,sqsize);

}

void StereoCamera::stereoCalibration(string imagesFilePath, int boardWidth, int boardHeight,float sqsize) {
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

bool StereoCamera::readStringList( const string& filename, vector<string>& l )
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
    
void StereoCamera::runStereoCalib(const vector<string>& imagelist, Size boardSize, const float squareSize)
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

void StereoCamera::saveCalibration(string extrinsicFilePath, string intrinsicFilePath) {

    if( Kleft.empty() || Kright.empty() || DistL.empty() || DistR.empty() || R.empty() || T.empty()) {
            cout << "Error: cameras are not calibrated! Run the calibration or set intrinsic and extrinsic parameters \n";
            return;
    }

    FileStorage fs(intrinsicFilePath+".yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << Kleft << "D1" << DistL << "M2" << Kright << "D2" << DistR;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    fs.open(extrinsicFilePath+".yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T <<"Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";



    ofstream fout((intrinsicFilePath+".ini").c_str());

    // Left Eye
    fout << "[left]" << endl;
    fout << "fx " << Kleft.at<double>(0,0) << endl;
    fout << "fy " << Kleft.at<double>(1,1) << endl;
    fout << "cx " << Kleft.at<double>(0,2) << endl;
    fout << "cy " << Kleft.at<double>(1,2) << endl;
    fout << "k1 " << DistL.at<double>(0,0) << endl;
    fout << "k2 " << DistL.at<double>(1,0) << endl;
    fout << "p1 " << DistL.at<double>(2,0) << endl;
    fout << "p2 " << DistL.at<double>(3,0) << endl;

    // Right Eye
    fout << "[right]" << endl;
    fout << "fx " << Kright.at<double>(0,0) << endl;
    fout << "fy " << Kright.at<double>(1,1) << endl;
    fout << "cx " << Kright.at<double>(0,2) << endl;
    fout << "cy " << Kright.at<double>(1,2) << endl;
    fout << "k1 " << DistR.at<double>(0,0) << endl;
    fout << "k2 " << DistR.at<double>(1,0) << endl;
    fout << "p1 " << DistR.at<double>(2,0) << endl;
    fout << "p2 " << DistR.at<double>(3,0) << endl;

    fout.close();



}





const Mat StereoCamera::getImLeft() {

    return this->imleft;
}

const Mat StereoCamera::getImRight() {
    return this->imright;
}

const Mat StereoCamera::getDisparity() {
    return this->Disparity;
}

const Mat StereoCamera::getDisparity16() {
    return this->Disparity16;
}
const Mat StereoCamera::getQ() {
    return this->Q;
}
void StereoCamera::computeDisparity(bool best, int uniquenessRatio, int speckleWindowSize,int speckleRange) {
    if(this->Kleft.empty() || this->DistL.empty() || this->Kright.empty() || this->DistR.empty()) {
        cout <<" Cameras are not calibrated! Run the Calibration first!" << endl;
        return;
    }
    if(this->imleft.empty() || this->imright.empty()) {
          cout << "Images are not set! set the images first!" << endl;
          return;
    }
        int SADWindowSize = 0, numberOfDisparities = 0;
        Size img_size = this->imleft.size();

        Rect roi1, roi2;

        StereoSGBM sgbm;

        if(cameraChanged)
        {
            mutex->wait();
            stereoRectify(this->Kleft, this->DistL, this->Kright, this->DistR, img_size, this->R, this->T, this->RLrect, this->RRrect, this->PLrect, this->PRrect, this->Q, -1, img_size, &roi1, &roi2 );
            mutex->post();
        }

        if(cameraChanged)
        {
            initUndistortRectifyMap(this->Kleft, this->DistL, this->RLrect, this->PLrect, img_size, CV_32FC1, this->map11, this->map12);
            initUndistortRectifyMap(this->Kright,  this->DistR, this->RRrect, this->PRrect, img_size, CV_32FC1, this->map21, this->map22);
        }
        
        Mat img1r, img2r;
        remap(this->imleft, img1r, this->map11, this->map12, INTER_LINEAR);
        remap(this->imright, img2r, this->map21,this->map22, INTER_LINEAR);
  
        numberOfDisparities=64;        
        sgbm.preFilterCap = 63; //63
        sgbm.SADWindowSize = 7;         
        int cn = this->imleft.channels();        
        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity =0; //-15
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = uniquenessRatio; //22
        sgbm.speckleWindowSize = speckleWindowSize; //100
        sgbm.speckleRange = speckleRange; //32
        sgbm.disp12MaxDiff = 0;
        sgbm.fullDP = best; // alg == STEREO_HH
       
        
        Mat disp, disp8, map, dispTemp;
        sgbm(img1r, img2r, disp);

        disp.convertTo(map, CV_32FC1, 255/(numberOfDisparities*16.));

        if(cameraChanged)
        {
            this->mutex->wait();
            Mat inverseMapL(map.rows*map.cols,1,CV_32FC2);
            Mat inverseMapR(map.rows*map.cols,1,CV_32FC2);

            for( int y = 0; y < map.rows; y++ )
            {
               for( int x = 0; x < map.cols; x++ )
               {
                inverseMapL.ptr<float>(y*map.cols+x)[0]= (float)x;
                inverseMapL.ptr<float>(y*map.cols+x)[1] = (float)y;
                inverseMapR.ptr<float>(y*map.cols+x)[0]= (float)x;
                inverseMapR.ptr<float>(y*map.cols+x)[1] = (float)y;
               }
            }
            undistortPoints(inverseMapL,inverseMapL,this->Kleft,this->DistL,this->RLrect,this->PLrect);
            undistortPoints(inverseMapR,inverseMapR,this->Kright,this->DistR,this->RRrect,this->PRrect);

            Mat mapperL= inverseMapL.reshape(2,map.rows);
            Mat mapperR= inverseMapR.reshape(2,map.rows);
            this->MapperL=mapperL;
            this->MapperR=mapperR;    
            this->mutex->post();
            cameraChanged=false;
        }

        Mat x;
        remap(map,dispTemp,this->MapperL,x,INTER_LINEAR);
        dispTemp.convertTo(disp8, CV_8U); 

        this->mutex->wait();
        this->Disparity=disp8;        
        this->Disparity16=disp;
        this->mutex->post();
}



void StereoCamera::findMatch(bool visualize, double displacement, double radius) {
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
    crossCheckMatching( descriptorMatcher, descriptors1, descriptors2, filteredMatches,radius, 1 );

    vector<char> matchMask(filteredMatches.size(),1);
    for(int i=0; i<(int)filteredMatches.size(); i++) {
        Point2f pointL=keypoints1[filteredMatches[i].queryIdx].pt;
        Point2f pointR=keypoints2[filteredMatches[i].trainIdx].pt;

        if(abs(pointL.y-pointR.y)<displacement) { //10 320x240 20 640x480
            this->PointsR.push_back(pointR);
            this->PointsL.push_back(pointL);
        } else
            matchMask[i]=0;
    }

   // Visualize Matches
    if(visualize) {
        Mat matchImg;      
        drawMatches(this->imleftund, keypoints1, this->imrightund, keypoints2,filteredMatches,matchImg,Scalar(0,0,255,0), Scalar(0,0,255,0),matchMask);
        namedWindow("Match",1);
        imshow("Match",matchImg); 
        cvWaitKey(0);
    }


}

double StereoCamera::reprojectionErrorAvg() {
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

    for(int i =0; i<(int)this->InliersL.size(); i++) {

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


Point3f StereoCamera::triangulation(Point2f& pointleft, Point2f& pointRight) {
      
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

        point3D.x=(float) ((float) V.at<double>(3,0))/((float) V.at<double>(3,3));
        point3D.y=(float) ((float) V.at<double>(3,1))/((float) V.at<double>(3,3));     
        point3D.z=(float) ((float) V.at<double>(3,2))/((float) V.at<double>(3,3));
        return point3D;

}

void StereoCamera::estimateEssential() {
    if(this->PointsL.size()<10 || this->PointsL.size()<10 ) {
        cout << "Not enough matches in memory! Run findMatch first!" << endl;
        this->E=Mat(3,3,CV_64FC1);
        return;
    }

    vector<uchar> status;
    this->E=findFundamentalMat(Mat(PointsL), Mat(PointsR),status, CV_FM_RANSAC, 1, 0.999);

    for(int i=0; i<(int) PointsL.size(); i++) {
        if(status[i]==1) {
            InliersL.push_back(PointsL[i]);
            InliersR.push_back(PointsR[i]);
        }

    }
   

//    cout << "Matches: " << PointsL.size() << " Inliers: " << InliersL.size() << endl;
    this->E=this->Kright.t()*this->E*this->Kleft;

}

void StereoCamera::essentialDecomposition() {

    if(E.empty() ) {
        cout << "Essential Matrix is Empy! Run the estimateEssential first!" << endl;
        return;
    }
    if(this->PointsL.empty()) {
        cout << "No matches in memory! Run findMatch first!" << endl;
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

    Mat Rnew=Mat(3,3,CV_64FC1);
    Rnew.setTo(cvScalar(0,0,0,0));
    Mat tnew=Mat(3,1,CV_64FC1);

    chierality(R1,R2,t1,t2,Rnew,tnew,this->InliersL,this->InliersR);


    if(determinant(Rnew)!=0) {
        this->R=Rnew;
        this->T=tnew/norm(tnew);
        this->updatePMatrix();
    }

}


void StereoCamera::chierality( Mat& R1,  Mat& R2,  Mat& t1,  Mat& t2, Mat& R, Mat& t, Vector<Point2f> points1, Vector<Point2f> points2) {

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
         for(int i=0; i<(int) InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(points1[i],points2[i],P1,P2);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }

         if(passed==points1.size()) {
            R=R1;
            t=t1;
            return;
         }


        for(int i=0; i<(int) InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(points1[i],points2[i],P1,P3);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }
         if(passed==points1.size()) {
            R=R2;
            t=t2;
            return;
         }

        for(int i=0; i<(int) InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(points1[i],points2[i],P1,P4);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }
      if(passed==points1.size()) {
            R=R1;
            t=t2;
            return;
         }

        for(int i=0; i<(int) InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(points1[i],points2[i],P1,P5);
             if(point3D.z<0) {
                 passed=0;
                 break;
             }
             passed++;

         }
      if(passed==points1.size()) {
            R=R2;
            t=t1;
            return;
         }

}


Point3f StereoCamera::triangulation(Point2f& pointleft, Point2f& pointRight, Mat Camera1, Mat Camera2) {

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

        point3D.x=(float) ((float) V.at<double>(3,0))/((float) V.at<double>(3,3));
        point3D.y=(float) ((float) V.at<double>(3,1))/((float) V.at<double>(3,3));     
        point3D.z=(float) ((float) V.at<double>(3,2))/((float) V.at<double>(3,3));
        return point3D;

}







double * StereoCamera::reprojectionError(Mat& Rot, Mat& Tras) {
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
    for(int i =0; i<(int) this->InliersL.size(); i++) {
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
    for(int i =0; i<(int) WorldPoints.size(); i++) {
        errorLeft += pow(sqrt(pow(InliersL[i].x-reprojectionL[i].x,2)+ pow(InliersL[i].y-reprojectionL[i].y,2)),2);
        //cvSet2D(err,i,0,cvScalar(errorLeft,0,0,0));


        errorRight += pow(sqrt(pow(InliersR[i].x-reprojectionR[i].x,2)+ pow(InliersR[i].y-reprojectionR[i].y,2)),2);
        //cvSet2D(err,i+WorldPoints.size(),0,cvScalar(errorRight,0,0,0));

    }
    err[0]=errorLeft;
    err[1]=errorRight;
    return err;

}

void StereoCamera::undistortImages() {
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


const Mat StereoCamera::getImLeftUnd() {
    return this->imleftund;
}
const Mat StereoCamera::getImRightUnd() {
    return this->imrightund;
}

void StereoCamera::setRotation(Mat& Rot, int mul) {
    this->mutex->wait();
    if(mul==0)
        this->R=Rot;
    if(mul==1)
        this->R=Rot*R;
    if(mul==2)
        this->R=Rot*Rinit;
    this->updatePMatrix();
    this->cameraChanged=true;
    this->mutex->post();
}
void StereoCamera::setTranslation(Mat& Tras, int mul) {
    this->mutex->wait();
    if(mul==0)
        this->T=Tras;
    if(mul==1)
        this->T=Tras+T;
    if(mul==2)
        this->T=Tras+Tinit;

    if(!this->Kleft.empty() && !this->Kright.empty())
        this->updatePMatrix();
    this->cameraChanged=true;
    this->mutex->post();
}

void StereoCamera::printExtrinsic() {
    printMatrix(this->R);
    printMatrix(this->T);
}

void StereoCamera::updatePMatrix() {


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

const Mat StereoCamera::getTranslation() {
    return this->T;
}

const Mat StereoCamera::getRotation() {
    return this->R;
}


void StereoCamera::crossCheckMatching( Ptr<DescriptorMatcher>& descriptorMatcher,
                         const Mat& descriptors1, const Mat& descriptors2,
                         vector<DMatch>& filteredMatches12, double radius, int knn )
{
    filteredMatches12.clear();
    vector<vector<DMatch> > matches12, matches21;
    descriptorMatcher->radiusMatch( descriptors1, descriptors2, matches12, (float) radius );
    descriptorMatcher->radiusMatch( descriptors2, descriptors1, matches21, (float) radius );
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







void StereoCamera::hornRelativeOrientations() {

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


    this->R=Rot.clone();
    this->Rinit=Rot.clone();

    this->T=Tras/norm(Tras)*norm(T);
    this->Tinit=Tras/norm(Tras)*norm(Tinit);

    this->updatePMatrix();
}


void StereoCamera::horn(Mat & K1,Mat & K2, vector<Point2f> & PointsL,vector<Point2f> & PointsR,Mat & Rot,Mat & Tras) {
    double prevres = 1E40;
    double res = 1E39;
    double vanishing = 1E-16;
    Tras=Tras/norm(Tras);

    normalizePoints(K1,K2,PointsL,PointsR);
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
        for(int i=0; i<(int) PointsL.size(); i++) {
            
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

void StereoCamera::getRotation(Mat & q, Mat & Rot) {

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

void StereoCamera::normalizePoints(Mat & K1, Mat & K2, vector<Point2f> & PointsL, vector<Point2f> & PointsR) {

   
   Mat Point(3,1,CV_64FC1);
   for (int i=0; i<(int) PointsL.size(); i++) {

        Point.at<double>(0,0)=PointsL[i].x;
        Point.at<double>(1,0)=PointsL[i].y;
        Point.at<double>(2,0)=1;

        Mat pnorm=K1.inv()*Point;
        pnorm.at<double>(0,0)=pnorm.at<double>(0,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(1,0)=pnorm.at<double>(1,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(2,0)=1;
      
        
        PointsL[i].x=(float) pnorm.at<double>(0,0);
        PointsL[i].y=(float) pnorm.at<double>(1,0);

        Point.at<double>(0,0)=PointsR[i].x;
        Point.at<double>(1,0)=PointsR[i].y;
        Point.at<double>(2,0)=1;

        pnorm=K2.inv()*Point;
        pnorm.at<double>(0,0)=pnorm.at<double>(0,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(1,0)=pnorm.at<double>(1,0)/pnorm.at<double>(2,0);
        pnorm.at<double>(2,0)=1;

        
        PointsR[i].x=(float) pnorm.at<double>(0,0);
        PointsR[i].y=(float) pnorm.at<double>(1,0);
    }

}

void StereoCamera::savePoints(string pointsLPath,string pointsRPath, vector<Point2f>  PointL, vector<Point2f>  PointR) {
  ofstream myfile;
  myfile.open (pointsLPath.c_str());
  for(int i=0; i<(int) PointL.size(); i++)
       myfile << PointL[i].x << " " << PointL[i].y << "\n";
  myfile.close();

  myfile.open (pointsRPath.c_str());
  for(int i=0; i<(int) PointR.size(); i++)
     myfile << PointR[i].x << " " << PointR[i].y << "\n";
  myfile.close();

}


const Mat StereoCamera::getRLrect() {
    return this->RLrect;
}

const Mat StereoCamera::getRRrect() {
    return this->RRrect;
}

const Mat StereoCamera::getMapperL() {
    return this->MapperL;
}

const Mat StereoCamera::getMapperR() {
    return this->MapperR;
}


void StereoCamera::printMatrix(Mat &matrix) {
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


void StereoCamera::setIntrinsics(Mat& KL, Mat& KR, Mat& DistL, Mat& DistR) {
    this->mutex->wait();
    this->Kleft=KL;
    this->Kright=KR;
    this->DistL=DistL;
    this->DistR=DistR;

    if(!this->R.empty() && !this->T.empty())
        updatePMatrix();
    this->cameraChanged=true;
    this->mutex->post();
}

Point3f StereoCamera::metricTriangulation(Point2f &point1) {
    mutex->wait();

    if(Q.empty() || Disparity16.empty()) {
        cout << "Run computeDisparity() method first!" << endl;
        Point3f point;
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;

        mutex->post();
        return point;
    }

    int u=(int) point1.x; 
    int v=(int) point1.y;
    Point3f point;


    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->getMapperL();

    if(Mapper.empty()) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
       
        mutex->post();
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1]; 

    u=cvRound(usign);
    v=cvRound(vsign);

    IplImage disp16=this->getDisparity16();
    

    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
    }
    else {
        CvScalar scal= cvGet2D(&disp16,v,u);
        double disparity=-scal.val[0]/16.0;
        float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
        point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
        point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
        point.z=(float) Q.at<double>(2,3);

        point.x=point.x/w;
        point.y=point.y/w;
        point.z=point.z/w;
    }

    // discard points far more than 2.5 meters or with not valid disparity (<0)
    if(point.z>2.5 || point.z<0) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
    } 
    else {
            Mat P(3,1,CV_64FC1);
            P.at<double>(0,0)=point.x;
            P.at<double>(1,0)=point.y;
            P.at<double>(2,0)=point.z;

            // Rototranslation from rectified camera to original camera
            P=this->getRLrect().t()*P;

            point.x=(float) P.at<double>(0,0);
            point.y=(float) P.at<double>(1,0);
            point.z=(float) P.at<double>(2,0);
    }
 
    mutex->post();
    return point;

}



Point3f StereoCamera::metricTriangulation(Point2f &point1, Mat &H) {
    mutex->wait();

    if(Q.empty() || Disparity16.empty()) {
        cout << "Run computeDisparity() method first!" << endl;
        Point3f point;
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
        
        mutex->post();
        return point;
    }

    int u=(int) point1.x; // matrix starts from (0,0), pixels from (1,1)
    int v=(int) point1.y;
    Point3f point;


    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->getMapperL();

    if(Mapper.empty()) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
        
        mutex->post();
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1]; 

    u=cvRound(usign);
    v=cvRound(vsign);

    IplImage disp16=this->getDisparity16();
    

    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
    }
    else {
        CvScalar scal= cvGet2D(&disp16,v,u);
        double disparity=-scal.val[0]/16.0;
        float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
        point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
        point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
        point.z=(float) Q.at<double>(2,3);

        point.x=point.x/w;
        point.y=point.y/w;
        point.z=point.z/w;
    }

    // discard points far more than 2.5 meters or with not valid disparity (<0)
    if(point.z>2.5 || point.z<0) {
        point.x=-1.0;
        point.y=-1.0;
        point.z=-1.0;
    } 
    else {
        Mat RLrectTmp=this->getRLrect().t();
        Mat Tfake = Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;

        Mat Hrect=buildRotTras(RLrectTmp,Tfake);           
       
        P=H*Hrect*P;     

        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
    }
 
    mutex->post();
    return point;

}
