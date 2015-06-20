/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff, Giulia Pasquale
 * email:   vadim.tikhanoff@iit.it giulia.pasquale@iit.it
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

#include "iCub/stereoVision/stereoCamera.h"
#ifndef USING_GPU
    #include <opencv2/nonfree/nonfree.hpp>
#endif 

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

StereoCamera::StereoCamera(bool rectify) {
    this->mutex=new Semaphore(1);
    this->rectify=rectify;
    this->epipolarTh=0.01;

#ifndef USING_GPU
    cv::initModule_nonfree();
#endif 

    use_elas = false;

}

StereoCamera::StereoCamera(yarp::os::ResourceFinder &rf, bool rectify) {
        Mat KL, KR, DistL, DistR, R, T;
        loadStereoParameters(rf,KL,KR,DistL,DistR,R,T);
        this->mutex= new Semaphore(1);
        this->setIntrinsics(KL,KR,DistL,DistR);
        this->setRotation(R,0);
        this->setTranslation(T,0);

        this->cameraChanged=true;
        this->epipolarTh=0.01;
        this->rectify=rectify;
        buildUndistortRemap();

    #ifndef USING_GPU
        cv::initModule_nonfree();
    #endif 

    use_elas = false;

}

StereoCamera::StereoCamera(Camera Left, Camera Right,bool rectify) {
    this->Kleft=Left.getCameraMatrix();
    this->DistL=Left.getDistVector();

    this->Kright=Right.getCameraMatrix();
    this->DistR=Right.getDistVector();
    this->mutex=new Semaphore(1);
    this->cameraChanged=true;
    this->rectify=rectify;
    this->epipolarTh=0.01;
    buildUndistortRemap();

#ifndef USING_GPU
    cv::initModule_nonfree();
#endif 

    use_elas = false;

}

void StereoCamera::initELAS(yarp::os::ResourceFinder &rf)
{
    use_elas = true;

    string elas_string = rf.check("elas_setting",Value("ROBOTICS")).asString().c_str();

    double disp_scaling_factor = rf.check("disp_scaling_factor",Value(1.0)).asDouble();

    elaswrap = new elasWrapper(disp_scaling_factor, elas_string);

    
    if (rf.check("elas_subsampling"))
        elaswrap->set_subsampling(true);

    if (rf.check("elas_add_corners"))
    	elaswrap->set_add_corners(true);


    elaswrap->set_ipol_gap_width(40);
    if (rf.check("elas_ipol_gap_width"))
    	elaswrap->set_ipol_gap_width(rf.find("elas_ipol_gap_width").asInt());


    if (rf.check("elas_support_threshold"))
    	elaswrap->set_support_threshold(rf.find("elas_support_threshold").asDouble());

    if(rf.check("elas_gamma"))
    	elaswrap->set_gamma(rf.find("elas_gamma").asDouble());

    if (rf.check("elas_sradius"))
    	elaswrap->set_sradius(rf.find("elas_sradius").asDouble());

    if (rf.check("elas_match_texture"))
    	elaswrap->set_match_texture(rf.find("elas_match_texture").asInt());

    if (rf.check("elas_filter_median"))
    	elaswrap->set_filter_median(rf.find("elas_filter_median").asBool());

    if (rf.check("elas_filter_adaptive_mean"))
    	elaswrap->set_filter_adaptive_mean(rf.find("elas_filter_adaptive_mean").asBool());


    cout << endl << "ELAS parameters:" << endl << endl;

    cout << "disp_scaling_factor: " << disp_scaling_factor << endl;

    cout << "setting: " << elas_string << endl;

    cout << "postprocess_only_left: " << elaswrap->get_postprocess_only_left() << endl;
    cout << "subsampling: " << elaswrap->get_subsampling() << endl;

    cout << "add_corners: " << elaswrap->get_add_corners() << endl;

    cout << "ipol_gap_width: " << elaswrap->get_ipol_gap_width() << endl;

    cout << "support_threshold: " << elaswrap->get_support_threshold() << endl;
    cout << "gamma: " << elaswrap->get_gamma() << endl;
    cout << "sradius: " << elaswrap->get_sradius() << endl;

    cout << "match_texture: " << elaswrap->get_match_texture() << endl;

    cout << "filter_median: " << elaswrap->get_filter_median() << endl;
    cout << "filter_adaptive_mean: " << elaswrap->get_filter_adaptive_mean() << endl;

    cout << endl;

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
            Mat img = cv::imread(filename, 0);
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
                char c = (char)cv::waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    return;
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
    fprintf(stdout,"%i pairs have been successfully detected.\n",j);
    nimages = j;
    if( nimages < 2 )
    {
        fprintf(stdout,"Error: too few pairs detected");
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
    
    fprintf(stdout,"Running stereo calibration ...\n");
    
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
        fprintf(stdout,"done with RMS error= %f\n",rms);
    } else
    {
        double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                this->Kleft, this->DistL,
                this->Kright, this->DistR,
                imageSize, this->R, this->T, E, F,
                TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),CV_CALIB_FIX_INTRINSIC);
        fprintf(stdout,"done with RMS error= %f\n",rms);
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
    stereoRectify( this->Kleft, this->DistL, this->Kright, this->DistR, imageSize, this->R, this->T, R1, R2, P1, P2, this->Q, -1);
    fprintf(stdout,"average reprojection err = %f\n",err/npoints);

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
void StereoCamera::rectifyImages()
{
    if(this->Kleft.empty() || this->DistL.empty() || this->Kright.empty() || this->DistR.empty()) {
        cout <<" Cameras are not calibrated! Run the Calibration first!" << endl;
        return;
    }
    if(this->imleft.empty() || this->imright.empty()) {
          cout << "Images are not set! set the images first!" << endl;
          return;
    }
    Size img_size = this->imleft.size();

    if(cameraChanged)
    {
        mutex->wait();
        stereoRectify(this->Kleft, this->DistL, this->Kright, this->DistR, img_size, this->R, this->T, this->RLrect, this->RRrect, this->PLrect, this->PRrect, this->Q, -1);

        mutex->post();
    }

    if(cameraChanged)
    {
        initUndistortRectifyMap(this->Kleft, this->DistL, this->RLrect, this->PLrect, img_size, CV_32FC1, this->map11, this->map12);
        initUndistortRectifyMap(this->Kright,  this->DistR, this->RRrect, this->PRrect, img_size, CV_32FC1, this->map21, this->map22);
    }
    
    Mat img1r, img2r;
    remap(this->imleft, img1r, this->map11, this->map12, cv::INTER_LINEAR);
    remap(this->imright, img2r, this->map21,this->map22, cv::INTER_LINEAR);
    imgLeftRect=img1r;
    imgRightRect=img2r;

}


void StereoCamera::computeDisparity(bool best, int uniquenessRatio, int speckleWindowSize,
                                    int speckleRange, int numberOfDisparities, int SADWindowSize,
                                    int minDisparity, int preFilterCap, int disp12MaxDiff)
{
    if (this->Kleft.empty() || this->DistL.empty() || this->Kright.empty() || this->DistR.empty())
    {
        cout <<" Cameras are not calibrated! Run the Calibration first!" << endl;
        return;
    }

    if (this->imleft.empty() || this->imright.empty())
    {
        cout << "Images are not set! set the images first!" << endl;
        return;
    }

    Size img_size=this->imleft.size();

    if (cameraChanged)
    {
        mutex->wait();
        stereoRectify(this->Kleft, this->DistL, this->Kright, this->DistR, img_size,
                      this->R, this->T, this->RLrect, this->RRrect, this->PLrect,
                      this->PRrect, this->Q, -1);

        if (!rectify)
        {
            this->RLrect=Mat::eye(3,3,CV_32FC1);
            this->RRrect=Mat::eye(3,3,CV_32FC1);
            this->PLrect=this->Kleft;
            this->PRrect=this->Kright;
        }
        mutex->post();
    }

    if (cameraChanged)
    {
        initUndistortRectifyMap(this->Kleft, this->DistL, this->RLrect, this->PLrect,
                                img_size, CV_32FC1, this->map11, this->map12);
        initUndistortRectifyMap(this->Kright,  this->DistR, this->RRrect, this->PRrect,
                                img_size, CV_32FC1, this->map21, this->map22);
    }
    
    Mat img1r, img2r;
    remap(this->imleft, img1r, this->map11, this->map12, cv::INTER_LINEAR);
    remap(this->imright, img2r, this->map21,this->map22, cv::INTER_LINEAR);

    imgLeftRect = img1r;
    imgRightRect = img2r;

    Mat disp,disp8,map,dispTemp;

    if (use_elas)
    {
        
        elaswrap->compute_disparity(img1r, img2r, disp, numberOfDisparities);

        map = disp * (255.0 / numberOfDisparities);
        //threshold(map, map, 0, 255.0, THRESH_TOZERO);

    } else
    {
        StereoSGBM sgbm;
        sgbm.preFilterCap =         preFilterCap; //63
        sgbm.SADWindowSize =        SADWindowSize;
        int cn =                    this->imleft.channels();
        sgbm.P1 =                   8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 =                   32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity =         minDisparity; //-15
        sgbm.numberOfDisparities =  numberOfDisparities;
        sgbm.uniquenessRatio =      uniquenessRatio; //22
        sgbm.speckleWindowSize =    speckleWindowSize; //100
        sgbm.speckleRange =         speckleRange; //32
        sgbm.disp12MaxDiff =        disp12MaxDiff;
        sgbm.fullDP =               best; // alg == STEREO_HH

        sgbm(img1r, img2r, disp);

        disp.convertTo(map, CV_32FC1, 1.0,0.0);
        map.convertTo(map,CV_32FC1,255/(numberOfDisparities*16.));
        //normalize(map,map, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    }
    
    if (cameraChanged)
    {
        this->mutex->wait();
        Mat inverseMapL(map.rows*map.cols,1,CV_32FC2);
        Mat inverseMapR(map.rows*map.cols,1,CV_32FC2);

        for (int y=0; y<map.rows; y++)
        {
            for (int x=0; x<map.cols; x++)
            {
                inverseMapL.ptr<float>(y*map.cols+x)[0]=(float)x;
                inverseMapL.ptr<float>(y*map.cols+x)[1]=(float)y;
                inverseMapR.ptr<float>(y*map.cols+x)[0]=(float)x;
                inverseMapR.ptr<float>(y*map.cols+x)[1]=(float)y;
            }
        }

        undistortPoints(inverseMapL,inverseMapL,this->Kleft,this->DistL,this->RLrect,this->PLrect);
        undistortPoints(inverseMapR,inverseMapR,this->Kright,this->DistR,this->RRrect,this->PRrect);

        Mat mapperL=inverseMapL.reshape(2,map.rows);
        Mat mapperR=inverseMapR.reshape(2,map.rows);
        this->MapperL=mapperL;
        this->MapperR=mapperR;
        this->mutex->post();
        cameraChanged=false;
    }

    Mat x;
    remap(map,dispTemp,this->MapperL,x,cv::INTER_LINEAR);
    dispTemp.convertTo(disp8,CV_8U); 

    this->mutex->wait();

    this->Disparity = disp8;

    if (use_elas)
        disp.convertTo(disp, CV_16SC1, 16.0);

    this->Disparity16 = disp;

    this->mutex->post();
}


Point2f StereoCamera::fromRectifiedToOriginal(int u, int v, int camera)
{
    cv::Point2f originalPoint;


    if(u>=map11.rows || u<0 || v>=map12.cols || v< 0)
    {
        originalPoint.x=0;
        originalPoint.y=0;
        return originalPoint;
    }
    if(camera==LEFT)
    {
            originalPoint.x=map11.ptr<float>(v)[u];
            originalPoint.y=map12.ptr<float>(v)[u];
    }
    else
    {
            originalPoint.x=map21.ptr<float>(v)[u];
            originalPoint.y=map22.ptr<float>(v)[u];
    }


    return originalPoint;

}


Mat StereoCamera::findMatch(bool visualize, double displacement, double radius)
{
    if (this->imleftund.empty() || this->imrightund.empty())
    {
        imleftund=imleft;
        imrightund=imright;
    }

    this->PointsL.clear();
    this->PointsR.clear();
    
    this->InliersL.clear();
    this->InliersR.clear();

    Mat grayleft(imleftund.rows,imleftund.cols, CV_8UC1);
    imleftund.convertTo(grayleft,CV_8UC1);
    
    Mat grayright(imrightund.rows,imrightund.cols,CV_8UC1);
    imrightund.convertTo(grayright,CV_8UC1);
    
    vector<KeyPoint> keypoints1,keypoints2;
    Mat descriptors1,descriptors2;

    Ptr<cv::FeatureDetector> detector=cv::FeatureDetector::create("SIFT");
    Ptr<cv::DescriptorExtractor> descriptorExtractor=cv::DescriptorExtractor::create("SIFT");
    cv::BFMatcher descriptorMatcher;

    yAssert(detector!=NULL);
    yAssert(descriptorExtractor!=NULL);

    detector->detect(grayleft,keypoints1);
    descriptorExtractor->compute(grayleft,keypoints1,descriptors1);

    detector->detect(grayright,keypoints2);
    descriptorExtractor->compute(grayright,keypoints2,descriptors2);

    vector<DMatch> filteredMatches;
    crossCheckMatching(descriptorMatcher,descriptors1,descriptors2,filteredMatches,radius);

    for (size_t i=0; i<filteredMatches.size(); i++)
    {
        Point2f pointL=keypoints1[filteredMatches[i].queryIdx].pt;
        Point2f pointR=keypoints2[filteredMatches[i].trainIdx].pt;
        if (fabs(pointL.y-pointR.y)<displacement)
        {
            this->PointsR.push_back(pointR);
            this->PointsL.push_back(pointL);
        }   
    }

    Mat matchImg;
    if (visualize)
        cv::drawMatches(this->imleftund,keypoints1,this->imrightund,keypoints2,
                        filteredMatches,matchImg,Scalar(0,0,255,0),Scalar(0,0,255,0));

    return matchImg;
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
      J.setTo(0);
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

Mat StereoCamera::FfromP(Mat& P1, Mat& P2)
{
    Mat F_true(3,3,CV_64FC1);

    Mat X1(2,4,CV_64FC1);
    Mat X2(2,4,CV_64FC1);
    Mat X3(2,4,CV_64FC1);

    Mat Y1(2,4,CV_64FC1);
    Mat Y2(2,4,CV_64FC1);
    Mat Y3(2,4,CV_64FC1);
    

    for(int i=0; i<P1.rows; i++)
    {
        for(int j=0; j<P1.cols; j++)
        {
            if(i==0)
            {
                X2.at<double>(1,j)= P1.at<double>(i,j);
                X3.at<double>(0,j)= P1.at<double>(i,j);
                Y2.at<double>(1,j)= P2.at<double>(i,j);
                Y3.at<double>(0,j)= P2.at<double>(i,j);                    
            }
            if(i==1)
            {
                X1.at<double>(0,j)= P1.at<double>(i,j);
                X3.at<double>(1,j)= P1.at<double>(i,j);
                Y1.at<double>(0,j)= P2.at<double>(i,j);
                Y3.at<double>(1,j)= P2.at<double>(i,j);                   
            }

            if(i==2)
            {
                X1.at<double>(1,j)= P1.at<double>(i,j);
                X2.at<double>(0,j)= P1.at<double>(i,j);
                Y1.at<double>(1,j)= P2.at<double>(i,j);
                Y2.at<double>(0,j)= P2.at<double>(i,j);

            }

        }
    }



    std::vector<Mat> MatX;
    std::vector<Mat> MatY;

    MatX.push_back(X1);
    MatX.push_back(X2);
    MatX.push_back(X3);

    MatY.push_back(Y1);
    MatY.push_back(Y2);
    MatY.push_back(Y3);



    for(int i=0; i<F_true.rows; i++)
    {
        for(int j=0; j<F_true.cols; j++)
        {
            Mat X=MatX[i];
            Mat Y=MatY[j];

            Mat concatenated;

            cv::vconcat(X,Y,concatenated);

            F_true.at<double>(j,i)=cv::determinant(concatenated);


        }

    }


    return F_true;

}

void StereoCamera::estimateEssential()
{
    this->InliersL.clear();
    this->InliersR.clear();

    if (this->PointsL.size()<10 || this->PointsL.size()<10 )
    {
        cout << "Not enough matches in memory! Run findMatch first!" << endl;
        this->E=Mat(3,3,CV_64FC1);
        return;
    }

    updateExpectedCameraMatrices();
    Mat F_exp=FfromP(Pleft_exp,Pright_exp);

    vector<Point2f> filteredL;
    vector<Point2f> filteredR;

    fprintf(stdout,"%lu Match Found \n",PointsR.size());
    Mat pl=Mat(3,1,CV_64FC1);
    Mat pr=Mat(3,1,CV_64FC1);

    for (size_t i=0; i<(int) PointsL.size(); i++)
    {
        pl.at<double>(0,0)=PointsL[i].x;
        pl.at<double>(1,0)=PointsL[i].y;
        pl.at<double>(2,0)=1;
        
        pr.at<double>(0,0)=PointsR[i].x;
        pr.at<double>(1,0)=PointsR[i].y;
        pr.at<double>(2,0)=1;
             
        Mat xrFxl=pr.t()*F_exp*pl;
        Mat Fxl=F_exp*pl;
        Mat Fxr=F_exp.t()*pr;

        pow(xrFxl,2,xrFxl);

        pow(Fxl,2,Fxl);

        pow(Fxr,2,Fxr);
        
        Scalar den1,den2;
        den1=sum(Fxl);
        den2=sum(Fxr);
        double sampsonDistance=xrFxl.at<double>(0,0)/(den1.val[0]+den2.val[0]);
        
        if (sampsonDistance<0.1)
        {
            filteredL.push_back(PointsL[i]);
            filteredR.push_back(PointsR[i]);
        }
    }

    fprintf(stdout,"%lu Match After Kinematics Filtering \n",filteredL.size());

    vector<uchar> status;
    this->F=findFundamentalMat(Mat(filteredL), Mat(filteredR),status, CV_FM_8POINT, 1, 0.999);
    
    for (size_t i=0; i<(int) filteredL.size(); i++)
    {
        pl.at<double>(0,0)=filteredL[i].x;
        pl.at<double>(1,0)=filteredL[i].y;
        pl.at<double>(2,0)=1;
        
        pr.at<double>(0,0)=filteredR[i].x;
        pr.at<double>(1,0)=filteredR[i].y;
        pr.at<double>(2,0)=1;
             
        Mat xrFxl=pr.t()*F*pl;
        Mat Fxl=F*pl;
        Mat Fxr=F.t()*pr;

        pow(xrFxl,2,xrFxl);
        pow(Fxl,2,Fxl);
        pow(Fxr,2,Fxr);
        
        Scalar den1,den2;
        den1=sum(Fxl);
        den2=sum(Fxr);
        double sampsonDistance=xrFxl.at<double>(0,0)/(den1.val[0]+den2.val[0]);

        if (status[i]==1 && xrFxl.at<double>(0,0)<0.001) 
        {
            InliersL.push_back(filteredL[i]);
            InliersR.push_back(filteredR[i]);
        }
    }

    fprintf(stdout,"%lu Match After RANSAC Filtering \n",InliersL.size());

    if (this->InliersL.size()<10 || this->InliersR.size()<10 )
    {
        InliersL.clear();
        InliersR.clear();
        cout << "Not enough matches in memory! Run findMatch first!" << endl;
        this->E=Mat(3,3,CV_64FC1);
        return;
    }    
   
    this->F=findFundamentalMat(Mat(InliersL), Mat(InliersR),status, CV_FM_8POINT, 1, 0.999);
    this->E=this->Kright.t()*this->F*this->Kleft;

}


bool StereoCamera::essentialDecomposition()
{
    if (E.empty())
    {
        cout << "Essential Matrix is empty! Run the estimateEssential first!" << endl;
        return false;
    }
    
    if (this->InliersL.empty())
    {
        cout << "No matches in memory! Run findMatch first!" << endl;
        return false;
    }

    Mat W=Mat(3,3,CV_64FC1);
    W.setTo(0);
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
    
    Mat Y=Mat::eye(3,3,CV_64FC1);
    Y.at<double>(2,2)=0.0;
    E=dec.u*Y*dec.vt; // projection to the Essential Matrix space
    
    dec(E);

    Mat V=dec.vt;
    Mat U=dec.u;

    Mat R1=U*W*V;
    Mat R2=U*W.t()*V;
    
    if (determinant(R1)<0 || determinant(R2)<0)
    {
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
    Rnew.setTo(0);
    Mat tnew=Mat(3,1,CV_64FC1);

    chierality(R1,R2,t1,t2,Rnew,tnew,this->InliersL,this->InliersR);
    
    //double t_norm=norm(T/norm(T),tnew/norm(tnew));
    //double r_norm=norm(R,Rnew);
    
    Mat rvec_new=Mat::zeros(3,1,CV_64FC1);
    Mat rvec_exp=Mat::zeros(3,1,CV_64FC1);
    Rodrigues(Rnew,rvec_new);
    Rodrigues(R_exp,rvec_exp);

    Mat t_est=(tnew/norm(tnew))*norm(this->T);
    
    Mat diff_angles=rvec_exp-rvec_new;
    Mat diff_tran=T_exp-t_est;
    
    fprintf(stdout,"Angles Differences: %f %f %f\n",diff_angles.at<double>(0,0),diff_angles.at<double>(1,0),diff_angles.at<double>(2,0));
    fprintf(stdout,"Translation Differences: %f %f %f\n",diff_tran.at<double>(0,0),diff_tran.at<double>(1,0),diff_tran.at<double>(2,0));    
    
    // Magic numbers: rvec_new are the rotation angles, only vergence (rvec_new(1,0)) is allowed to be large
    // t_est is the translation estimated, it can change a little bit when joint 4 of the head is moving
    if (fabs(diff_angles.at<double>(0,0))<0.15 && fabs(diff_angles.at<double>(1,0))<0.15 && fabs(diff_angles.at<double>(2,0))<0.15 &&
        fabs(diff_tran.at<double>(0,0))<0.01 && fabs(diff_tran.at<double>(1,0))<0.01  && fabs(diff_tran.at<double>(2,0))<0.01)    
    {
        this->mutex->wait();
        this->R=Rnew;
        this->T=t_est;
        this->updatePMatrix();
        this->cameraChanged=true;
        this->mutex->post();
        return true;
    }
    else
        return false;
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
        A= Mat::eye(3,4,CV_64FC1);
        
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
        A= Mat::eye(3,4,CV_64FC1);

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
        A= Mat::eye(3,4,CV_64FC1);


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

     int err1=0; //R1 t1
     int err2=0; //R2 t2
     int err3=0; //R1 t2
     int err4=0; //R2 t1
     Mat point(4,1,CV_64FC1);

         for(int i=0; i<(int) InliersL.size(); i++) 
         {
             Point3f point3D=triangulation(points1[i],points2[i],P1,P2);
             Mat H1=buildRotTras(R1,t1);
             point.at<double>(0,0)=point3D.x;
             point.at<double>(1,0)=point3D.y;
             point.at<double>(2,0)=point3D.z;
             point.at<double>(0,0)=1.0;
             Mat rotatedPoint=H1*point;

             //fprintf(stdout, "Camera P2 Point3D: %f %f %f Rotated Point: %f %f %f \n", point3D.x,point3D.y,point3D.z, rotatedPoint.at<double>(0,0),rotatedPoint.at<double>(1,0),rotatedPoint.at<double>(2,0));

             if(point3D.z<0 || rotatedPoint.at<double>(2,0)<0) {
                 err1++;                 
             }
             point3D=triangulation(points1[i],points2[i],P1,P3);
             Mat H2=buildRotTras(R2,t2);
             point.at<double>(0,0)=point3D.x;
             point.at<double>(1,0)=point3D.y;
             point.at<double>(2,0)=point3D.z;
             point.at<double>(0,0)=1.0;
             rotatedPoint=H2*point;  
             //fprintf(stdout, "Camera P3 Point3D: %f %f %f Rotated Point: %f %f %f \n", point3D.x,point3D.y,point3D.z, rotatedPoint.at<double>(0,0),rotatedPoint.at<double>(1,0),rotatedPoint.at<double>(2,0));

             if(point3D.z<0 || rotatedPoint.at<double>(2,0)<0) {
                 err2++;                 
             }
                          
             point3D=triangulation(points1[i],points2[i],P1,P4);   
             Mat H3=buildRotTras(R1,t2);
             point.at<double>(0,0)=point3D.x;
             point.at<double>(1,0)=point3D.y;
             point.at<double>(2,0)=point3D.z;
             point.at<double>(0,0)=1.0;
             rotatedPoint=H3*point;
             //fprintf(stdout, "Camera P4 Point3D: %f %f %f Rotated Point: %f %f %f \n", point3D.x,point3D.y,point3D.z, rotatedPoint.at<double>(0,0),rotatedPoint.at<double>(1,0),rotatedPoint.at<double>(2,0));

             if(point3D.z<0 || rotatedPoint.at<double>(2,0)<0) {
                 err3++;                 
             } 
             
             point3D=triangulation(points1[i],points2[i],P1,P5);
             Mat H4=buildRotTras(R2,t1);
             point.at<double>(0,0)=point3D.x;
             point.at<double>(1,0)=point3D.y;
             point.at<double>(2,0)=point3D.z;
             point.at<double>(0,0)=1.0;
             rotatedPoint=H4*point;
             //fprintf(stdout, "Camera P5 Point3D: %f %f %f Rotated Point: %f %f %f \n", point3D.x,point3D.y,point3D.z, rotatedPoint.at<double>(0,0),rotatedPoint.at<double>(1,0),rotatedPoint.at<double>(2,0));
             
             if(point3D.z<0 || rotatedPoint.at<double>(2,0)<0) {
                 err4++;                 
             } 

         }

    /*printMatrix(R1);
    printMatrix(t1);
    printMatrix(R2);
    printMatrix(t2);*/
    //fprintf(stdout, "Inliers: %d, %d, \n",points1.size(),points2.size());
    //fprintf(stdout, "errors: %d, %d, %d, %d, \n",err1,err2,err3,err4);

      double minErr=10000;
      double secondErr=minErr;

      int idx=0;
      if(err1<minErr && t1.ptr<double>(0)[0]<0)
      {
        idx=1;
        secondErr=minErr;
        minErr=err1;
      }
        
      if(err2<minErr && t2.ptr<double>(0)[0]<0)
      {
        idx=2;
        secondErr=minErr;
        minErr=err2;
      } 
      if(err3<minErr && t2.ptr<double>(0)[0]<0)
      {
        idx=3;
        secondErr=minErr;
        minErr=err3;
      }
      if(err4<minErr && t1.ptr<double>(0)[0]<0)
      {
        idx=4;
        secondErr=minErr;
        minErr=err4;
      }

      /*if(secondErr==minErr)
      {
        R=this->R;
        t=this->T;
        return;      
      }*/
      if(idx==1) {
            R=R1;
            t=t1;
            return;
       }
      if(idx==2) {
            R=R2;
            t=t2;
            return;
       }
      if(idx==3) {
            R=R1;
            t=t2;
            return;
       }
      if(idx==4) {
            R=R2;
            t=t1;
            return;
       }

}


Point3f StereoCamera::triangulation(Point2f& pointleft, Point2f& pointRight, Mat Camera1, Mat Camera2) {

      Point3f point3D;
      Mat J=Mat(4,4,CV_64FC1);
      J.setTo(0);
                
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

       // printMatrix(V);
        
        /*Mat sol=Mat(4,1,CV_64FC1);
        sol.at<double>(0,0)=V.at<double>(0,0);
        sol.at<double>(1,0)=V.at<double>(1,1);
        sol.at<double>(2,0)=V.at<double>(2,2);
        sol.at<double>(3,0)=V.at<double>(3,3);
        
        Mat test=J*sol;
        
        printMatrix(test);*/
        point3D.x=(float) ((float) V.at<double>(3,0))/((float) V.at<double>(3,3));
        point3D.y=(float) ((float) V.at<double>(3,1))/((float) V.at<double>(3,3));     
        point3D.z=(float) ((float) V.at<double>(3,2))/((float) V.at<double>(3,3));
        return point3D;

}



/*Point3f StereoCamera::triangulationLS(Point2f& point1, Point2f& point2, Mat P, Mat P1)
{

    Point3f u(point1.x,point1.y,1.0);
    Mat um = Kleft.inv() * Mat(u);
    ul = um.at<Point3f>(0);

    Point3f u1(point2.x,point2.y,1.0);
    Mat um1 = Kright.inv() * Mat(u1);
    ur = um1.at<Point3f>(0);
    
    Mat A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));

   Mat A=Mat(4,3,CV_64FC1);
   
    A.at<double>(0,0)=u.x*P.at<double>(2,0)-P.at<double(0.0);
    
    return u;


}*/



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

const Mat StereoCamera::getFundamental() {
    return this->F;
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
        
    if(R_exp.empty())
       R_exp=R;
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

    if(T_exp.empty())
        T_exp=T;
        
    if(!this->Kleft.empty() && !this->Kright.empty())
        this->updatePMatrix();
    this->cameraChanged=true;
    this->mutex->post();
}

void StereoCamera::printExtrinsic() {
    printMatrix(this->R);
    printMatrix(this->T);
}

void StereoCamera::updateExpectedCameraMatrices()
{
        Mat A = Mat::eye(3, 4, CV_64F);
        Pleft_exp=Kleft*A;

        for(int i = 0; i < this->R_exp.rows; i++)
        {
         double* Mi = A.ptr<double>(i);
         double* MRi = this->R_exp.ptr<double>(i);
            for(int j = 0; j < this->R_exp.cols; j++)
                 Mi[j]=MRi[j];
        }
        for(int i = 0; i < this->T_exp.rows; i++)
        {
         double* Mi = A.ptr<double>(i);
         double* MRi = this->T_exp.ptr<double>(i);
         Mi[3]=MRi[0];
        }

        Pright_exp=Kright*A;
}


void StereoCamera::updatePMatrix()
{
        Mat A = Mat::eye(3, 4, CV_64F);
        Pleft=Kleft*A;
   
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


void StereoCamera::crossCheckMatching( cv::BFMatcher &descriptorMatcher,
                         const Mat& descriptors1, const Mat& descriptors2,
                         vector<DMatch>& filteredMatches12, double radius)
{
    filteredMatches12.clear();
    vector<vector<DMatch> > matches12, matches21;

    descriptorMatcher.radiusMatch(descriptors1,descriptors2,matches12,(float)radius);
    descriptorMatcher.radiusMatch(descriptors2,descriptors1,matches21,(float)radius);

    for (size_t m=0; m<matches12.size(); m++)
    {
        bool findCrossCheck=false;
        for (size_t fk=0; fk<matches12[m].size(); fk++)
        {
            DMatch forward=matches12[m][fk];
            for (size_t bk=0; bk<matches21[forward.trainIdx].size(); bk++)
            {
                DMatch backward=matches21[forward.trainIdx][bk];
                if (backward.trainIdx==forward.queryIdx)
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck=true;
                    break;
                }
            }

            if (findCrossCheck)
                break;
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


Mat StereoCamera::drawMatches()
{
    if (this->imleftund.empty() || this->imrightund.empty())
    {
        imleftund=imleft;
        imrightund=imright;
    }

    Mat matchImg;
    vector<KeyPoint> keypoints1(InliersL.size());
    vector<KeyPoint> keypoints2(InliersL.size());
    vector<DMatch> filteredMatches(InliersL.size());

    for (int i=0; i<InliersL.size(); i++)
    {
        filteredMatches[i].queryIdx=i;
        filteredMatches[i].trainIdx=i;

        keypoints1[i]=cv::KeyPoint(InliersL[i],2);
        keypoints2[i]=cv::KeyPoint(InliersR[i],2);
    }

    cv::drawMatches(this->imleftund,keypoints1,this->imrightund,keypoints2,
                    filteredMatches,matchImg,Scalar(0,0,255,0),Scalar(0,0,255,0));

    return matchImg;
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
        
        B.setTo(0);
        C.setTo(0);
        D.setTo(0);
        cs.setTo(0);
        ds.setTo(0);
       
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
    buildUndistortRemap();
    this->mutex->post();
}

Point3f StereoCamera::metricTriangulation(Point2f &point1, double thMeters) {
    mutex->wait();

    if(Q.empty() || Disparity16.empty()) {
        cout << "Run computeDisparity() method first!" << endl;
        Point3f point;
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;

        mutex->post();
        return point;
    }

    int u=(int) point1.x; 
    int v=(int) point1.y;
    Point3f point;


    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->getMapperL();

    if(Mapper.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
       
        mutex->post();
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1]; 

    u=cvRound(usign);
    v=cvRound(vsign);

    IplImage disp16=this->getDisparity16();
    

    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        mutex->post();
        return point;
    }

    CvScalar scal=cvGet2D(&disp16,v,u);
    double disparity=scal.val[0]/16.0;
    float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
    point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
    point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
    point.z=(float) Q.at<double>(2,3);

    point.x=point.x/w;
    point.y=point.y/w;
    point.z=point.z/w;

    // discard points far more than thMeters meters or with not valid disparity (<0)
    if(point.z>thMeters || point.z<0) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
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



Point3f StereoCamera::metricTriangulation(Point2f &point1, Mat &H, double thMeters) {
    mutex->wait();

    if(H.empty())
        H=H.eye(4,4,CV_64FC1);

    if(Q.empty() || Disparity16.empty()) {
        cout << "Run computeDisparity() method first!" << endl;
        Point3f point;
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        mutex->post();
        return point;
    }

    int u=(int) point1.x; // matrix starts from (0,0), pixels from (1,1)
    int v=(int) point1.y;
    Point3f point;


    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->getMapperL();

    if(Mapper.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        
        mutex->post();
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1]; 

    u=cvRound(usign);
    v=cvRound(vsign);

    IplImage disp16=this->getDisparity16();
    

    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        mutex->post();
        return point;
    }

    CvScalar scal=cvGet2D(&disp16,v,u);
    double disparity=scal.val[0]/16.0;
    float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
    point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
    point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
    point.z=(float) Q.at<double>(2,3);

    point.x=point.x/w;
    point.y=point.y/w;
    point.z=point.z/w;

    // discard points far more than thMeters meters or with not valid disparity (<0)
    if(point.z>thMeters || point.z<0) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        mutex->post();
        return point;
    } 

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

    mutex->post();
    return point;

}


Point3f StereoCamera::triangulateKnownDisparity(float u, float v, float d, Mat &H)
{
    mutex->wait();
    if(Q.empty())
    {
        cout << "Run rectifyImages() method first!" << endl;
        Point3f point;
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        mutex->post();
        return point;
    }

    if(H.empty())
        H=H.eye(4,4,CV_64FC1);

    Point3f point;

    float w= (float) ((float) d*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
    point.x= (float)((float) (u)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
    point.y=(float)((float) (v)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
    point.z=(float) Q.at<double>(2,3);

    // Rectified Camera System
    point.x=point.x/w;
    point.y=point.y/w;
    point.z=point.z/w;

    // We transform to H Coordinate System
    Mat RLrectTmp=this->getRLrect().t(); // First it transform the point to the unrectified camera reference system
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

    mutex->post();
    return point;
}

Mat StereoCamera::getLRectified()
{
    return this->imgLeftRect;
}

Mat StereoCamera::getRRectified()
{
    return this->imgRightRect;
}

vector<Point2f> StereoCamera::projectPoints3D(string camera, vector<Point3f> &points3D, Mat &H)
{
    vector<Point2f> points2D;

    if(this->Kleft.empty() || this->DistL.empty() || this->Kright.empty() || this->DistR.empty()) {
        cout <<" Cameras are not calibrated! Run the Calibration first!" << endl;
        return points2D;
    }

    if(H.empty())
        H=H.eye(4,4,CV_64FC1);

    mutex->wait();

    for (int i=0; i<points3D.size(); i++)
    {   
        // Apply inverse Trasformation for each point
        Point3f point=points3D[i];
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;

        P=H.inv()*P;

        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));

        points3D[i]=point;
    }

    Mat cameraMatrix, distCoeff, rvec, tvec;
    rvec=Mat::zeros(3,1,CV_64FC1);

    if(camera=="left")
    {
        cameraMatrix=this->Kleft;
        distCoeff=this->DistL;
        Mat R2= Mat::eye(3,3,CV_64FC1);
        Rodrigues(R2,rvec);
        tvec=Mat::zeros(3,1,CV_64FC1);
    }
    else
    {
        cameraMatrix=this->Kright;
        distCoeff=this->DistR;
        Mat R2= this->R;
        Rodrigues(R2,rvec);
        tvec=this->T;
    }

    Mat points3Mat(points3D);
    projectPoints(points3Mat,rvec,tvec,cameraMatrix,distCoeff,points2D);
    mutex->post();

    return points2D;
}

Mat StereoCamera::computeWorldImage(Mat &H)
{

    Mat worldImg(Disparity16.rows,Disparity16.cols,CV_32FC3);

    if(H.empty())
        H=H.eye(4,4,CV_64FC1);

    if(Disparity16.empty() || MapperL.empty() || Q.empty())
    {
        cout <<" Run computeDisparity() method first" << endl;
        return worldImg;
    }


    Mat dispTemp;
    Mat x;
    remap(this->Disparity16,dispTemp,this->MapperL,x,cv::INTER_LINEAR);
    reprojectImageTo3D(dispTemp, worldImg,this->Q,true);

    for(int i=0; i<worldImg.rows; i++)
    {
       for(int j=0; j<worldImg.cols; j++)
        {   
            Mat RLrectTmp=this->getRLrect().t();
            Mat Tfake = Mat::zeros(0,3,CV_64F);
            Mat P(4,1,CV_64FC1);
            if((worldImg.data + worldImg.step * i)[j * worldImg.channels() + 2]>100)
            {
                P.at<double>(0,0)=0.0;
                P.at<double>(1,0)=0.0;
                P.at<double>(2,0)=0.0;
                P.at<double>(3,0)=1.0;
            }
            else
            {
                P.at<double>(0,0)=(worldImg.data + worldImg.step * i)[j * worldImg.channels() + 0];
                P.at<double>(1,0)=(worldImg.data + worldImg.step * i)[j * worldImg.channels() + 1];
                P.at<double>(2,0)=(worldImg.data + worldImg.step * i)[j * worldImg.channels() + 2];
                P.at<double>(3,0)=1;

                Mat Hrect=buildRotTras(RLrectTmp,Tfake);
                P=H*Hrect*P;
            }
            (worldImg.data + worldImg.step * i)[j * worldImg.channels() + 0]=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
            (worldImg.data + worldImg.step * i)[j * worldImg.channels() + 1]=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
            (worldImg.data + worldImg.step * i)[j * worldImg.channels() + 2]=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
        }
    }
    
    return worldImg;
}
Mat StereoCamera::getDistCoeffLeft()
{
    return this->DistL;
}

Mat StereoCamera::getDistCoeffRight()
{
    return this->DistR;
}

void StereoCamera::buildUndistortRemap()
{
    Mat newCam;
    Size img_size = this->imleft.size();
    initUndistortRectifyMap(this->Kleft, this->DistL, Mat::eye(3,3,CV_64FC1), newCam, img_size, CV_32FC1,this->mapxL,this->mapyL);
    initUndistortRectifyMap(this->Kright, this->DistR, Mat::eye(3,3,CV_64FC1), newCam, img_size, CV_32FC1,this->mapxR,this->mapyR);

}

Point2f StereoCamera::getDistortedPixel(int u, int v, int cam)
{
    Point2f distortedPixel;
    Mat MapperX,MapperY;

    if(cam==LEFT)
    {
        MapperX=mapxL;
        MapperY=mapyL;
    }
    else
    {
        MapperX=mapxR;
        MapperY=mapyR;
    }
    distortedPixel.x=MapperX.ptr<float>(v)[u];
    distortedPixel.y=MapperY.ptr<float>(v)[u];

    return distortedPixel;
}
bool StereoCamera::loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &Ro, Mat &T)
{

    Bottle left=rf.findGroup("CAMERA_CALIBRATION_LEFT");
    if(!left.check("fx") || !left.check("fy") || !left.check("cx") || !left.check("cy"))
        return false;

    double fx=left.find("fx").asDouble();
    double fy=left.find("fy").asDouble();

    double cx=left.find("cx").asDouble();
    double cy=left.find("cy").asDouble();

    double k1=left.check("k1",Value(0)).asDouble();
    double k2=left.check("k2",Value(0)).asDouble();

    double p1=left.check("p1",Value(0)).asDouble();
    double p2=left.check("p2",Value(0)).asDouble();

    DistL=Mat::zeros(1,8,CV_64FC1);
    DistL.at<double>(0,0)=k1;
    DistL.at<double>(0,1)=k2;
    DistL.at<double>(0,2)=p1;
    DistL.at<double>(0,3)=p2;
    

    KL=Mat::eye(3,3,CV_64FC1);
    KL.at<double>(0,0)=fx;
    KL.at<double>(0,2)=cx;
    KL.at<double>(1,1)=fy;
    KL.at<double>(1,2)=cy;

    Bottle right=rf.findGroup("CAMERA_CALIBRATION_RIGHT");
    if(!right.check("fx") || !right.check("fy") || !right.check("cx") || !right.check("cy"))
        return false;

    fx=right.find("fx").asDouble();
    fy=right.find("fy").asDouble();

    cx=right.find("cx").asDouble();
    cy=right.find("cy").asDouble();

    k1=right.check("k1",Value(0)).asDouble();
    k2=right.check("k2",Value(0)).asDouble();

    p1=right.check("p1",Value(0)).asDouble();
    p2=right.check("p2",Value(0)).asDouble();

    DistR=Mat::zeros(1,8,CV_64FC1);
    DistR.at<double>(0,0)=k1;
    DistR.at<double>(0,1)=k2;
    DistR.at<double>(0,2)=p1;
    DistR.at<double>(0,3)=p2;
    

    KR=Mat::eye(3,3,CV_64FC1);
    KR.at<double>(0,0)=fx;
    KR.at<double>(0,2)=cx;
    KR.at<double>(1,1)=fy;
    KR.at<double>(1,2)=cy;

    Ro=Mat::zeros(3,3,CV_64FC1);
    T=Mat::zeros(3,1,CV_64FC1);

    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");
    if (Bottle *pXo=extrinsics.find("HN").asList()) {
        for (int i=0; i<(pXo->size()-4); i+=4) {
            Ro.at<double>(i/4,0)=pXo->get(i).asDouble();
            Ro.at<double>(i/4,1)=pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2)=pXo->get(i+2).asDouble();
            T.at<double>(i/4,0)=pXo->get(i+3).asDouble();
        }
    }
    else
        return false;

    return true;
}

void StereoCamera::setMatches(std::vector<cv::Point2f> & pointsL, std::vector<cv::Point2f> & pointsR)
{
    PointsL=pointsL;
    PointsR=pointsR;

}


void StereoCamera::setExpectedPosition(Mat &Rot, Mat &Tran)
{
    R_exp=Rot;
    T_exp=Tran; 
}
