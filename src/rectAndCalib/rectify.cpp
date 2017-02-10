#include "rectify.h"

void rectAndCalib::loadStereoExtrinsics(unordered_map<string, Mat> extData, Rect roi1, Rect roi2, Mat &R1, Mat &P1, Mat &R2, Mat &P2, Rect &validRoi){
    R1 = extData["R1"];
    P1 = extData["P1"];
    R2 = extData["R2"];
    P2 = extData["P2"];

    Point2i rectCorner1(max(roi1.x, roi2.x), max(roi1.y, roi2.y));

    Point2i rectCorner2(min(roi1.x + roi1.width, roi2.x + roi2.width), min(roi1.y + roi1.height, roi2.y + roi2.height));

    validRoi = Rect(rectCorner1.x, rectCorner1.y,
             rectCorner2.x - rectCorner1.x, rectCorner2.y - rectCorner1.y);

    return;
}

void rectAndCalib::loadStereoIntrinsics(unordered_map<string, Mat> intData1, unordered_map<string, Mat> intData2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &distCoeffs1,  Mat &distCoeffs2){
    cameraMatrix1 = intData1["Camera Matrix"];
    cameraMatrix2 = intData2["Camera Matrix"];

    distCoeffs1 = intData1["Distortion Coefficients"];
    distCoeffs2 = intData2["Distortion Coefficients"];

    return;
}


void rectAndCalib::undistortStereoImages(Mat inputImg1, Mat inputImg2, Mat &outputImg1, Mat &outputImg2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &distCoeffs1,  Mat &distCoeffs2){

    undistort(inputImg1, outputImg1, cameraMatrix1, distCoeffs1);
    undistort(inputImg2, outputImg2, cameraMatrix2, distCoeffs2);

    return;
}


void rectAndCalib::loadIntEntFromFS(string ymlExtrinsics, string ymlIntrinsics, unordered_map<string, Mat> &intData1, unordered_map<string, Mat> &intData2, unordered_map<string, Mat> &extData, Rect &roi1, Rect &roi2){

    FileStorage fStorage(ymlExtrinsics.c_str(), FileStorage::READ);

    if (fStorage.isOpened()){
        fStorage["R1"] >> extData["R1"];
        fStorage["P1"] >> extData["P1"];
        fStorage["R2"] >> extData["R2"];
        fStorage["P2"] >> extData["P2"];
        fStorage["roi1"] >> roi1;
        fStorage["roi2"] >> roi2;

        fStorage.release();
    }

    FileStorage fStorage1(ymlIntrinsics.c_str(), FileStorage::READ);

    if (fStorage1.isOpened()){
        fStorage1["M1"] >> intData1["Camera Matrix"];
        fStorage1["D1"] >> intData1["Distortion Coefficients"];

        fStorage1["M2"] >> intData2["Camera Matrix"];
        fStorage1["D2"] >> intData2["Distortion Coefficients"];
        fStorage1.release();
    }
    return;
}

void rectAndCalib::rectifyHandler(Mat inputImg1, Mat inputImg2, unordered_map<string, Mat> intData1, unordered_map<string, Mat> intData2, unordered_map<string, Mat> extData, Rect roi1, Rect roi2, Mat& rectImg1, Mat& rectImg2)
{
    Mat undistorted1, undistorted2, cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2;
    Mat noDist = Mat::zeros(5,1, CV_32F);
    Size imageSize;
    Mat R1, R2, P1, P2;
    Rect validRoi;

    imageSize.height = (inputImg1).rows;
    imageSize.width = (inputImg2).cols;
    
    cout << "Loading intrinsics!" << endl;
    loadStereoIntrinsics(intData1, intData2,
                         cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2);    
    cout << "Intrinsics loading complete" << endl;


    cout << "Loading extrinsics!" << endl;
    loadStereoExtrinsics(extData, roi1, roi2, R1, P1, R2, P2, validRoi);
    cout << "Extrinsics loading complete" << endl;

    cout << "Undistorting images!" << endl;
    undistortStereoImages(inputImg1, inputImg2, undistorted1, undistorted2, cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2);
    cout << "Image undistortion complete" << endl;

    Mat rmap[2][2];
    initUndistortRectifyMap(cameraMatrix1, noDist, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix2, noDist, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat img1 = undistorted1, remapImg1;
    remap(img1, remapImg1, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
    rectImg1 = remapImg1(validRoi);

    Mat img2 = undistorted2, remapImg2;
    remap(img2, remapImg2, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
    rectImg2 = remapImg2(validRoi);

    // This is for debug only
    //--------------------------------------------------------------------------- 
    // imwrite("rectimg1.jpg", rectImg1);
    // imwrite("rectimg2.jpg", rectImg2);
    
    return;
}

void rectAndCalib::fsTest(string ymlExtrinsics, string ymlIntrinsics, Mat imgLeft, Mat imgRight, Mat& rectImg1, Mat& rectImg2){
    unordered_map<string, Mat> intData1, intData2, extData;
    Rect roi1, roi2;

    loadIntEntFromFS(ymlExtrinsics, ymlIntrinsics, intData1, intData2, extData, roi1, roi2);

    rectifyHandler(imgLeft, imgRight, intData1, intData2, extData, roi1, roi2, rectImg1, rectImg2);

    return;
}