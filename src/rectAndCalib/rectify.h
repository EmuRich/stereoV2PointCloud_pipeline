#ifndef RECTANDCALIB_H
#define RECTANDCALIB_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <libconfig.h++>
#include <unordered_map>
#include <string>

using namespace std;
using namespace cv;

namespace rectAndCalib
{
    void loadStereoExtrinsics(unordered_map<string,Mat> extData, Rect roi1, Rect roi2, Mat &R1, Mat &P1, Mat &R2, Mat &P2, Rect &validRoi);
    void loadStereoIntrinsics(unordered_map<string, Mat> intData1, unordered_map<string, Mat> intData2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &distCoeffs1,  Mat &distCoeffs2);
    void undistortStereoImages(Mat inputImg1, Mat inputImg2, Mat &outputImg1, Mat &outputImg2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &distCoeffs1,  Mat &distCoeffs2);
    void loadIntEntFromFS(string ymlExtrinsics, string ymlIntrinsics, unordered_map<string, Mat> &intData1, unordered_map<string, Mat> &intData2, unordered_map<string, Mat> &extData, Rect &roi1, Rect &roi2);
    void rectifyHandler(Mat inputImg1, Mat inputImg2, unordered_map<string, Mat> intData1, unordered_map<string, Mat> intData2, unordered_map<string, Mat> extData, Rect roi1, Rect roi2, Mat& rectImg1, Mat& rectImg2);
    void fsTest(string ymlExtrinsics, string ymlIntrinsics, Mat imgLeft, Mat imgRight, Mat& rectImg1, Mat& rectImg2);
}

#endif // RECTANDCALIB_H
