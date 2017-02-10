#ifndef DISPCLOUD_H
#define DISPCLOUD_H

#include <iostream>
#include "thirdParty/stereoprocessor.h"
#include <libconfig.h++>
#include "thirdParty/imageprocessor.h"
#include <cstdlib>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <iomanip>


using namespace std;
using namespace cv;

namespace dispCloud
{
    bool loadQMatrix(string file, Mat &Q);
    void dispAndCloudHandler(Mat rawImgLeft, Mat rawImgRight, string configLink);
    void fsTest(string img1LeftURL, string img2RightURL, string cfgFile);
}

#endif