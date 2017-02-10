#include "dispCloud.h"

bool dispCloud::loadQMatrix(string file, Mat &Q)
{
    bool success = false;
    try
    {
        FileStorage fStorage(file.c_str(), FileStorage::READ);
        fStorage["Q"] >> Q;
        fStorage.release();
        success = true;
    }
    catch(Exception ex)
    {
    }

    return success;
}


void dispCloud::dispAndCloudHandler(Mat rawImgLeft, Mat rawImgRight, string configLink){
    bool readSuccessfully = false;
    bool success = false;
    libconfig::Config cfg;
    string ymlExtrinsic;
    uint dMin; uint dMax; Size censusWin; float defaultBorderCost;
    float lambdaAD; float lambdaCensus; uint aggregatingIterations;
    uint colorThreshold1; uint colorThreshold2; uint maxLength1; uint maxLength2; uint colorDifference;
    float pi1; float pi2; uint dispTolerance; uint votingThreshold; float votingRatioThreshold;
    uint maxSearchDepth; uint blurKernelSize; uint cannyThreshold1; uint cannyThreshold2; uint cannyKernelSize;

    try
    {
        cfg.readFile(configLink.c_str());
        readSuccessfully = true;
    }
    catch(const libconfig::FileIOException &fioex)
    {
        cerr << "[ADCensusCV] I/O error while reading file." << endl;
    }
    catch(const libconfig::ParseException &pex)
    {
        cerr << "[ADCensusCV] Parsing error" << endl;
    }
    if(readSuccessfully)
    {
        try
        {
            dMin = (uint) cfg.lookup("dMin");
            dMax = (uint) cfg.lookup("dMax");
            ymlExtrinsic = (const char *) cfg.lookup("ymlExtrinsic");
            censusWin.height = (uint) cfg.lookup("censusWinH");
            censusWin.width = (uint) cfg.lookup("censusWinW");
            defaultBorderCost = (float) cfg.lookup("defaultBorderCost");
            lambdaAD = (float) cfg.lookup("lambdaAD");
            lambdaCensus = (float) cfg.lookup("lambdaCensus");
            aggregatingIterations = (uint) cfg.lookup("aggregatingIterations");
            colorThreshold1 = (uint) cfg.lookup("colorThreshold1");
            colorThreshold2 = (uint) cfg.lookup("colorThreshold2");
            maxLength1 = (uint) cfg.lookup("maxLength1");
            maxLength2 = (uint) cfg.lookup("maxLength2");
            colorDifference = (uint) cfg.lookup("colorDifference");
            pi1 = (float) cfg.lookup("pi1");
            pi2 = (float) cfg.lookup("pi2");
            dispTolerance = (uint) cfg.lookup("dispTolerance");
            votingThreshold = (uint) cfg.lookup("votingThreshold");
            votingRatioThreshold = (float) cfg.lookup("votingRatioThreshold");
            maxSearchDepth = (uint) cfg.lookup("maxSearchDepth");
            blurKernelSize = (uint) cfg.lookup("blurKernelSize");
            cannyThreshold1 = (uint) cfg.lookup("cannyThreshold1");
            cannyThreshold2 = (uint) cfg.lookup("cannyThreshold2");
            cannyKernelSize = (uint) cfg.lookup("cannyKernelSize");
        }
        catch(const libconfig::SettingException &ex)
        {
            cerr << "[ADCensusCV] " << ex.what() << endl
                 << "config file format:\n"
                    "dMin(uint)\n"
                    "ymlExtrinsic(string)\n"
                    "censusWinH(uint)\n"
                    "censusWinW(uint)\n"
                    "defaultBorderCost(float)\n"
                    "lambdaAD(float)\n"
                    "lambdaCensus(float)\n"
                    "aggregatingIterations(uint)\n"
                    "colorThreshold1(uint)\n"
                    "colorThreshold2(uint)\n"
                    "maxLength1(uint)\n"
                    "maxLength2(uint)\n"
                    "colorDifference(uint)\n"
                    "pi1(float)\n"
                    "pi2(float)\n"
                    "dispTolerance(uint)\n"
                    "votingThreshold(uint)\n"
                    "votingRatioThreshold(float)\n"
                    "maxSearchDepth(uint)\n"
                    "blurKernelSize(uint)\n"
                    "cannyThreshold1(uint)\n"
                    "cannyThreshold2(uint)\n"
                    "cannyKernelSize(uint)\n";
            readSuccessfully = false;
        }
    }

    if(readSuccessfully)
    {
        Mat Q(4, 4, CV_64F);
        bool gotExtrinsic = loadQMatrix(ymlExtrinsic, Q);
        bool error = false;
    
        boost::posix_time::ptime start = boost::posix_time::second_clock::local_time();
        boost::posix_time::ptime end;
        boost::posix_time::time_duration diff;

        ImageProcessor iP(0.1);
        Mat eLeft, eRight;
        eLeft = iP.unsharpMasking(rawImgLeft, "gauss", 3, 1.9, -1);
        eRight = iP.unsharpMasking(rawImgRight, "gauss", 3, 1.9, -1);

        StereoProcessor sP(dMin, dMax, rawImgLeft, rawImgRight, censusWin, defaultBorderCost, lambdaAD, lambdaCensus, "savey", aggregatingIterations, colorThreshold1, colorThreshold2, maxLength1, maxLength2, colorDifference, pi1, pi2, dispTolerance, votingThreshold, votingRatioThreshold, maxSearchDepth, blurKernelSize, cannyThreshold1, cannyThreshold2, cannyKernelSize);

        string errorMsg;
        error = !sP.init(errorMsg);

        if(!error && sP.compute())
        {
            success = true;
            if(gotExtrinsic)
            {
                Mat disp = sP.getDisparity();
                FileStorage fs("_disp.yml", FileStorage::WRITE);
                fs << "disp" << disp;
                fs.release();
                // createAndSavePointCloud(disp, rawImgLeft, Q, "point_cloud.pcd");
            }
            else
            {
                cerr << "[ADCensusCV] Could not create point cloud (no extrinsic)!" << endl;
            }
        }
        else
        {
            cerr << "[ADCensusCV] " << errorMsg << endl;
        }
        end = boost::posix_time::second_clock::local_time();
        diff = end - start;
        cout << "Finished computation after " << setw(2) << right <<  setfill('0') << ((int)(diff.total_seconds() / 3600)) << ":" << setw(2) << right <<  setfill('0') << ((int)((diff.total_seconds() / 60) % 60)) << ":" << setw(2) << right <<  setfill('0') << (diff.total_seconds() % 60) << " (" << diff.total_seconds() << "s) !" << endl;
    }
    
    return;
}

void dispCloud::fsTest(string img1LeftURL, string img2RightURL, string cfgFile){
  Mat imgLeft = imread(img1LeftURL);
  Mat imgRight = imread(img2RightURL);

  dispAndCloudHandler(imgLeft, imgRight, cfgFile);
  return;
}