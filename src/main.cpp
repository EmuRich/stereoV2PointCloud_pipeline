#include "rectAndCalib/rectify.h"
#include "dispCloud/dispCloud.h"

#include <boost/filesystem.hpp>
#include <string>
#include <unordered_map>


int main(){
  Mat rectImg1, rectImg2;
  rectAndCalib::fsTest("ext.yml", "int.yml", imread("09660_L.jpg"), imread("09660_R.jpg"), rectImg1, rectImg2);
  std::cout << "Finished calibration and rectification";
  dispCloud::dispAndCloudHandler(rectImg1, rectImg2, "config.cfg");
  return 1;  
}
