#include "dispCloud.h"
#include <boost/filesystem.hpp>

using namespace dispCloud;

#include <string>

int main(){
  fsTest("img1.jpg", "img2.jpg", "config.cfg");
  return 1;
}
