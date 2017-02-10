#include "rectify.h"
#include <boost/filesystem.hpp>

using namespace intrinsicExtrinsic;

#include <string>
#include <unordered_map>


int main(){
  Mat rectImg1, rectImg2;
  fsTest("ext.yml", "int.yml", imread("img1.jpg"), imread("img2.jpg"), rectImg1, rectImg2);
  return 1;
}
