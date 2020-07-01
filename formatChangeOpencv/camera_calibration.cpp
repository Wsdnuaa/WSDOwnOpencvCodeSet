#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
      

    Mat img1 = imread("../bb3000.tif",0);
    imwrite("bb3000.pgm",img1);
    Mat img2 = imread("../ff3000.tif",0);
    imwrite("ff3000.pgm",img2);
    



}