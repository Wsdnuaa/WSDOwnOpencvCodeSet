/*
 * gdal_image.cpp -- Load GIS data into OpenCV Containers using the Geospatial Data Abstraction Library
*/
// OpenCV Headers
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
// C++ Standard Libraries
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace std;
using namespace cv;

/*
 * Main Function
*/
int main( int argc, char* argv[] ){
    /*
     * Check input arguments
    */
    if( argc < 2 ){
        cout << "usage: " << argv[0] << " <image_name> " << endl;
        return -1;
    }
    // load the image (note that we don't have the projection information.  You will
    // need to load that yourself or use the full GDAL driver.  The values are pre-defined
    // at the top of this file
    cv::Mat image = cv::imread(argv[1], IMREAD_LOAD_GDAL | IMREAD_COLOR | IMREAD_ANYDEPTH );
        if (!image.data) //或者image.empty()
    {
        cout  << "  cannot open!" << endl;
        return -1;
    }
    cout << image.channels()<<" x " <<image.rows<<" x "<<image.cols<< endl; 
    resize(image,image,Size(5000,10000));
    cout<<"image size :"<<image.size();
    cout<<"image rows cols:"<<image.rows<<"  "<<image.cols<<endl;
   Mat eightMat(image.size(),CV_8UC1);
    for(int i = 0;i<image.rows;i++){
        for(int j= 0;j<image.cols;j++)
    {
       // cout<<dst.at<ushort>(i,j)<<endl;
        int n=(image.at<ushort>(i,j)-490)/2;
        //cout<<n<<endl;
        eightMat.at<uchar>(i,j)=n;

        //out<<eightMat.at<uchar>(i,j)<<endl;
    }
    }
    
    //     for(int i = 0;i<1000;i++){
    //     for(int j= 0;j<1000;j++)
    // {
    //    // cout<<dst.at<ushort>(i,j)<<endl;
    //     int n=image.at<ushort>(i,j)-512;
    //     cout<<n<<endl;
    //     eightMat.at<uchar>(i,j)=n;

    //     //out<<eightMat.at<uchar>(i,j)<<endl;
    // }
    // }
    // Rect rect(0,0,5000,5000);
	// Mat roi = eightMat(rect);
	// imshow("roi", roi);
    //imwrite("left.tif",roi );
    imwrite("my.tif",eightMat );
    cv::namedWindow("Gdal",0);
    cv::imshow("Gdal",eightMat);
    cv::waitKey();
    return 0;
}
