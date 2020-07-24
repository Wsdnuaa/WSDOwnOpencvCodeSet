#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include<vector>
#include<map>
using namespace cv;
using namespace std;
enum adaptiveMethod{meanFilter,gaaussianFilter,medianFilter};
void myAdaptiveThreshold( Mat src , Mat dst, double maxValue,
                            int method, int type, int blockSize, double delta );
void AdaptiveThreshold(cv::Mat& src, cv::Mat& dst, double Maxval, int Subsize, double c, adaptiveMethod method );


int main(){

	cv::Mat threshold_output;
	cv::Mat frameMat = imread("3.jpg");
	cv::Mat grayImg;// = imread("1.png",0);
	
	if(frameMat.channels() != 1)
		cvtColor(frameMat, grayImg, COLOR_RGB2GRAY);
	//auto oriSize = frameMat.size();
	resize(grayImg,grayImg,Size(1920, 1080));
	GaussianBlur(grayImg,grayImg,Size(5,5),0,0);
	//threshold(grayImg, grayImg, 0, 255,THRESH_OTSU);
	//threshold(grayImg, grayImg, 100, 250,THRESH_BINARY);
	Mat grayImgAd = grayImg.clone();
	myAdaptiveThreshold(grayImg,grayImgAd,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,11,10);
	Mat grayImgOtsu;
	threshold(grayImg, grayImgOtsu, 0, 255,THRESH_OTSU);
	Mat grayImgAdOpencv = grayImg.clone();
	AdaptiveThreshold(grayImg,grayImgAdOpencv, 255, 25, 3, gaaussianFilter);
	//resize(grayImg,grayImg,oriSize);
	namedWindow("Ad",0);
	imshow("Ad",grayImgAd);
	namedWindow("otsu",0);
	imshow("otsu",grayImgOtsu);
	namedWindow("Ado",0);
	imshow("Ado",grayImgAdOpencv);
	waitKey();
}


void myAdaptiveThreshold( Mat src , Mat dst, double maxValue,
                            int method, int type, int blockSize, double delta )
{

   // Mat src = _src.getMat();
    Size size = src.size();
	Mat downSampleSrc;
	const int b = 5;
	resize(src,downSampleSrc,Size(0, 0),0.2,0.2);
    // _dst.create( size, src.type() );
    // Mat dst = _dst.getMat();

    if( maxValue < 0 )
    {
        dst = Scalar(0);
        return;
    }
	//一个均值mat
    Mat mean;

    if( downSampleSrc.data != dst.data )
        mean = dst;
	//判断使用何种滤波方式,均值滤波||高斯滤波
    if (method == ADAPTIVE_THRESH_MEAN_C)
        boxFilter( downSampleSrc, mean, downSampleSrc.type(), Size(blockSize, blockSize),
                   Point(-1,-1), true, BORDER_REPLICATE );
    else if (method == ADAPTIVE_THRESH_GAUSSIAN_C)
    {
        Mat srcfloat,meanfloat;
        downSampleSrc.convertTo(srcfloat,CV_32F);
        meanfloat=srcfloat;
        GaussianBlur(srcfloat, meanfloat, Size(blockSize, blockSize), 0, 0, BORDER_REPLICATE);
        meanfloat.convertTo(mean, downSampleSrc.type());
    }
    else

    int i, j;
	//溢出保护,如果超过255就置为255
    uchar imaxval = saturate_cast<uchar>(maxValue);//此处一般设置位255
	// 	函数cvRound，cvFloor，cvCeil 都是用一种舍入的方法将输入浮点数转换成整数：
	// cvRound 返回跟参数最接近的整数值；
	// cvFloor 返回不大于参数的最大整数值；
	// cvCeil 返回不小于参数的最小整数值。
    int idelta = type == THRESH_BINARY ? cvCeil(delta) : cvFloor(delta);
    uchar tab[768];
	//创造一个hash表
    if( type == CV_THRESH_BINARY )
        for( auto i = 0; i < 768; i++ )
            tab[i] = (uchar)(i - 255 > -idelta ? imaxval : 0);
    else if( type == CV_THRESH_BINARY_INV )
        for( auto i = 0; i < 768; i++ )
            tab[i] = (uchar)(i - 255 <= -idelta ? imaxval : 0);
    else
        //CV_Error( CV_StsBadFlag, "Unknown/unsupported threshold type" );
	//如果图像;连续,就整成一个向量
    if( src.isContinuous() && mean.isContinuous() && dst.isContinuous() )
    {
        size.width *= size.height;
        size.height = 1;
    }
	resize(mean,mean,Size(0, 0),b,b);
    for( auto i = 0; i < size.height; i++ )
    {
        const uchar* sdata = src.ptr(i);
        const uchar* mdata = mean.ptr(i);
        uchar* ddata = dst.ptr(i);

        for( auto j = 0; j < size.width; j++ )
            ddata[j] = tab[sdata[j] - mdata[j] + 255];
    }
}




void AdaptiveThreshold(cv::Mat& src, cv::Mat& dst, double Maxval, int Subsize, double c, adaptiveMethod method = meanFilter){
 
	if (src.channels() > 1)
		cv::cvtColor(src, src, CV_RGB2GRAY);
	Mat downSampleSrc;
	const int b = 5;
	resize(src,downSampleSrc,Size(0, 0),0.2,0.2);
	cv::Mat smooth;
	switch (method)
	{
	case  meanFilter:
		cv::blur(downSampleSrc, smooth, cv::Size(Subsize, Subsize));  //均值滤波
		break;
	case gaaussianFilter:
		cv::GaussianBlur(downSampleSrc, smooth, cv::Size(Subsize, Subsize),0,0); //高斯滤波
		break;
	case medianFilter:
		cv::medianBlur(downSampleSrc, smooth, Subsize);   //中值滤波
		break;
	default:
		break;
	}
 
	smooth = smooth - c;
	resize(smooth,smooth,Size(0, 0),b,b);
	dst = src.clone();
	cout<<dst.cols<<" , "<<dst.rows<<endl;
	for (int i = 0; i < dst.rows;++i){
		const uchar* srcptr = src.ptr<uchar>(i);
		const uchar* smoothptr = smooth.ptr<uchar>(i);
		uchar* dstptr = dst.ptr<uchar>(i);
		for (int j = 0; j < dst.cols; ++j){
			if (srcptr[j]>smoothptr[j]){
				dstptr[j] = Maxval;
			}
			else
				dstptr[j] = 0;
		}
	}
 
}
