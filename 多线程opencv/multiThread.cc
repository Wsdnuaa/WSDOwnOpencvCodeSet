// pthreadDemo.cpp : 定义控制台应用程序的入口点。
//
 
#include <stdio.h>
#include <pthread.h>
#include <assert.h>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;
#define THREAD_NUMS 3//分成四块
 
/*paramThread用于传递线程需要的参数值*/
struct paramThread
{
	int w;
	int h;
	uchar * data;
};
 
/********************************************************
*	@brief       : 多线程处理函数
*	@param  args : 多线程传入的参数
*	@return      : void
********************************************************/
void * threadProcess(void* args) {
 
	pthread_t myid = pthread_self();
	paramThread *para = (paramThread *)args;
	int w = para->w;
	int h = para->h;
	cv::Mat image(h,w,CV_8UC3,(uchar *)para->data);
	//cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
	cv::blur(image, image,cv::Size(7,7), cv::Point(-1, -1), cv::BORDER_REPLICATE);
	//printf("thread id = %d, w=%d, h=%d\n", myid, w,h);
	//cv::imshow("image", image); cv::waitKey(2000);
	pthread_exit(NULL);
	return NULL;
}
 
/********************************************************
*	@brief       : 实现图像分割，
*	@param  num  :  分割个数
*	@param  type : 0：垂直分割(推荐)，1：水平分割（不推荐）
*	@return      : vector<cv::Mat>
*   PS：使用水平分割时（type=1），处理完后必须调用catImage进行拼接，
*   使用垂直分割时（type=0），可以不进行catImage，因为是对原图进行操作的
********************************************************/
vector<cv::Mat> splitImage(cv::Mat image, int num,int type) {
	int rows = image.rows;
	int cols = image.cols;
	vector<cv::Mat> v;
	if (type == 0) {//垂直分割
		for (size_t i = 0; i < num; i++) {
			int star = rows / num*i;
			int end = rows / num*(i + 1);
			if (i == num - 1) {
				end = rows;
			}
			//cv::Mat b = image.rowRange(star, end);
			v.push_back(image.rowRange(star, end));
		}
	}
	else if (type == 1) {//水平分割
		for (size_t i = 0; i < num; i++){
			int star = cols / num*i;
			int end = cols / num*(i + 1);
			if (i == num - 1) {
				end = cols;
			}
			//cv::Mat b = image.colRange(star, end);
			/*解决水平分割的Bug:必须clone()*/
			v.push_back(image.colRange(star, end).clone());
		}
	}
	return  v;
}
 
/********************************************************
*	@brief       : 实现图像拼接，
*	@param  v    :  
*	@param  type : 0：垂直拼接，1：水平拼接
*	@return      : Mat
********************************************************/
cv::Mat catImage(vector<cv::Mat> v, int type) {
	cv::Mat dest= v.at(0);
	for (size_t i = 1; i < v.size(); i++)
	{
		if (type == 0)//垂直拼接
		{
			cv::vconcat(dest, v.at(i), dest);
		}
		else if (type == 1)//水平拼接
		{
			cv::hconcat(dest, v.at(i), dest);
		}
	}
	return dest;
}
 
int main() {
	string path = "../1.jpg";
	cv::Mat src = cv::imread(path);
	printf("image size =  w=%d, h=%d\n", src.cols, src.rows);
 
	cv::Mat image1 = src.clone();
	cv::Mat image2 = src.clone();
	cv::imshow("src", src); cv::waitKey(30);
 
	double T0 = static_cast<double>(cv::getTickCount());
	/*不使用多线程图像处理*/
	cv::blur(image1, image1, cv::Size(7, 7));
	double T1 = static_cast<double>(cv::getTickCount());
 
	/*使用多线程图像处理*/
	int type = 0;
	vector<cv::Mat> v = splitImage(image2, THREAD_NUMS, type);
	paramThread args[THREAD_NUMS];
	pthread_t pt[THREAD_NUMS];	//创建THREAD_NUMS个子线程
	for (size_t i = 0; i < THREAD_NUMS; i++)
	{
		args[i].h = v.at(i).rows;
		args[i].w = v.at(i).cols;
		args[i].data = v.at(i).data;
		pthread_create(&pt[i], NULL, &threadProcess, (void *)(&args[i]));//(void)无类型指针
	}
	/*等待全部子线程处理完毕*/
	for (size_t i = 0; i < THREAD_NUMS; i++)
	{
		pthread_join(pt[i], NULL);
	}
	cv::Mat dest = catImage(v, type);
	double T2 = static_cast<double>(cv::getTickCount());
	printf("       run times = %3.3fms\n", (T1 - T0)*1000 / cv::getTickFrequency());
	printf("Thread run times = %3.3fms\n,", (T2 - T1)*1000 / cv::getTickFrequency());
 
	cv::imshow("dest", dest); cv::waitKey(30);
	cv::imshow("image2", image2); cv::waitKey(30);
 
 
	cv::waitKey(0);
	return 0;
}