#include<iostream>
#include<thread>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include<chrono>
#include<mutex>

std::mutex mtk;

cv::Mat frame1;
cv::Mat frame2;
void readVideo1(cv::Mat frame)
{   
    cv::VideoCapture* cap =new cv::VideoCapture;
    cap->open("../data/myDog.mp4");
// cv::Mat frame;
    while(1)
    {   
        cap->read(frame1);
        if(frame1.empty())
            break;
        std::cout<<"fuck :"<<frame1.size()<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
       // cv:: waitKey(30);
       // std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
}

void readvideo2(cv::Mat frame)
{
    cv::VideoCapture cap ("../data/myDog1.mp4");
    //cv::Mat frame;
    while(1)
    {
        //std::unique_lock<std::mutex> lck(mtk);
        cv::Mat temp;
        cvtColor(frame1, temp, 7);
        cv::Canny(temp, frame2, 200, 130);
        //frame2 = frame1.clone();
        
       std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
       //mtk.unlock();
    }    
}
 

int main()
{
    //申明std::thread
    //);
    cv::namedWindow("lovelydog");
    cv::namedWindow("mydog");
    std::thread videoCap1(readVideo1,frame1);
    //videoCap1.detach();
    std::thread videoCap2(readvideo2,frame2);
    //videoCap2.detach();   
    // videoCap2.join();
    // videoCap1.join();
    std::cout<<"Exit of Main function"<<std::endl;
    //videoCap2.join();
    //videoCap1.join();
    // while(1){
    //     std::cout<<"x"<<std::endl;
    // }
    while(1){
        cv::waitKey(10);
        if(!frame1.empty())
             cv::imshow("mydog",frame1);
        if(!frame2.empty())
             cv::imshow("lovelydog",frame2);
        //std::cout<<"wsd"<<frame1.size()<<std::endl;
        cv::waitKey(10);
        if(frame1.empty()&&frame2.empty())
            break;

    }
    videoCap2.join();
    videoCap1.join();
    return(0);
}