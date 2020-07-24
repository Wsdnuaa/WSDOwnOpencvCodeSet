#include<iostream>
#include <fstream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
bool LoadImages_euroc(const string &strPathToSequence, vector<string> &vstrImageLeft,
		vector<string> &vstrImageRight, vector<double> &vTimestamps)
	{
		ifstream fTimes;
		string strPathTimeFile = strPathToSequence + "/cam0/data.csv";
		string strPrefixLeft = strPathToSequence + "/cam0/data/";
		string strPrefixRight = strPathToSequence + "/cam1/data/";

		fTimes.open(strPathTimeFile.c_str());
		string s;
		getline(fTimes, s);
		while (!fTimes.eof())
		{
			string s;
			getline(fTimes, s);
			if (!s.empty())
			{
				int index = s.find_first_of(",");
				string t = s.substr(0, index);

				vTimestamps.push_back(stod(t) / 10.0e8);
				vstrImageLeft.push_back(strPrefixLeft + t + ".png");
				vstrImageRight.push_back(strPrefixRight + t + ".png");
			}
		}
	}
    int main(){
        string dataFile = "/home/indemind/Documents/dataSet/euroc/mav0";
        vector<string> vstrImageLeft;
        vector<string> vstrImageRight;
        vector<double> vTimestamps;
        LoadImages_euroc(dataFile, vstrImageLeft,vstrImageRight,vTimestamps);
        cout<<vstrImageLeft.size()<<endl;
        Mat frame;
        for(auto imgLeft :vstrImageRight){
            frame  = imread(imgLeft,1);
            if(!frame.empty())
                imshow("left" ,frame);
            waitKey(2);

        }
        return 0;
    }