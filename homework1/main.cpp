#include "opencv2/opencv.hpp"
#include <iostream>
#include "sys/io.h"
#include <unistd.h>
#include<sys/stat.h>
using namespace cv;
using namespace std;
void CreatDirectory(string outDir){
    if (access(outDir.c_str(), 0) == -1)
    {
        std::cout << outDir << " is not existing" << std::endl;
        std::cout << "now make it" << std::endl;
        int flag = mkdir(outDir.c_str(),777);
        if (flag == 0)
        {
            std::cout << "make successfully" << std::endl;
        }
        else {
            std::cout << "make errorly" << std::endl;
        }
    }
}
string int2paddingString(int num){ // 1--0001
    string snum = "";
// padding 1 to 0001
    if (num<10) {
        snum = snum + "000" + std::to_string(num);
    }
    else if (num < 100){
        snum = snum + "00" + std::to_string(num);
    }
    else if (num < 1000){
        snum = snum + "0" + std::to_string(num);
    }
    else {
        snum = std::to_string(num);
    }
    return snum;
}
int main(int argc, char* argv[])
{
    string dataPath = "../rgbd_dataset_freiburg1_desk/rgb/"; //"./rgbd_dataset_freiburg1_desk/rgb/"
    dataPath = dataPath + "/";
    vector<String> imageNames;
    glob(dataPath, imageNames, false);
    for (int num = 0; num < imageNames.size(); num++){
        Mat colorMat = cv::imread(imageNames[num]);
        if (colorMat.empty()) // check
        {
            cout << "Can't open color image!" << imageNames[num] << endl;
        }
        string colorIndex = dataPath + "rgbIndex/";
        CreatDirectory(colorIndex); //creat directory if not exist
        string padNum = int2paddingString(num);
        cv::imwrite(colorIndex + padNum + ".png", colorMat);
        num++;
        std::cout << "\r[ processing frame " << std::to_string(num) << " ]";
    }
    std::cout << "All done :)" << std::endl;
    return 0;
}