# 题目
##  SLAM是处理序列图像的，有时候需要格式化的图像名字用作输入。前面提到的TUM的RGB-D数据集中图像是根据时间命名的，请从下面链接下载数据集fr1/desk [Computer Vision Group - Dataset Download ](https://vision.in.tum.de/data/datasets/rgbd-dataset/download#)并解压。请编程实现将文件夹/rgb下以时间命名的序列图片重新命名为0000-9999的格式。
### **知识点**： 熟悉cmake的使用、OpenCV读写操作、C++的string操作
### **友情链接**：https://blog.csdn.net/weixin_40023317/article/details/83752151

``` C++
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
    string dataPath = argv[1]; //"./rgbd_dataset_freiburg1_desk/rgb/"
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
```
#### CMake

```
cmake_minimum_required(VERSION 3.15)
project(homework1)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(homework1 main.cpp)
target_link_libraries(homework1 ${OpenCV_LIBS})
```