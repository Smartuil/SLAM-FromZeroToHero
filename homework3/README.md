# 题目
## 请使用C++新特性改写以下函数。该函数功能：将一组无序的坐标按照“Z”字形排序，并输出。题目如下所示：
```C++
/****************************
* 题目：请使用C++新特性改下以下函数。该函数功能：将一组无序的坐标按照“Z”字形排序，并输出。
*
* 本程序学习目标：
* 熟悉C++新特性（简化循环、自动类型推导、列表初始化、lambda函数）
*
* 作者：公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
* 时间：2018.09
****************************/

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

bool cmp(Point2i pt1, Point2i pt2){

	// --- 开始你的代码 ----//

	// --- 结束你的代码 ----//
}
int main()
{
	vector<Point2i> vec;
	vec.push_back(Point2i(2, 1));
	vec.push_back(Point2i(3, 3));
	vec.push_back(Point2i(2, 3));
	vec.push_back(Point2i(3, 2));
	vec.push_back(Point2i(3, 1));
	vec.push_back(Point2i(1, 3));
	vec.push_back(Point2i(1, 1));
	vec.push_back(Point2i(2, 2));
	vec.push_back(Point2i(1, 2));
	
	cout << "Before sort: " << endl;
	for (int i = 0; i < vec.size(); i++){
		cout << vec[i] << endl;
	}

	sort(vec.begin(), vec.end(), cmp);

	cout << "After sort: " << endl;
	for (int i = 0; i < vec.size(); i++){
		cout << vec[i] << endl;
	}

	return 0;
}
```
![](https://github.com/Smartuil/SLAM-FromZeroToHero/blob/master/images/%E4%BD%9C%E4%B8%9A3_1.png)
```C++
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;
int main()
{
    vector<Point2i> vec{ {2, 1} ,{3, 3} ,{2, 3} ,{3, 2} ,{3, 1} ,{1, 3} ,{1, 1} ,{2, 2} ,
                         {1, 2} };
    cout << "Before sort: " << endl;
    for (auto v : vec) { cout << v << endl; }
    sort(vec.begin(), vec.end(), [=](Point2i pt1, Point2i pt2)->bool {
        return (pt1.x < pt2.x) ? (1) : (pt1.x == pt2.x && pt1.y < pt2.y); });
    cout << "After sort: " << endl;
    for (auto v : vec) { cout << v << endl; }
    return 0;
}
```
```C++
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
bool cmp(Point2i pt1, Point2i pt2)
{
    if(pt1.x < pt2.x) return true;
    if(pt1.x == pt2.x)
    {
        return pt1.y < pt2.y;
    }
    return false;
}
int main()
{
/* initializer, Method 1*/
#if 0
    vector<Point2i> vec;
    vec.push_back(Point2i(2, 1));
    vec.push_back(Point2i(3, 3));
    vec.push_back(Point2i(2, 3));
    vec.push_back(Point2i(3, 2));
    vec.push_back(Point2i(3, 1));
    vec.push_back(Point2i(1, 3));
    vec.push_back(Point2i(1, 1));
    vec.push_back(Point2i(2, 2));
    vec.push_back(Point2i(1, 2));
#endif
/* initializer, Method 2*/
    vector<Point2i> vec = {{2, 1}, {3, 3}, {2, 3}, {3, 2}, {3, 1}, {1, 3}, {1, 1}, {2, 2},
                           {1, 2}};
    cout << "Before sort: " << endl;
//Method 1
#if 0
    for (int i = 0; i < vec.size(); i++){
        cout << vec[i] << endl;
    }
#endif
//Method 2
    for(auto i: vec)
    {
        cout << i << endl;
    }
/* sort, Method 1*/
#if 0
    sort(vec.begin(), vec.end(), cmp);
#endif
/* sort, Method 2: lambda */
    sort(vec.begin(), vec.end(),
         [](Point2i a, Point2i b)-> bool
         {
             if(a.x < b.x) return true;
             if(a.x == b.x)
             {
                 return a.y < b.y;
             }
             return false;
         });
    cout << "After sort: " << endl;
    for (int i = 0; i < vec.size(); i++){
        cout << vec[i] << endl;
    }
    return 0;
}
```