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