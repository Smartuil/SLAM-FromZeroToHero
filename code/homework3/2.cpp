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