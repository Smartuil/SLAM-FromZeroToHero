# 题目
## 已知相机的位姿用四元数表示为q=[0.35,0.2,0.3,0.1],顺序为x,y,z,w，请编程实现：输出四元数对应的旋转矩阵、旋转矩阵的转置，旋转矩阵的逆矩阵，旋转矩阵乘以自身的转置，验证旋转矩阵的正交性。
### **知识点**：熟悉cmake的使用、学习eigen的基本操作；根据实践验证旋转矩阵的约束
```C++
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "iostream"

using namespace std;
int main( int argc, char** argv )
{
    Eigen::Quaterniond quat = Eigen::Quaterniond(0.1,0.35, 0.2, 0.3);
    quat.normalize();
    Eigen::Matrix3d rotation_matrix = quat.matrix();
    cout<<"旋转矩阵：r = \n"<<rotation_matrix <<endl;
    cout<<"旋转矩阵转置后：rt = \n"<<rotation_matrix.transpose() <<endl;
    cout<<"旋转矩阵的逆矩阵：r.inv = \n"<<rotation_matrix.inverse() <<endl;
    cout<<"旋转矩阵乘以自身的转置：r*rt =\n"<<rotation_matrix * rotation_matrix.transpose()<<endl;
    return 0;
}
```
```C++
//
// Created by smartuil on 2020/3/21.
//
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iomanip>
using namespace std;
using namespace Eigen;
/* 相机位姿用四元数表示 q = [0.35, 0.2, 0.3, 0.1] x,y,z.w
* 注意：Quaterniond（w,x,y,z） W 在前!!!
* 实现：输出四元素对应的旋转矩阵，旋转矩阵的转置，
* 旋转矩阵的逆矩阵，旋转矩阵乘以自身的转置，验证旋转矩阵的正交性
*/
int main(int argc, char** argv) {
    cout << "作业2:eigen运算" << endl;
    Quaterniond q = Quaterniond(0.1, 0.35, 0.2, 0.3).normalized();
    Matrix3d matrix_T = q.toRotationMatrix();
    cout << "四元数的旋转矩阵：" << endl << matrix_T << endl;
    Matrix3d matrix_transposeT = matrix_T.transpose();
    cout << "旋转矩阵的转置：" << endl << matrix_transposeT << endl;
    Matrix3d matrix_invT = matrix_T.inverse();
    cout << "旋转矩阵的逆矩阵：" << endl << matrix_invT << endl;
    Matrix3d matrix_T1 = matrix_T * matrix_transposeT ;
    cout << "旋转矩阵乘以自身的转置:" << endl << matrix_T1 << endl;
// 验证旋转矩阵的正交性用定义:直接计算 AA^T, 若 等于单位矩阵E, 就是正交矩阵
    cout.setf(ios::fixed);
    cout << "验证旋转矩阵的正交性:" << endl
         << fixed << setprecision(5) << matrix_T1 << endl;
    cout.unsetf(ios::fixed);
    cout << "matrix_T * matrix_transposeT 是单位矩阵，即旋转矩阵是正交矩阵";
    return 0;
}
```
```C++
//
// Created by smartuil on 2020/3/21.
//
#include<Eigen/Core>
#include<Eigen/Geometry>
#include "iostream"

using namespace std;
int main( int argc, char** argv)
{
    Eigen::Quaterniond quat = Eigen::Quaterniond(0.1, 0.35, 0.2,
                                                 0.3).normalized();
    //（1）旋转矩阵
    Eigen::Matrix3d R_from_q = quat.toRotationMatrix();
    cout<<"Rotation Matrix:\n"<<R_from_q<<endl;
    //（2）矩阵的转置
    cout<<"The transpose of R: \n"<<R_from_q.transpose()<<endl;
    // (3) 矩阵的逆矩阵
    cout<<"The inverse of R: \n" <<R_from_q.inverse()<<endl;
    //（4）乘以自身的转置
    cout<<"R*R.T = \n"<<R_from_q * R_from_q.transpose() << endl;
    //————————结束我的表演————————
    return 0;
}
```
#### CMake
```
cmake_minimum_required(VERSION 3.15)
project(homework2)

set(CMAKE_CXX_STANDARD 14)

include_directories( "/usr/local/include/eigen3" )

add_executable(1 1.cpp)
add_executable(2 2.cpp)
add_executable(3 3.cpp)
```
