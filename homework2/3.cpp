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
