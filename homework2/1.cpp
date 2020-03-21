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