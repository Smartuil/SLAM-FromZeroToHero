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
    cout << "验证旋转矩阵的正交性:" << endl << fixed << setprecision(5) << matrix_T1 << endl;
    cout.unsetf(ios::fixed);
    cout << "matrix_T * matrix_transposeT 是单位矩阵，即旋转矩阵是正交矩阵";
    return 0;
}