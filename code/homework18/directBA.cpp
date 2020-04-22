/************************************
 * 直接法BA
 *
 * @Author: Johnson
 * @date: 2020.04.01
 *
 * *****************************************/

#include <iostream>
#include <string.h>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <pangolin/pangolin.h>

#include <boost/format.hpp>

#define LOG_INFO(X) cout << "[INFO] " << X << endl
#define LOG_ERROR(X) cout << "[ERROR] " << X << endl

using namespace std;
using namespace Eigen;

typedef vector<Sophus::SE3d> VecSE3d;
typedef vector<Eigen::Vector3d> VecVec3d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 16, 1> Vector16d;

// 相机内参
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

Matrix3d K;

inline bool bInImage(float u, float v, int w, int h)
{
    if(u>=0 && u<w && v>=0 && v<h)
        return true;
    else
        return false;
}

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    if (x < 0 || x >= img.cols || y < 0 || y >= img.rows) return 0;

    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

// 获得原始图片的灰度矩阵
Vector16d get_gray_matrix(cv::Mat img, double ix, double iy) {
    int x = floor(ix);
    int y = floor(iy);
    if (x < 0 || x >= img.cols || y < 0 || y >= img.rows) return Vector16d::Zero();

    int x_down = max(0, x - 2);
    int y_down = max(0, y - 2);
    int x_up = min(img.cols, x + 2);
    int y_up = min(img.rows, y + 2);

    Matrix4d res = Matrix4d::Zero();
    int m_x_down = max(2 - x, 0);
    int m_y_down = max(2 - y, 0);
    int m_x_up = (x+2)>img.cols?min(4 + x - img.cols, 4):4;
    int m_y_up = (y+2)>img.rows?min(4 + y - img.rows, 4):4;

    int m_x = m_x_down;
    int m_y = m_y_down;
    for (int i=x_down;i<x_up;i++) {
        m_y = m_y_down;
        for (int j=y_down;j<y_up;j++) {
            res(m_y, m_x) = img.at<uchar>(j, i);
            m_y++;
        }
        m_x++;
    }

    Vector16d res_vec;
    int index = 0;
    for (int i=0;i<4;i++) {
        for (int j=0;j<4;j++) {
            res_vec(index++, 0) = res(j, i);
        }
    }

    return res_vec;
}

// 画图函数，直接拷贝
void Draw(const VecSE3d&, const VecVec3d&);

// 文本预处理函数
void preprocess(string& line) {
    if (line == "") return ;

    line.erase(0, line.find_first_not_of(" "));
    line.erase(line.find_last_not_of(" "));
}

// 顶点类，待优化位姿
class PoseVertex : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PoseVertex() {}

    virtual bool read(istream& in) override {}
    virtual bool write(ostream& out) const override {}

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double* update) override {
        Vector6d update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }
};

// 边类，重投影灰度误差
class EdgeGrayProjection : public g2o::BaseBinaryEdge<16, Vector16d, PoseVertex, g2o::VertexSBAPointXYZ> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeGrayProjection(cv::Mat img) { //, Vector16d gray_value) {
        this->img = img;
        // this->gray_value = gray_value;
    }

    virtual bool read(istream& in) override {}
    virtual bool write(ostream& out) const override {}

    virtual void computeError() override {
        PoseVertex* v_0 = static_cast<PoseVertex*>(vertex(0));
        g2o::VertexSBAPointXYZ* v_1 = static_cast<g2o::VertexSBAPointXYZ*>(vertex(1));

        Sophus::SE3d T = v_0->estimate();
        Vector3d p3d = v_1->estimate();

        Vector3d pos_pixel = T * p3d;

        float u = pos_pixel[0] / pos_pixel[2] * fx + cx;
        float v = pos_pixel[1] / pos_pixel[2] * fy + cy;

        if(!bInImage(u-3,v-3,img.cols,img.rows) || !bInImage(u+2,v+2,img.cols,img.rows)) {
            this->setLevel(1);
            for(int n=0;n<16;n++)
                _error[n] = 0;
        } else {
            for(int i = -2; i<2; i++) {
                for(int j = -2; j<2; j++) {
                    int num = 4 * i + j + 10;
                    _error[num] = _measurement[num] - GetPixelValue(img, u+i, v+j);
                }
            }
        }
    }

    // 下面函数不用自己写了，但是要能看懂
    virtual void linearizeOplus() override
    {
        if(level()==1)
        {
            _jacobianOplusXj = Matrix<double,16,3>::Zero();
            _jacobianOplusXi = Matrix<double,16,6>::Zero();
            return;
        }
        const PoseVertex* vertexTcw = static_cast<const PoseVertex* >(vertex(0));
        const g2o::VertexSBAPointXYZ* vertexPw = static_cast<const g2o::VertexSBAPointXYZ* >(vertex(1));
        Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();
        float x = Pc[0];
        float y = Pc[1];
        float z = Pc[2];

        float inv_z = 1.0/z;
        float inv_z2 = inv_z * inv_z;
        float u = x * inv_z * fx + cx;
        float v = y * inv_z * fy + cy;

        Matrix<double,2,3> J_Puv_Pc;
        J_Puv_Pc(0,0) = fx * inv_z;
        J_Puv_Pc(0,1) = 0;
        J_Puv_Pc(0,2) = -fx * x * inv_z2;
        J_Puv_Pc(1,0) = 0;
        J_Puv_Pc(1,1) = fy * inv_z;
        J_Puv_Pc(1,2) = -fy * y * inv_z2;


        Matrix<double,3,6> J_Pc_kesi = Matrix<double,3,6>::Zero();
        J_Pc_kesi(0,0) = 1;
        J_Pc_kesi(0,4) = z;
        J_Pc_kesi(0,5) = -y;
        J_Pc_kesi(1,1) = 1;
        J_Pc_kesi(1,3) = -z;
        J_Pc_kesi(1,5) = x;
        J_Pc_kesi(2,2) = 1;
        J_Pc_kesi(2,3) = y;
        J_Pc_kesi(2,4) = -x;

        Matrix<double,1,2> J_I_Puv;
        for(int i = -2; i<2; i++)
            for(int j = -2; j<2; j++) {
                int num = 4 * i + j + 10;
                J_I_Puv(0,0) = (GetPixelValue(img,u+i+1,v+j) - GetPixelValue(img,u+i-1,v+j))/2;             // x方向像素梯度
                J_I_Puv(0,1) = (GetPixelValue(img,u+i,v+j+1) - GetPixelValue(img,u+i,v+j-1))/2;             // y方向像素梯度
                _jacobianOplusXj.block<1,3>(num,0) = -J_I_Puv * J_Puv_Pc * vertexTcw->estimate().rotationMatrix();
                _jacobianOplusXi.block<1,6>(num,0) = -J_I_Puv * J_Puv_Pc * J_Pc_kesi;
            }
    }

private:
    cv::Mat img;
    // Vector16d gray_value;
};

string img_file_dir = "./img/";
string points_file = "./points.txt";
string poses_file = "./poses.txt";

int main() {

    // 内参矩阵
    K << fx, 0, cx,
            fy, cy, 0,
            0, 0, 1;

    // 读取poses.txt用于初始化位姿顶点
    VecSE3d poses;

    ifstream fin(poses_file);
    if (!fin) {
        LOG_ERROR("Cannot open the file. " << poses_file);
        return 1;
    }

    string line;
    while (getline(fin, line)) {
        preprocess(line);
        if (line == "") continue;

        stringstream ss;
        ss << line;
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        ss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        // LOG_INFO(timestamp << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " " << qw);
        poses.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
    }
    LOG_INFO("Total poses: " << poses.size());

    fin.close();
    fin.clear(ios::goodbit);
    // 读取points.txt，用于建立边
    VecVec3d points;
    vector<Vector16d> color;

    fin.open(points_file);
    if (!fin) {
        LOG_ERROR("Cannot open the file. " << points_file);
        return 1;
    }

    while (getline(fin, line)) {
        preprocess(line);
        if (line == "") continue;

        stringstream ss;
        ss << line;
        double x, y, z;
        double* g = new double[16];
        ss >> x >> y >> z;
        for (int i=0;i<16;i++) {
            ss >> g[i];
        }

        points.push_back(Eigen::Vector3d(x, y, z));
        Vector16d gray_value;
        gray_value << g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7], g[8], g[9], g[10], g[11], g[12], g[13], g[14], g[15];
        color.push_back(gray_value);
    }
    LOG_INFO("Total points: " << points.size());
    LOG_INFO("Total gray matrix: " << color.size());

    // read images
    vector<cv::Mat> images;
    vector<cv::String> imgs_name;
    cv::String pattern = "./img/*.png";
    cv::glob(pattern, imgs_name);
    for (auto img_name: imgs_name) {
        cv::Mat img = cv::imread(img_name, 0);
        images.push_back(img);
    }

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > BlockSolverType;  // pose 维度为 6, landmark 维度为 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg ( g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    // 第4步：创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose(true);

    // 第5步：添加顶点和边
    // ----------------  开始你的代码 ----------------------//
    int index = 0;
    for(int i = 0; i < points.size(); i++) {
        g2o::VertexSBAPointXYZ* vertexPw = new g2o::VertexSBAPointXYZ();
        vertexPw->setEstimate(points[i]);
        vertexPw->setId(index++);
        vertexPw->setMarginalized(true);
        optimizer.addVertex(vertexPw);
    }
    for(int j = 0; j < poses.size(); j++) {
        PoseVertex* vertexTcw = new PoseVertex();
        vertexTcw->setEstimate(poses[j]);
        vertexTcw->setId(index++);
        optimizer.addVertex(vertexTcw);
    }

    for(int c = 0; c < poses.size(); c++)
        for(int p = 0; p < points.size(); p++) {
            EdgeGrayProjection* edge = new EdgeGrayProjection(images[c]);
            edge->setMeasurement(color[p]);
            edge->setVertex(0,dynamic_cast<PoseVertex*>(optimizer.vertex(c + points.size())));
            edge->setVertex(1,dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(p)));
            edge->setInformation(Matrix<double,16,16>::Identity());
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            edge->setRobustKernel(rk);
            optimizer.addEdge(edge);
        }
    // ----------------  结束你的代码 ----------------------//

    // 第6步：执行优化
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // 从optimizer中获取结果
    for (int i=0;i<points.size();i++) {
        g2o::VertexSBAPointXYZ* v = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i));
        points[i] = v->estimate();
    }

    for (int i=0;i<poses.size();i++) {
        PoseVertex* v = static_cast<PoseVertex*>(optimizer.vertex(i + points.size()));
        poses[i] = v->estimate();
    }

    // plot the optimized points and poses
    Draw(poses, points);

    return 0;
}

// 画图函数，无需关注
void Draw(const VecSE3d &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }
    cout<<"Draw poses: "<<poses.size()<<", points: "<<points.size()<<endl;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}