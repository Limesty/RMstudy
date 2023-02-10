#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;
using namespace std;

//相机定位系统
class Location {
    // 最终处理所得图像
    Mat ret_image;
    // 左右相机采集到的图像
    Mat l_image;
    Mat r_image;
    // 左右相机畸变参数
    Mat l_dist_coef;
    Mat r_dist_coef;
    // 左右相机内参
    Mat l_intrinsics;
    Mat r_intrinsics;
    // 左右相机外参
    Mat l_extrinsics;
    Mat r_extrinsics;
    // 左右相机间的旋转矩阵、平移矩阵
    Mat rvec;
    Mat tvec;
    // 世界坐标系的特征点
    vector <Point3f> objPoints;
    public:
    // 1. 相机定位系统初始化
    void Init_Location();   //初始化内参等
    // 2. 畸变矫正（核心API：findChessboardCorners、find4QuadCornerSubpix、calibrateCamera）
    void camCalibration();  // 采集标定图像后使用opencv内置的API矫正
    // 3. 图像预处理
    Mat img_process(Mat img);   // 做一些图像增强、滤波等
    // 4. 空间点三维重建
    Mat loc_resolve(Mat l_img, Mat r_img);
};
