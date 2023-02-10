#ifndef CAMERA_CALIBRATION_RECONSTRUCT_CPP_STEREO_RECONSTRUCT_H
#define CAMERA_CALIBRATION_RECONSTRUCT_CPP_STEREO_RECONSTRUCT_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;


/**
 * 双目摄像头的相机参数
 */
struct CameraParam {
    int width;           //图像的宽度width
    int height;          //图像的高度height
    Mat cameraMatrixL;   //左相机内参K1(3×3)
    Mat distCoeffL;      //左相机畸变系数D1(5×1)
    Mat cameraMatrixR;   //右相机内参K2(3×3)
    Mat distCoeffR;      //右相机畸变系数D2(5×1)
    Mat T;               //平移向量T(3×1)
    Mat R;               //旋转矩阵R(3×3)，如果是(3×1)旋转向量，请使用cv::Rodrigues()进行变换转为(3×3)旋转矩阵R
};

/***
 * 设置摄像头参数，需要根据双目摄像头标定结果进行填写
 */
static CameraParam camera1 = {640,//width
                              480,//height
                              (Mat_<double>(3, 3)
                                      << 7.6159209686633153e+02, 0., 3.2031427422691633e+02, 0., 7.6167321446015626e+02, 2.2467546926913309e+02, 0., 0., 1.),//cameraMatrixL
                              (Mat_<double>(5, 1)
                                      << 3.4834574887256914e-02, -5.5261651680159028e-02, 5.7491952534806736e-04, -4.2764223950233445e-05, 1.8477350164208820e-02),//distCoeffL
                              (Mat_<double>(3, 3)
                                      << 7.6327773983796783e+02, 0., 2.8768149776326379e+02, 0., 7.6350419482215057e+02, 2.1897333669573928e+02, 0., 0., 1.),
                              (Mat_<double>(5, 1)
                                      << 3.5020967512300320e-02, -4.0770565902033332e-02, -4.4231049297594003e-04, -1.0552565496142535e-03, -9.7750314807571667e-02),
                              (Mat_<double>(3, 1)
                                      << -6.0005833075452117e+01, 1.7047023105446815e-01, 6.0300273851103448e-01),
                              (Mat_<double>(3, 3)
                                      << 9.9999370551606337e-01, 7.8563882630048958e-04, 3.4600144345510440e-03, -7.9503149273969136e-04, 9.9999600080163187e-01, 2.7140938945082542e-03, -3.4578682997252063e-03, -2.7168276311286426e-03, 9.9999033095047696e-01),
};

/***
 * 显示图像
 * @param winname 窗口名称
 * @param image 图像
 * @param delay 显示延迟，0表示阻塞显示
 * @param flags 显示方式
 */
static void show_image(const string &winname, cv::Mat &image, int delay = 0, int flags = cv::WINDOW_AUTOSIZE) {
    cv::namedWindow(winname, flags);
    cv::imshow(winname, image);
    cv::waitKey(delay);
}

/***
 * 读取视频文件
 * @param video_file 视频文件
 * @param cap 视频流对象
 * @param width 设置图像的宽度
 * @param height 设置图像的高度
 * @param fps 设置视频播放频率
 * @return
 */
bool get_video_capture(string video_file, cv::VideoCapture &cap, int width = -1, int height = -1, int fps = -1) {
    //VideoCapture video_cap;
    cap.open(video_file);
    if (width > 0 && height > 0) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width); //设置图像的宽度
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height); //设置图像的高度
    }
    if (fps > 0) {
        cap.set(cv::CAP_PROP_FPS, fps);
    }
    if (!cap.isOpened())//判断是否读取成功
    {
        return false;
    }
    return true;
}

/***
 * 读取摄像头
 * @param camera_id 摄像头ID号，默认从0开始
 * @param cap 视频流对象
 * @param width 设置图像的宽度
 * @param height 设置图像的高度
 * @param fps 设置视频播放频率
 * @return
 */
bool get_video_capture(int camera_id, cv::VideoCapture &cap, int width = -1, int height = -1, int fps = -1) {
    //VideoCapture video_cap;
    cap.open(camera_id);    //摄像头ID号，默认从0开始
    if (width > 0 && height > 0) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width); //设置捕获图像的宽度
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);  //设置捕获图像的高度
    }
    if (fps > 0) {
        cap.set(cv::CAP_PROP_FPS, fps);
    }
    if (!cap.isOpened()) //判断是否成功打开相机
    {
        return false;
    }
    return true;
}

class StereoReconstruct {
public:
    cv::Mat xyz_coord;                                   //用于存放每个像素点距离相机镜头的三维坐标

    /***
     * 构造函数，初始化StereoReconstruct
     * @param camera 双目相机参数
     * @param use_wls 是否使用WLS滤波器对视差图进行滤波
     */
    StereoReconstruct(CameraParam camera, bool use_wls = true);

    /***
     * release
     */
    ~StereoReconstruct();

    /***
     * 开始双目测距任务
     * @param frameL
     * @param frameR
     */
    void task(Mat frameL, Mat frameR, int delay = 0);

    /***
     * 畸变校正和立体校正
     * @param imgL 左视图
     * @param imgR 右视图
     * @param rectifiedL 校正后左视图
     * @param rectifiedR 校正后右视图
     */
    void get_rectify_image(Mat &imgL, Mat &imgR, Mat &rectifiedL, Mat &rectifiedR);

    /***
     * 获得视差图
     * @param imgL 畸变校正和立体校正后的左视图
     * @param imgR 畸变校正和立体校正后的右视图
     * @param dispL 返回视差图
     * @param use_wls 是否使用WLS滤波器对视差图进行滤波
     */
    void get_disparity(Mat &imgL, Mat &imgR, Mat &dispL, bool use_wls = true);//SGBM匹配算法

    /***
     * 计算像素点的3D坐标（左相机坐标系下）
     * @param disp 视差图
     * @param points_3d 返回三维坐标points_3d，三个通道分布表示(X,Y,Z)，其中Z是深度图depth, 即距离,单位是毫米(mm)
     * @param scale 单位变换尺度,默认scale=1.0,单位为毫米
     */
    void get_3dpoints(Mat &disp, Mat &points_3d, float scale = 1.0);

    /***
     * 将输入深度图转换为伪彩色图，方面可视化
     * @param depth
     * @param colormap
     */
    void get_visual_depth(cv::Mat &depth, cv::Mat &colormap, float clip_max = 6000.0);

    /***
     * 显示矫正效果
     * @param rectifiedL
     * @param rectifiedR
     */
    void show_rectify_result(cv::Mat rectifiedL, cv::Mat rectifiedR);

    /***
     * 可视化视差图和深度图
     * @param frameL
     * @param frameR
     * @param points_3d
     * @param disp
     * @param delay
     */
    void show_2dimage(Mat &frameL, Mat &frameR, Mat &points_3d, Mat &disp, int delay);

    /***
     * 显示Mat的最大最小值
     * @param src
     * @param vmin 最小值下限
     * @param vmax 最大值下限
     */
    void clip(cv::Mat &src, float vmin, float vmax);

    /***
     * 显示Mat的最大最小值
     * @param src
     * @param th
     * @param vmin
     */
    void clip_min(cv::Mat &src, float th, float vmin);


public:
    string depth_windows = "depth-color";             // 深度图的窗口名称
    int use_wls;                                      // 是否使用WLS滤波器对视差图进行滤波
    Size image_size;                                  // 图像宽高(width,height)
    Rect validROIL;                                   // 图像校正之后，会对图像进行裁剪，这里的左视图裁剪之后的区域
    Rect validROIR;                                   // 图像校正之后，会对图像进行裁剪，这里的右视图裁剪之后的区域
    Mat mapLx, mapLy, mapRx, mapRy;                   // 映射表
    Mat Rl, Rr, Pl, Pr, Q;                            // 校正后的旋转矩阵R，投影矩阵P, 重投影矩阵Q
    cv::Ptr<cv::StereoSGBM> sgbm;
};


#endif //CAMERA_CALIBRATION_RECONSTRUCT_CPP_STEREO_RECONSTRUCT_H
