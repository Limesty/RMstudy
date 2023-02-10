//
// 双目测距Demo
//
#include <opencv2/opencv.hpp>
#include <iostream>
#include "stereo_reconstruct.h"

/***
 * 测试demo视频文件
 * @return
 */
int test_video_file() {
    CameraParam camera = camera1;//双目相机参数
    bool use_wls = true;         //是否使用WLS滤波器对视差图进行滤波
    StereoReconstruct *detector = new StereoReconstruct(camera, use_wls);
    int imageWidth = camera1.width;      //单目图像的宽度
    int imageHeight = camera1.height;    //单目图像的高度
    string left_video = "../data/lenacv-video/left_video.avi";
    string right_video = "../data/lenacv-video/right_video.avi";
    VideoCapture capL, capR;
    bool retL = get_video_capture(left_video, capL, imageWidth, imageHeight);
    bool retR = get_video_capture(right_video, capR, imageWidth, imageHeight);
    Mat frameL, frameR;
    while (retL && retR) {
        capL >> frameL;
        capR >> frameR;
        if (frameL.empty() or frameR.empty()) break;
        detector->task(frameL, frameR, 20);
    }
    capL.release();         //释放对相机的控制
    capR.release();         //释放对相机的控制
    delete detector;
    return 0;

}


/***
 * 测试双目摄像头(双USB连接线的双目摄像头)
 * @return
 */
int test_camera() {
    CameraParam camera = camera1;//双目相机参数
    bool use_wls = true;         //是否使用WLS滤波器对视差图进行滤波
    StereoReconstruct *detector = new StereoReconstruct(camera, use_wls);
    int imageWidth = camera1.width;       //单目图像的宽度
    int imageHeight = camera1.height;     //单目图像的高度
    int camera1 = 0;                      //左摄像头ID号(请修改成自己左摄像头ID号)
    int camera2 = 1;                      //右摄像头ID号(请修改成自己右摄像头ID号)
    VideoCapture capL, capR;
    bool retL = get_video_capture(camera1, capL, imageWidth, imageHeight);
    bool retR = get_video_capture(camera2, capR, imageWidth, imageHeight);
    Mat frameL, frameR;
    while (retL && retR) {
        capL >> frameL;
        capR >> frameR;
        if (frameL.empty() or frameR.empty()) break;
        detector->task(frameL, frameR, 20);
    }
    capL.release();         //释放对相机的控制
    capR.release();         //释放对相机的控制
    delete detector;
    return 0;
}

/***
 * 测试一对左右图像
 * @return
 */
int test_pair_image_file() {
    CameraParam camera = camera1;//双目相机参数
    bool use_wls = true;         //是否使用WLS滤波器对视差图进行滤波
    StereoReconstruct *detector = new StereoReconstruct(camera, use_wls);
    Mat frameL = imread("../data/left.png", IMREAD_COLOR);
    Mat frameR = imread("../data/right.png", IMREAD_COLOR);
    detector->task(frameL, frameR, 0);
    cv::Point start;
    start = Point(203, 200);
    cout << " world coords=(x,y,depth)=" << detector->xyz_coord.at<Vec3f>(start) << endl;
    delete detector;
    return 0;
}


int main() {
    //测试一对左右图像
    test_pair_image_file();
    // //测试demo视频文件
    // test_video_file();
    // //测试双目摄像头(双USB连接线的双目摄像头)
    // test_camera();
    // while(1);
    return 0;
}
