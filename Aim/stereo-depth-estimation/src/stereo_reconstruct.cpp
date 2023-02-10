#include "stereo_reconstruct.h"
#include "opencv2/ximgproc.hpp"


StereoReconstruct::StereoReconstruct(CameraParam camera, bool use_wls) {
    this->use_wls = use_wls;
    this->image_size = Size(camera.width, camera.height);
    //Mat rec = (Mat_<double>(3, 1) << -0.01688, -0.00781, -0.00766);   //rec旋转向量
    //Rodrigues(rec, R); //Rodrigues变换：将旋转向量转为旋转矩阵R
    //立体校正
    stereoRectify(camera.cameraMatrixL, camera.distCoeffL,
                  camera.cameraMatrixR, camera.distCoeffR,
                  image_size, camera.R,
                  camera.T, Rl, Rr, Pl, Pr, Q,
                  CALIB_ZERO_DISPARITY,
                  0, image_size,
                  &validROIL, &validROIR);
    initUndistortRectifyMap(camera.cameraMatrixL, camera.distCoeffL, Rl, Pl, image_size, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(camera.cameraMatrixR, camera.distCoeffR, Rr, Pr, image_size, CV_32FC1, mapRx, mapRy);

    //SGBM算法初始化
    int mindisparity = 0;                                                    //最小视差
    int blockSize = 3;                                                       //窗口的大小
    int numDisparities = 5 * 16;                                             //最大的视差，要被16整除
    //int P1 = 4 * rectifyImageL.channels() * SADWindowSize* SADWindowSize;  //惩罚系数1
    //int P2 = 32 * rectifyImageL.channels() * SADWindowSize* SADWindowSize; //惩罚系数2
    int P1 = 8 * 3 * blockSize;  //惩罚系数1
    int P2 = 32 * 3 * blockSize; //惩罚系数2
    sgbm = cv::StereoSGBM::create(mindisparity, numDisparities, blockSize);
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setDisp12MaxDiff(12);                                             //视差图的像素点检查
    sgbm->setUniquenessRatio(10);                                         //代价方程概率因子
    sgbm->setSpeckleWindowSize(50);                                     //针对散斑滤波的窗口大小
    sgbm->setSpeckleRange(32);                                              //相邻像素点的视差值浮动范围
    sgbm->setPreFilterCap(63);                                               //滤波系数
    //sgbm->setMode(cv::StereoSGBM::MODE_HH);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

}


StereoReconstruct::~StereoReconstruct() {
    mapLx.release();
    mapLy.release();
    mapRx.release();
    mapRy.release();
    Rl.release();
    Rr.release();
    Pl.release();
    Pr.release();
    Q.release();
    sgbm.release();
}

void StereoReconstruct::task(Mat frameL, Mat frameR, int delay) {
    // 畸变校正和立体校正
    Mat rectifiedL, rectifiedR;
    this->get_rectify_image(frameL, frameR, rectifiedL, rectifiedR);
    // 绘制等间距平行线，检查立体校正的效果
    this->show_rectify_result(rectifiedL, rectifiedR);
    // Get the disparity map 获得视差图
    Mat dispL;
    this->get_disparity(rectifiedL, rectifiedR, dispL, this->use_wls);
    //用于存放每个像素点距离相机镜头的三维坐标
    Mat points_3d;
    this->get_3dpoints(dispL, points_3d);
    xyz_coord = points_3d;
    // 显示视差图效果
    this->show_2dimage(frameL, frameR, points_3d, dispL, delay);
}

void StereoReconstruct::get_rectify_image(Mat &imgL, Mat &imgR, Mat &rectifiedL, Mat &rectifiedR) {
    //经过remap之后，左右相机的图像已经共面并且行对准
    //remap(grayL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    //remap(grayR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    remap(imgL, rectifiedL, mapLx, mapLy, INTER_LINEAR);
    remap(imgR, rectifiedR, mapRx, mapRy, INTER_LINEAR);
}

void StereoReconstruct::show_rectify_result(cv::Mat rectifiedL, cv::Mat rectifiedR) {
    //把校正结果显示出来
    //show_image("ImageL After Rectify", rectifiedL, 1);
    //show_image("ImageR After Rectify", rectifiedR, 1);
    //显示在同一张图上
    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(image_size.width, image_size.height);
    w = cvRound(image_size.width * sf);
    h = cvRound(image_size.height * sf);
    canvas.create(h, w * 2, CV_8UC3);

    //左图像画到画布上
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分
    resize(rectifiedL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);    //把图像缩放到跟canvasPart一样大小
    Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),                  //获得被截取的区域
               cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
    //rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);
    //cout << "Painted ImageL" << endl;

    //右图像画到画布上
    canvasPart = canvas(Rect(w, 0, w, h));                                        //获得画布的另一部分
    resize(rectifiedR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
               cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
    //rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
    //cout << "Painted ImageR" << endl;
    //画上对应的线条
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    // show_image("rectified", canvas, 1);
}

void StereoReconstruct::get_disparity(Mat &imgL, Mat &imgR, Mat &dispL, bool use_wls) {
    //https://blog.csdn.net/nayuanxiu8089/article/details/126227633
    //Mat rectifyImageL, rectifyImageR;
    cvtColor(imgL, imgL, COLOR_BGR2GRAY);
    cvtColor(imgR, imgR, COLOR_BGR2GRAY);
    sgbm->compute(imgL, imgR, dispL);
    if (use_wls) {
        //RightMatcher 初始化
        int lmbda = 80000;
        float sigma = 1.3;
        Ptr<StereoMatcher> matcherR = cv::ximgproc::createRightMatcher(sgbm);
        cv::Mat dispR;
        matcherR->compute(imgR, imgL, dispR);
        auto filter = cv::ximgproc::createDisparityWLSFilter(sgbm);
        filter->setLambda(lmbda);
        filter->setSigmaColor(sigma);
        filter->filter(dispL, imgL, dispL, dispR);
    }
    //除以16得到真实视差（因为SGBM算法得到的视差是×16的）
    //clip_min(dispL,0,0);
    dispL.convertTo(dispL, CV_32F, 1.0 / 16);
}

void StereoReconstruct::get_3dpoints(Mat &disp, Mat &points_3d, float scale) {
    //在实际求距离时ReprojectTo3D出来
    reprojectImageTo3D(disp, points_3d, Q, true);
    points_3d = points_3d * scale;
}

void StereoReconstruct::show_2dimage(Mat &frameL, Mat &frameR, Mat &points_3d, Mat &disp, int delay) {
    //显示结果
    vector<Mat> xy_depth;
    split(points_3d, xy_depth); //分离出深度通道
    cv::Mat depth = xy_depth[2];

    // show_image("left", frameL, 1);
    // show_image("right", frameR, 1);
    // 可视化视差图
    Mat disp_colormap(disp.size(), CV_8UC3);
    get_visual_depth(disp, disp_colormap);
    // show_image("disparity-color", disp_colormap, 1);

    // 可视化深度图
    Mat depth_colormap(depth.size(), CV_8UC3);
    get_visual_depth(depth, depth_colormap);
    //鼠标响应函数setMouseCallback
    // namedWindow(depth_windows, WINDOW_AUTOSIZE);
    // setMouseCallback(depth_windows, onMouse, 0);
    // show_image(depth_windows, depth_colormap, delay);
}

void StereoReconstruct::get_visual_depth(cv::Mat &depth, cv::Mat &colormap, float clip_max)                    //颜色变换
{
    clip(depth, 0.0, clip_max);
    Mat int8disp = Mat(depth.rows, depth.cols, CV_8UC1);                       //用于显示
    normalize(depth, int8disp, 0, 255, NORM_MINMAX, CV_8UC1);
    //medianBlur(int8disp, int8disp, 9);                                         //中值滤波
    cv::applyColorMap(int8disp, colormap, cv::COLORMAP_JET);
}

void StereoReconstruct::clip(cv::Mat &src, float vmin, float vmax) {
    int h = src.rows;
    int w = src.cols;
    if (src.isContinuous() && src.isContinuous()) {
        h = 1;
        w = w * src.rows * src.channels();
    }
    for (int i = 0; i < h; i++) {
        float *sptr = src.ptr<float>(i);
        for (int j = 0; j < w; j++) {
            //*dptr++ = *sptr++;
            sptr[j] = sptr[j] < vmax ? sptr[j] : vmax;
            sptr[j] = sptr[j] > vmin ? sptr[j] : vmin;
        }
    }
}

void StereoReconstruct::clip_min(cv::Mat &src, float th, float v) {
    int h = src.rows;
    int w = src.cols;
    if (src.isContinuous() && src.isContinuous()) {
        h = 1;
        w = w * src.rows * src.channels();
    }
    for (int i = 0; i < h; i++) {
        float *sptr = src.ptr<float>(i);
        for (int j = 0; j < w; j++) {
            //*dptr++ = *sptr++;
            sptr[j] = sptr[j] < th ? v : sptr[j];
        }
    }
}