#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

Mat imgProc(Mat imgIn) {
    Mat original,reverse;
    imgIn.copyTo(original);
    imgIn.copyTo(reverse);

    //图像取反
    bitwise_not(reverse,reverse);
    //或者使用Mat original = ~original?

    //提取红色区域和蓝色区域
    //转换为HSV空间
    Mat original_hsv;
    Mat reverse_hsv;
    cvtColor(original, original_hsv, COLOR_BGR2HSV);
    cvtColor(reverse, reverse_hsv, COLOR_BGR2HSV);

    //定义红色区间
    Scalar lower_red_1 = Scalar(0, 0, 0);
    Scalar upper_red_1 = Scalar(25, 255, 255);//inital red: 10. change to 25 to get some orange
    Scalar lower_red_2 = Scalar(156, 0, 0);
    Scalar upper_red_2 = Scalar(180, 255, 255);

    //提取红色区域
    Mat red_mask, red_mask_1, red_mask_2;
    inRange(original_hsv, lower_red_1, upper_red_1, red_mask_1);
    inRange(original_hsv, lower_red_2, upper_red_2, red_mask_2);
    red_mask = red_mask_1 + red_mask_2;

    //提取蓝色区域
    Mat blue_mask, red_mask_1_reverse, red_mask_2_reverse;
    inRange(reverse_hsv, lower_red_1, upper_red_1, red_mask_1_reverse);
    inRange(reverse_hsv, lower_red_2, upper_red_2, red_mask_2_reverse);
    blue_mask = red_mask_1_reverse + red_mask_2_reverse;
 
    Mat red_res;
    Mat blue_res;
    bitwise_and(original, original, red_res, red_mask);
    bitwise_and(reverse, reverse, blue_res, blue_mask);
    bitwise_not(blue_res,blue_res);   //出现绿色，需要修改
    
    //红蓝区域改灰度
    Mat red_gray,blue_gray;
    cvtColor(red_res, red_res, COLOR_HSV2BGR);
    cvtColor(red_res, red_gray, COLOR_BGR2GRAY);
    cvtColor(blue_res, blue_res, COLOR_HSV2BGR);
    cvtColor(blue_res, blue_gray, COLOR_BGR2GRAY);
    
    //红色区域填空洞
    Mat red_hole;
    red_gray.copyTo(red_hole);
    floodFill(red_hole, Point(5, 50), Scalar(255), 0, FLOODFILL_FIXED_RANGE);
    int thre = 120, maxv = 255;  //thre=thresh阈值，maxv=maxval灰度最大值
    threshold(red_hole, red_hole, thre, maxv, THRESH_BINARY);   //灰度值大于thre，变为255，反之变为0
    bitwise_xor(~red_hole,red_gray,red_gray);
    erode(red_gray, red_gray, Mat());
    threshold(red_gray, red_gray, thre, maxv, THRESH_BINARY);
    for (int i = 0; i < 4; ++i)
    {
        erode(red_gray, red_gray, Mat());
        dilate(red_gray, red_gray, Mat());
        erode(red_gray, red_gray, Mat());
        dilate(red_gray, red_gray, Mat());
        erode(red_gray, red_gray, Mat());
        dilate(red_gray, red_gray, Mat());
        erode(red_gray, red_gray, Mat());
    }
    dilate(red_gray, red_gray, Mat());

    //处理蓝色区域
    dilate(blue_gray, blue_gray, Mat());
    floodFill(blue_gray, Point(5, 50), Scalar(0), 0, FLOODFILL_FIXED_RANGE);
    threshold(blue_gray, blue_gray, thre, maxv, THRESH_BINARY);
    erode(blue_gray, blue_gray, Mat());
    erode(blue_gray, blue_gray, Mat());
    erode(blue_gray, blue_gray, Mat());
    dilate(blue_gray, blue_gray, Mat());
    dilate(blue_gray, blue_gray, Mat());

    return blue_gray;
 }

void imageIO() {
    //读取图像路径
    //string imgPath;
    //cout << "Select a file to Open: \n";
    //cin >> imgPath;
    //if (imgPath.empty()) imgPath = "D:/testimg.png";

    //读取并显示图像
    Mat img = imread("./testimg.png");
    imshow("Display Image", imgProc(img));

    waitKey();
}

void videoIO() {
    Mat frame;
    VideoCapture video;
    video.open("../TestFiles/Energy.mp4");

    do {
        video >> frame;//cin >> a;

        if (frame.empty()) break;

        imshow("Video", frame);

        imshow("Proc", imgProc(frame));
    } while (waitKey(0) != 'q');//100ms
}


int main() {
    imageIO();
    return 0;
}