#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

Mat imgProc(Mat imgIn) {
    Mat imgOut,reverse;
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y, dst;
    imgIn.copyTo(imgOut);
    imgIn.copyTo(reverse);

    //图像取反
    bitwise_not(reverse,reverse);
    //或者使用Mat imgOut = ~imgOut?

    int thre = 120, maxv = 255;
    //thre=thresh阈值，maxv=maxval灰度最大值

    //提取红色区域和蓝色区域
    Mat imgOut_hsv;
    Mat reverse_hsv;
    cvtColor(imgOut, imgOut_hsv, COLOR_BGR2HSV);
    cvtColor(reverse, reverse_hsv, COLOR_BGR2HSV);

    Scalar lower_blue = Scalar(100, 0, 0);
    Scalar upper_blue = Scalar(124, 255, 255);
    //Scalar lower_green = Scalar(35, 50, 50);
    //Scalar upper_green = Scalar(77, 255, 255);
    Scalar lower_red_1 = Scalar(0, 0, 0);
    Scalar upper_red_1 = Scalar(25, 255, 255);//inital red: 10. change to 25 to get some orange
    Scalar lower_red_2 = Scalar(156, 0, 0);
    Scalar upper_red_2 = Scalar(180, 255, 255);
    Mat blue_mask, green_mask, red_mask, red_mask_1, red_mask_2;
    Mat blue_mask_reverse, green_mask_reverse, red_mask_reverse, red_mask_1_reverse, red_mask_2_reverse;
    inRange(imgOut_hsv, lower_blue, upper_blue, blue_mask);
    //inRange(imgOut_hsv, lower_green, upper_green, green_mask);
    inRange(imgOut_hsv, lower_red_1, upper_red_1, red_mask_1);
    inRange(imgOut_hsv, lower_red_2, upper_red_2, red_mask_2);
    red_mask = red_mask_1 + red_mask_2;
    inRange(reverse_hsv, lower_blue, upper_blue, blue_mask_reverse);
    //inRange(reverse_hsv, lower_green, upper_green, green_mask_reverse);
    inRange(reverse_hsv, lower_red_1, upper_red_1, red_mask_1_reverse);
    inRange(reverse_hsv, lower_red_2, upper_red_2, red_mask_2_reverse);
    red_mask_reverse = red_mask_1_reverse + red_mask_2_reverse;
    Mat blue_res;
    Mat blue_res_reverse;
    bitwise_and(imgOut, imgOut, blue_res, blue_mask);
    bitwise_and(reverse, reverse, blue_res_reverse, blue_mask_reverse);
    Mat red_res;
    Mat red_res_reverse;
    bitwise_and(imgOut, imgOut, red_res, red_mask);
    bitwise_and(reverse, reverse, red_res_reverse, red_mask_reverse);
    Mat res =  blue_res - red_res;
    Mat res_reverse = blue_res_reverse - red_res_reverse;

    //return red_res;
    return blue_res_reverse;
    //cvtColor(imgOut, imgOut, COLOR_BGR2GRAY);
    //begin to get the edge
    /*Sobel(imgOut, grad_x, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);
    Sobel(imgOut, grad_y, CV_16S, 0, 1, 3, 1, 1, BORDER_DEFAULT);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, imgOut);*/
    //end
    //threshold(imgOut, imgOut, thre, maxv, THRESH_BINARY);
    //dilate(imgOut, imgOut, Mat());//really need?
    //return imgOut;
    
    /*
    cvtColor(imgOut, imgOut, COLOR_BGR2GRAY);//convert to GRAY(use?)
    erode(imgOut, imgOut, Mat());
    dilate(imgOut, imgOut, Mat());
    GaussianBlur(imgOut, imgOut, Size(15, 15), 2, 2, 4);//blur to make edge more easy to approach
                                                        //not sure whether to use it
    floodFill(imgOut, Point(5, 50), Scalar(255), 0, FLOODFILL_FIXED_RANGE);
    */
 }

void imageIO() {
    //input image path
    //string imgPath;
    //cout << "Select a file to Open: \n";
    //cin >> imgPath;

    //Ctrl + D
    //if (imgPath.empty()) imgPath = "D:/testimg.png";

    //read image
    Mat img = imread("./testimg.png");

    //show image
    imshow("Display Image", imgProc(img));

    //quit
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