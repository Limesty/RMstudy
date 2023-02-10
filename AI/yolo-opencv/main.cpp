// 这是用来测试的源文件

#include "yolo.hpp"
#include <sys/time.h>

using namespace std;

// camera detect
int main()
{
    Net_config netconfig={0.5,0.3,"../yoloRM.onnx","../class.txt",640,640};
    YOLO yolo(netconfig);
    VideoCapture cam(0);    // replace this 0 with the path of video, this will become a video detection.
    Mat img;
    char key=0;
    string windowName="Camera detect";
    namedWindow(windowName);
    struct timeval time;
    long t0,t1;
    while(1)
    {
        cam>>img;
        gettimeofday(&time,NULL);
        t0=time.tv_usec+time.tv_sec*1000*1000;
        yolo.detect_(img);
        gettimeofday(&time,NULL);
        t1=time.tv_usec+time.tv_sec*1000*1000;
        cout<<"It take "<<(t1-t0)/1000<<"ms to run detect_()."<<endl;
        imshow(windowName,img);
        key=waitKey(1);
        if (key=='q'){break;}
    }
    cam.release();
    destroyWindow(windowName);
    return 0;
}


// RM data detect ---- success
/*
int main()
{
    Net_config netconfig={0.5,0.3,"../yoloRM.onnx","../class.txt",640,640};
    YOLO yolo(netconfig);
    Mat img=imread("../test.jpg");
    yolo.detect_(img);
    imshow(" ",img);
    waitKey(0);
    return 0;
}
*/

// environment test ---- success
/*
int main()
{
    Mat img=imread("../test.jpg");
    imshow(" ",img);
    waitKey(0);
    return 0;
}
*/
