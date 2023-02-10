// 这是用来测试的源文件

#include "yolo.hpp"
#include <sys/time.h>

using namespace std;

// camera detect
int main()
{
    Net_config netconfig={0.5,0.3,"../yoloRM.onnx","../class.txt",640,640};
    YOLO yolo(netconfig);
    Mat img = imread("./testimg.png");
    yolo.detect_(img);
    imshow("testimg",img);
    waitKey();
    return 0;
}


