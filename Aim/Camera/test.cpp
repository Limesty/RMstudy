#include "Camera.hpp"
#include <stdio.h>

using namespace cv;

MVCamera newCamera;
int main() {
    bool flag = newCamera.OpenCamera();
    if(flag){
        imshow("Camera->OpenCV Test", newCamera.ReadCamera());
        newCamera.CloseCamera();
    }
    return 0;
}