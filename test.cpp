#include "AI/yolo-opencv/yolo.hpp"
//opencv include path: /usr/local/include/opencv4

const Net_config DETECTOR_PRESETS={0.5, 0.3, "./yoloRM.onnx", "./class.txt", 640, 640};
YOLO ArmorDetector(DETECTOR_PRESETS);

int main(){
    Mat testimg = imread("./test.png");
    int cnt = 0;
    while(waitKey(1) != 'q') {
        
        //imshow("test_in", testimg);
        vector<Detection> result=ArmorDetector.detect(testimg);
        //imshow("test_out", testimg);
        cout << "[info] frame " << cnt++ << ": " << getTickCount() / getTickFrequency() << endl;
    }
    return 0;
}