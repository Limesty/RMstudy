#include <fstream>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "onnxruntime_cxx_api.h"

using namespace cv;
using namespace Ort;
using namespace std;

enum ArmorNumber
{
    uknown=0,
    one,
    two,
    three,
    four,
    five,
    six,
    seven,
    eight,
    nine
};
// this direction is on camera vision.
enum ArmorDirection
{
    tooleft=0,
    left,
    front,
    right,
    tooright
};

struct ArmorInfo
{
    ArmorNumber number;
    float numConfidence;
    ArmorDirection direction;
    float directConfidence;
};
class ArmorRecognizer
{
    public:
        ArmorRecognizer(string modelpath);
        ArmorInfo recogize(Mat& img);
    private:
        const int _width_=128;
        vector<char*> input_names;
        vector<char*> output_names;
        vector<float> input_image_;
        Ort::Session *ort_session = nullptr;
        const float* preTreatAndForward(Mat& img);
};