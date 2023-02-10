#include <iostream>
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace dnn;
using namespace std;

struct Net_config
{
	float confThreshold; // 置信度阈值
	float nmsThreshold;  // 最大置信重合
	string modelpath;
	string namefilepath;
	int inpwidth;
	int inpheight;
};

struct Detection	//识别结果
{
	int classNumber;
	string className;
	float confidence;
	float x1;
	float y1;			//对象左上角
	float x2;
	float y2;			//对象右下角
};

class YOLO
{
public:
	YOLO(Net_config config);
	vector<Detection> detect(Mat& frame);
	void detect_(Mat& frame);
private:
	int inpWidth;
	int inpHeight;
	vector<string> class_names;
	int num_class;

	float confThreshold;
	float nmsThreshold;
	Net net;
	Mat resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left);
	void drawPred(float conf, int left, int top, int right, int bottom, Mat& frame, int classid);
};