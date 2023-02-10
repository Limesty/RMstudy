#include <fstream>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "onnxruntime_cxx_api.h"

using namespace cv;
using namespace Ort;
using namespace std;

struct Net_config
{
	float confThreshold; // 置信度阈值
	float nmsThreshold;  // 最大置信重合
	string modelpath;
	string namefilepath;
};

struct Detection	//识别结果
{
	int classNumber;		//识别结果对映的数字
	string className;		//识别结果的标签
	float confidence;		//识别的置信度
	float x1;			//对象左上角x
	float y1;			//对象左上角y
	float x2;			//对象右下角x
	float y2;			//对象右下角y
};

typedef struct BoxInfo
{
	float x1;
	float y1;
	float x2;
	float y2;
	float score;
	int label;
} BoxInfo;

class YOLO
{
public:
	YOLO(Net_config config);
	vector<Detection> detect(Mat& frame);
	void DrawOnImage(Mat img,vector<Detection> detection);
private:
	int inpWidth;
	int inpHeight;
	int nout;
	int num_proposal;
	vector<string> class_names;
	int num_class;

	float confThreshold;
	float nmsThreshold;
	vector<float> input_image_;
	void normalize_(Mat img);
	void nms(vector<BoxInfo>& input_boxes);
	vector<BoxInfo> forwardAndNMS(Mat& frame);

	Ort::Session *ort_session = nullptr;
	vector<char*> input_names;
	vector<char*> output_names;
};
