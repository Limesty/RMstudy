#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <onnxruntime_cxx_api.h>

//显卡加速需要包含一个头文件
//#include <cuda_provider_factory.h>

using namespace cv;
using namespace ort;
using namespace std;

sstruct Net_config
{
	float confThreshold; // 置信度阈值
	float nmsThreshold;  // 最大置信重合
	string modelpath;
	string namefilepath;
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
	void detect_(Mat& frame);
private:
	int inpWidth;
	int inpHeight;
	int nout;
	int num_proposal;
	vector<string> class_names;
	int num_class;

	float confThreshold;
	float nmsThreshold;
	const bool keep_ratio = true;
	vector<float> input_image_;
	void normalize_(Mat img);
	void nms(vector<BoxInfo>& input_boxes);
	Mat resize_image(Mat srcimg, int *newh, int *neww, int *top, int *left);

	Env env = Env(ORT_LOGGING_LEVEL_ERROR, "YOLO");
	Ort::Session *ort_session = nullptr;
	SessionOptions sessionOptions = SessionOptions();
	vector<char*> input_names;
	vector<char*> output_names;
	vector<vector<int64_t>> input_node_dims;  // >=1 outputs
	vector<vector<int64_t>> output_node_dims; // >=1 outputs
};
