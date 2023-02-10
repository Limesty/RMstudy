#include <iostream>
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace dnn;
using namespace std;

struct Net_config					//YOLO配置
{
	float confThreshold; 		//置信度阈值
	float nmsThreshold;  		//最大置信重合
	string modelpath;				//模型文件路径
	string namefilepath;		//需识别类名称文件路径
};

struct Detection				//识别结果
{
	int classNumber;				//识别类编号
	string className;			//识别类名称
	float confidence;			//准确度
	float x1;
	float y1;								//对象左上角
	float x2;
	float y2;								//对象右下角
};

class YOLO																				//YOLO类
{
public:
	YOLO(Net_config config);											//传入参数
	vector<Detection> detect(Mat& frame);			//识别
	void detect_(Mat& frame);											//绘制识别后图像

private:
	const int inpWidth=640;												//固定图像宽度
	const int inpHeight=640;											//固定图像高度
	vector<string> class_names;								//存放需识别类名称的字符串容器
	int num_class;																	//需识别类的数量

	float confThreshold;
	float nmsThreshold;
	Net net;
	Mat resize_image(Mat srcimg);								//存放导入的参数

	//绘制识别后图像中矩形
	void drawPred(float conf, int left, int top, int right, int bottom, Mat& frame, int classid);
};