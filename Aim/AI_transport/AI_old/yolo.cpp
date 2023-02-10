#include <fstream>
#include <sstream>
#include "yolo.hpp"


//通过this指针可以访问类的所有成员
YOLO::YOLO(Net_config config)												//将各种参数传入YOLO类中
{
	this->confThreshold = config.confThreshold;		//传入置信度阈值
	this->nmsThreshold = config.nmsThreshold;			//传入最大置信重合
	this->net = readNet(config.modelpath);						//传入模型文件

	ifstream ifs(config.namefilepath);								//用文件输入流读入需识别类
	string line;
	while (getline(ifs, line)) this->class_names.push_back(line);
	//循环读入字符串并加入在字符串容器class_names的尾部
	this->num_class = class_names.size();						//传入需识别类数量
}


Mat YOLO::resize_image(Mat srcimg)							//重置图片大小
{
	int srch = srcimg.rows, srcw = srcimg.cols;		//像素矩阵的行数rows，列数cols
	Mat dstimg;																				//处理后的图像

	//纯色填充成为正方形
	if (srch > srcw){
		copyMakeBorder(srcimg, dstimg, 0, 0, (srch-srcw)/2, (srch+srcw)/2, BORDER_CONSTANT, 114);
	}
	else{
		copyMakeBorder(srcimg, dstimg, (srcw-srch)/2, (srcw+srch)/2, 0, 0, BORDER_CONSTANT, 114);
	}

	//缩放到640*640
	resize(dstimg, dstimg, Size(this->inpWidth, this->inpHeight), INTER_AREA);	

	return dstimg;
}


void YOLO::drawPred(float conf, int left, int top, int right, int bottom, Mat& frame, int classid)
{
	//绘制显示边界框的矩形
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255), 2);

	//获取类名的标签及其可信度
	string label = format("%.2f", conf);
	label = this->class_names[classid] + ":" + label;

	//将标签显示在边界框上方
	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
}


vector<Detection> YOLO::detect(Mat& frame)
{
	//初始化
	int newh = 0, neww = 0, padh = 0, padw = 0;
	Mat srcimg; frame.copyTo(srcimg);
	Mat dstimg = this->resize_image(srcimg);//重置图片大小
	Mat blob = blobFromImage(dstimg, 1 / 255.0, Size(this->inpWidth, this->inpHeight), Scalar(0, 0, 0), true, false);	//构造二值图像
	this->net.setInput(blob);
	vector<Mat> outs;
	vector<Detection> result;
	this->net.forward(outs, this->net.getUnconnectedOutLayersNames());	//向前传递

	int num_proposal = outs[0].size[0];
	int nout = outs[0].size[1];
	if (outs[0].dims > 2)
	{
		num_proposal = outs[0].size[1];
		nout = outs[0].size[2];
		outs[0] = outs[0].reshape(0, num_proposal);
	}
	/////解析网络返回值
	vector<float> confidences;
	vector<Rect> boxes;
	vector<int> classIds;
	float ratioh = (float)srcimg.rows / newh, ratiow = (float)srcimg.cols / neww;
	int n = 0, row_ind = 0;
	float* pdata = (float*)outs[0].data;
	for (n = 0; n < num_proposal; n++)   ///特征图尺度
	{
		float box_score = pdata[4];
		if (box_score > this->confThreshold)
		{
			Mat scores = outs[0].row(row_ind).colRange(5, nout);
			Point classIdPoint;
			double max_class_socre;
			
			minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
			max_class_socre *= box_score;
			if (max_class_socre > this->confThreshold)
			{
				const int class_idx = classIdPoint.x;
				float cx = (pdata[0] - padw) * ratiow;	///cx
				float cy = (pdata[1] - padh) * ratioh;	///cy
				float w = pdata[2] * ratiow;			///w
				float h = pdata[3] * ratioh;			///h

				int left = int(cx - 0.5 * w);
				int top = int(cy - 0.5 * h);

				confidences.push_back((float)max_class_socre);
				boxes.push_back(Rect(left, top, (int)(w), (int)(h)));
				classIds.push_back(class_idx);
			}
		}
		row_ind++;
		pdata += nout;
	}

	// 通过非极大抑制挑选概率最高的结果
	vector<int> indices;
	dnn::NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		Detection detect = { classIds[idx],this->class_names[classIds[idx]],confidences[idx], box.x / 640.0f, box.y / 640.0f,
			(box.x + box.width) / 640.0f, (box.y + box.height) / 640.0f };
		result.push_back(detect);
	}
	return result;
}

void YOLO::detect_(Mat& frame)
{
	int newh = 0, neww = 0, padh = 0, padw = 0;
	Mat dstimg = this->resize_image(frame);
	Mat blob = blobFromImage(dstimg, 1 / 255.0, Size(this->inpWidth, this->inpHeight), Scalar(0, 0, 0), true, false);
	this->net.setInput(blob);
	vector<Mat> outs;
	this->net.forward(outs, this->net.getUnconnectedOutLayersNames());

	int num_proposal = outs[0].size[0];
	int nout = outs[0].size[1];
	if (outs[0].dims > 2)
	{
		num_proposal = outs[0].size[1];
		nout = outs[0].size[2];
		outs[0] = outs[0].reshape(0, num_proposal);
	}
	/////generate proposals
	vector<float> confidences;
	vector<Rect> boxes;
	vector<int> classIds;
	float ratioh = (float)frame.rows / newh, ratiow = (float)frame.cols / neww;
	int n = 0, row_ind = 0;
	float* pdata = (float*)outs[0].data;
	for (n = 0; n < num_proposal; n++)   ///特征图尺度
	{
		float box_score = pdata[4];
		if (box_score > this->confThreshold)
		{
			Mat scores = outs[0].row(row_ind).colRange(5, nout);
			Point classIdPoint;
			double max_class_socre;
			
			minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
			max_class_socre *= box_score;
			if (max_class_socre > this->confThreshold)
			{
				const int class_idx = classIdPoint.x;
				float cx = (pdata[0] - padw) * ratiow;  ///cx
				float cy = (pdata[1] - padh) * ratioh;   ///cy
				float w = pdata[2] * ratiow;   ///w
				float h = pdata[3] * ratioh;  ///h

				int left = int(cx - 0.5 * w);
				int top = int(cy - 0.5 * h);

				confidences.push_back((float)max_class_socre);
				boxes.push_back(Rect(left, top, (int)(w), (int)(h)));
				classIds.push_back(class_idx);
			}
		}
		row_ind++;
		pdata += nout;
	}

	vector<int> indices;
	dnn::NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		this->drawPred(confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame, classIds[idx]);
	}
}