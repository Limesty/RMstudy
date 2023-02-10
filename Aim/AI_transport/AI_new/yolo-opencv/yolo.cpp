#include <fstream>
#include <sstream>
#include "yolo.hpp"

YOLO::YOLO(Net_config config)
{
	this->inpWidth = config.inpwidth;
	this->inpHeight = config.inpheight;

	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;

	this->net = readNet(config.modelpath);
	ifstream ifs(config.namefilepath);
	string line;
	while (getline(ifs, line)) this->class_names.push_back(line);
	this->num_class = class_names.size();
}

Mat YOLO::resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left)
{
	int srch = srcimg.rows, srcw = srcimg.cols;
	*newh = this->inpHeight;
	*neww = this->inpWidth;
	Mat dstimg;
	float hw_scale = (float)srch / srcw;
	if (hw_scale > 1) {
		*newh = this->inpHeight;
		*neww = int(this->inpWidth / hw_scale);
		resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
		*left = int((this->inpWidth - *neww) * 0.5);
		copyMakeBorder(dstimg, dstimg, 0, 0, *left, this->inpWidth - *neww - *left, BORDER_CONSTANT, 114);
	}
	else {
		*newh = (int)this->inpHeight * hw_scale;
		*neww = this->inpWidth;
		resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
		*top = (int)(this->inpHeight - *newh) * 0.5;
		copyMakeBorder(dstimg, dstimg, *top, this->inpHeight - *newh - *top, 0, 0, BORDER_CONSTANT, 114);
	}
	return dstimg;
}

// 绘制识别框线、识别类名和置信度
void YOLO::drawPred(float conf, int left, int top, int right, int bottom, Mat& frame, int classid)   
{
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255), 2);

	string label = format("%.2f", conf);
	label = this->class_names[classid] + ":" + label;

	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);

	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0), 1);
}

//识别并返回识别结果
vector<Detection> YOLO::detect(Mat& frame)
{
	int newh = 0, neww = 0, padh = 0, padw = 0;
	Mat srcimg; frame.copyTo(srcimg);
	Mat dstimg = this->resize_image(srcimg, &newh, &neww, &padh, &padw);
	Mat blob = blobFromImage(dstimg, 1 / 255.0, Size(this->inpWidth, this->inpHeight), Scalar(0, 0, 0), true, false);
	this->net.setInput(blob);
	vector<Mat> outs;
	vector<Detection> result;
	
	//进行网络的推导
	this->net.forward(outs, this->net.getUnconnectedOutLayersNames());

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

//识别并且返回标注之后的图像
void YOLO::detect_(Mat& frame)
{
	int newh = 0, neww = 0, padh = 0, padw = 0;
	Mat dstimg = this->resize_image(frame, &newh, &neww, &padh, &padw);
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
