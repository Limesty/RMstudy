#include "yolo.hpp"

YOLO::YOLO(Net_config config)
{
	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;
	// 加载模型
	Env env = Env(ORT_LOGGING_LEVEL_ERROR, "YOLO");
	SessionOptions sessionOptions = SessionOptions();
	ort_session = new Session(env, config.modelpath.c_str(), sessionOptions);
	// 读取模型信息
	size_t numInputNodes = ort_session->GetInputCount();
	size_t numOutputNodes = ort_session->GetOutputCount();
	AllocatorWithDefaultOptions allocator;
	vector<vector<int64_t>> input_node_dims;
	vector<vector<int64_t>> output_node_dims;
	for (int i = 0; i < numInputNodes; i++)
	{
		input_names.push_back(ort_session->GetInputName(i, allocator));
		Ort::TypeInfo input_type_info = ort_session->GetInputTypeInfo(i);
		auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
		auto input_dims = input_tensor_info.GetShape();
		input_node_dims.push_back(input_dims);
	}
	for (int i = 0; i < numOutputNodes; i++)
	{
		output_names.push_back(ort_session->GetOutputName(i, allocator));
		Ort::TypeInfo output_type_info = ort_session->GetOutputTypeInfo(i);
		auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
		auto output_dims = output_tensor_info.GetShape();
		output_node_dims.push_back(output_dims);
	}
	this->inpHeight = input_node_dims[0][2];
	this->inpWidth = input_node_dims[0][3];
	this->nout = output_node_dims[0][2];
	this->num_proposal = output_node_dims[0][1];
	this->input_image_.resize(inpHeight * inpWidth * 3);
	// 读取标签
	ifstream ifs(config.namefilepath.c_str());
	string line;
	while (getline(ifs, line)) 
	{
		line.pop_back();
		this->class_names.push_back(line);	
	}
	this->num_class = class_names.size();
}
// 对输入进行归一化
void YOLO::normalize_(Mat img)
{
	img.convertTo(img, CV_32F);
	img/=255.0;
	for (int c = 0; c < 3; c++)
	{
		for (int i = 0; i < inpHeight; i++)
		{
			for (int j = 0; j < inpWidth; j++)
			{
				this->input_image_[c * inpHeight * inpWidth + i * inpWidth + j] = img.ptr<float>(i)[j * 3 + 2 - c];
			}
		}
	}
}
// 对输出进行非极大抑制
void YOLO::nms(vector<BoxInfo>& input_boxes)
{
	sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
	vector<float> vArea(input_boxes.size());
	for (int i = 0; i < int(input_boxes.size()); ++i)
	{
		vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1)
			* (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
	}

	vector<bool> isSuppressed(input_boxes.size(), false);
	for (int i = 0; i < int(input_boxes.size()); ++i)
	{
		if (isSuppressed[i]) { continue; }
		for (int j = i + 1; j < int(input_boxes.size()); ++j)
		{
			if (isSuppressed[j]) { continue; }
			float xx1 = (max)(input_boxes[i].x1, input_boxes[j].x1);
			float yy1 = (max)(input_boxes[i].y1, input_boxes[j].y1);
			float xx2 = (min)(input_boxes[i].x2, input_boxes[j].x2);
			float yy2 = (min)(input_boxes[i].y2, input_boxes[j].y2);

			float w = (max)(float(0), xx2 - xx1 + 1);
			float h = (max)(float(0), yy2 - yy1 + 1);
			float inter = w * h;
			float ovr = inter / (vArea[i] + vArea[j] - inter);

			if (ovr >= this->nmsThreshold)
			{
				isSuppressed[j] = true;
			}
		}
	}
	int idx_t = 0;
	input_boxes.erase(remove_if(input_boxes.begin(), input_boxes.end(), [&idx_t, &isSuppressed](const BoxInfo& f) { return isSuppressed[idx_t++]; }), input_boxes.end());
}
// 推理与输入输出处理
vector<BoxInfo> YOLO::forwardAndNMS(Mat& frame)
{
	int newh = inpHeight, neww = inpWidth, padh = 0, padw = 0;
	vector<Detection> result;
	Mat dstimg;
	resize(frame, dstimg, Size(inpWidth,inpHeight));
	// 设置输入并进行模型推理
	this->normalize_(dstimg);
	array<int64_t, 4> input_shape_{ 1, 3, this->inpHeight, this->inpWidth };
	auto allocator_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
	Value input_tensor_ = Value::CreateTensor<float>(allocator_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());
	vector<Value> ort_outputs = ort_session->Run(RunOptions{ nullptr }, &input_names[0], &input_tensor_, 1, output_names.data(), output_names.size());
	/////对网络的输出结果进行解析
	vector<BoxInfo> generate_boxes;
	float ratioh = (float)frame.rows / newh, ratiow = (float)frame.cols / neww;
	int n = 0, k = 0; ///cx,cy,w,h,box_score, class_score
	const float* pdata = ort_outputs[0].GetTensorMutableData<float>();
	for (n = 0; n < this->num_proposal; n++)
	{
		float box_score = pdata[4];
		if (box_score > this->confThreshold)
		{
			int max_ind = 0;
			float max_class_socre = 0;
			for (k = 0; k < num_class; k++)
			{
				if (pdata[k + 5] > max_class_socre)
				{
					max_class_socre = pdata[k + 5];
					max_ind = k;
				}
			}
			max_class_socre *= box_score;
			if (max_class_socre > this->confThreshold)
			{ 
				float cx = (pdata[0] - padw) * ratiow;  ///cx
				float cy = (pdata[1] - padh) * ratioh;   ///cy
				float w = pdata[2] * ratiow;   ///w
				float h = pdata[3] * ratioh;  ///h

				float xmin = cx - 0.5 * w;
				float ymin = cy - 0.5 * h;
				float xmax = cx + 0.5 * w;
				float ymax = cy + 0.5 * h;

				generate_boxes.push_back(BoxInfo{ xmin, ymin, xmax, ymax, max_class_socre, max_ind });
			}
		}
		pdata += nout;
	}
	nms(generate_boxes);
	return generate_boxes;
}
// 识别并返回结果
vector<Detection> YOLO::detect(Mat& frame)
{
	vector<BoxInfo> generate_boxes=forwardAndNMS(frame);
	vector<Detection> result;
	Detection detect;
	for (size_t i = 0; i < generate_boxes.size(); ++i)
	{

		detect = { generate_boxes[i].label,this->class_names[generate_boxes[i].label],generate_boxes[i].score
			,(generate_boxes[i].x1),(generate_boxes[i].y1),(generate_boxes[i].x2),(generate_boxes[i].y2)};
		result.push_back(detect);
	}
	return result;
}
// 识别并标注结果
void YOLO::DrawOnImage(Mat frame,vector<Detection> detection)
{
	int xmin,ymin;
	string label ;
	for (size_t i = 0; i < detection.size(); ++i)
	{
		xmin = int(detection[i].x1);
		ymin = int(detection[i].y1);
		rectangle(frame, Point(xmin, ymin), Point(int(detection[i].x2), int(detection[i].y2)), Scalar(0, 0, 255), 2);
		label = format("%.2f", detection[i].confidence);
		label = this->class_names[detection[i].classNumber] + ":" + label;
		putText(frame, label, Point(xmin, ymin - 5), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0), 1);
	}
}

