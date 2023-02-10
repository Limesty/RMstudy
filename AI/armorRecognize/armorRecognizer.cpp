#include "armorRecognizer.hpp"

ArmorRecognizer::ArmorRecognizer(string modelpath)
{
    Env env = Env(ORT_LOGGING_LEVEL_ERROR, "YOLO");
	SessionOptions sessionOptions = SessionOptions();
	ort_session = new Session(env, modelpath.c_str(), sessionOptions);
    // Read the model's information
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
    this->input_image_.resize(128*128);
}

const float* ArmorRecognizer::preTreatAndForward(Mat& img)
{
    // pretreat
    Mat distimg;
    cvtColor(img,distimg,COLOR_BGRA2GRAY);
    distimg.convertTo(distimg, CV_32F);
    distimg/=255.0;
    int c=0;
    for (int i = 0; i < _width_; i++)
    {
        for (int j = 0; j < _width_; j++)
        {
            this->input_image_[c * _width_ * _width_ + i * _width_ + j] = img.ptr<float>(i)[j + 2 - c];
        }
    }
    // set input and run forward
    array<int64_t, 4> input_shape_{ 1, 1, _width_,_width_};
	auto allocator_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
	Value input_tensor_ = Value::CreateTensor<float>(allocator_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());
	vector<Value> ort_outputs = ort_session->Run(RunOptions{ nullptr }, &input_names[0], &input_tensor_, 1, output_names.data(), output_names.size());
    return ort_outputs[0].GetTensorMutableData<float>();
}

ArmorInfo ArmorRecognizer::recogize(Mat& img)
{
    const float* NetResult=this->preTreatAndForward(img);
    int i,num=0,dirc=10;
    for(i=1;i<10;i++)
    {
        if(NetResult[i]>NetResult[num]){num=i;}
    }
    for(i=10;i<15;i++)
    {
        if(NetResult[i]>NetResult[dirc]){dirc=i;}
    }
    ArmorInfo result={(ArmorNumber)num,NetResult[num],(ArmorDirection)dirc,NetResult[dirc]};
    return result;
}