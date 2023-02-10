# 人工智能尝鲜组

## 图像识别算法

采用yolov7进行识别，yolo.cpp 中有 yolo 类可调用。

初始化 yolo 类需要指定参数，重点是 Net_config.modelpath 指定采用的模型文件路径。

```cpp
//初始化
//onnxruntime方案的类
Net_config YOLO_nets = { 0.3, 0.5,"yoloRM.onnx","labels.txt" };
YOLO YOLO_model(YOLO_nets);

vector<Detection> result=YOLO_model.detect(image);// 调用 yolo 得到识别结果
YOLO_model.DrawOnImage(image，result);//这是标注结果，是原位操作，image 会变成标注后的图像
```

opencv_dnn 的方式依赖于 opencv 版本，部署可以用的版本的方法在文件夹内。
onnxruntime 方案通过参数调节在352x256的模型上识别速度达到了 20fps 。
