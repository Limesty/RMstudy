# 人工智能尝鲜组

## 图像识别算法

### yolo.cpp

采用yolov7进行识别，yolo.cpp 中有  yolo 类可调用。

初始化 yolo 类需要指定参数，重点是 Net_config.modelpath 指定采用的模型文件路径。

```cpp
//初始化
Net_config YOLO_nets = { 0.3, 0.5,"./yoloRM.onnx","./class.txt" };
YOLO YOLO_model(YOLO_nets);

vector<Detection> result=YOLO_model.detect(image);// 调用 yolo 得到识别结果，注意里面的坐标是归一化后的结果
YOLO_model.detect_(image);//这是输出标注图像版本，是原位操作，image 会变成标注后的图像
```

### 调用 yolo 类可能出现的问题

（这个bug在Windows环境遇到过，ubuntu中还没有）

`opencv-python\opencv\modules\dnn\include\opencv2/dnn/shape_utils.hpp:171: error: (-215:Assertion failed) start <= (int)shape.size() && end <= (int)shape.size() && start <= end in function 'cv::dnn::dnn4_v20211220::total'`

一般版本的 opencv 似乎是不兼容 yolov7，在 github 上有一个[解决方案](https://github.com/opencv/opencv/issues/21967)。

Manjaro

`terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.6.0) /build/opencv/src/opencv-4.6.0/modules/dnn/include/opencv2/dnn/shape_utils.hpp:170: error: (-215:Assertion failed) start <= (int)shape.size() && end <= (int)shape.size() && start <= end in function 'total'
zsh: IOT instruction (core dumped)  ./test`