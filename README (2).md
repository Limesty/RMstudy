# 算法组2022-2023年度代码仓库
## 重要模块

### 1. 相机驱动

通过 MindVisionSDK 开发的相机驱动
```cpp
class MVCamera {
    void OpenCamera(); //启动相机
    vector<Mat> ReadCamera(); //获取一帧图像
    //返回值为获取到的图像，每个相机占用 vector 一个元素，一个 Mat 对应一个相机的图像
}
```

### 2. 装甲板检测

采用了 YOLOv7 作为神经网络架构，ONNXRuntime 作为运行环境
```cpp
class YOLO {
    vector<Detection> detect(Mat Test); //对一张图片进行一次装甲板检测
    //参数 Mat Test 是待检测的一张图片
    //返回值为识别到的对象，每个对象占用 vector 一个元素，具体内容如下文 struct Detection
}
struct Detection //识别到的对象
{
	int classNumber; //对象类别
	string className; //对象类别对应的名字
	float confidence; //识别正确的概率
	float x1; //左边缘坐标，单位pixel
	float y1; //上边缘坐标，单位pixel
	float x2; //右边缘坐标，单位pixel
	float y2; //下边缘坐标，单位pixel
};
```

### 3. 双目解析

有关于双目解析的相关内容都在StereoReconstruct类之中。

其中task方法接收参数为两个图像，分别为左目和右目相机传递进来的图像。经过task进行双目测距任务之后，得到了像素坐标的三维重建信息，存储在cv::Mat xyz_coord之中。如果需要打印信息，可以类似于下列代码来得到更详细的信息。

会得到[x, y, depth]格式的三维坐标，即为世界坐标系下的三维坐标。**世界坐标系的中心为左目相机的光心**。

```cpp
    cv::Point start;
    start = Point(203, 200);
    cout << " world coords=(x,y,depth)=" << detector->xyz_coord.at<Vec3f>(start) << endl;
```

### 4. 位置解算

通过相机的部分参数，解得识别到的对象的空间位置
```cpp
class Solver {
    Point2f solve(Point2f LeftUpPoint, Point2f RightDownPoint, Size size); //对一个对象进行一次位置解算
    //参数 Point2f LeftUpPoint 是对象的左上角
    //参数 Point2f RightDownPoint 是对象的右下角
    //参数 Size size 是对象所在的图片的尺寸信息
    //返回值对象中心点解算出的角度信息，属性 x 是 yaw，属性 y 是 pitch
    //以相机视线为x轴正方向，相机的水平向左方向为y轴正方向，相机的垂直向上方向为z轴正方向，建立空间直角坐标系
    //设对象中心点在 xOy 平面上的投影点为 P，则射线 OP 的角度为 yaw，取值范围 (-π, π)
    //设对象中心点在 xOz 平面上的投影点为 Q，则射线 OQ 的角度为 pitch，取值范围 (-π/2, π/2)
}
```