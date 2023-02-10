这是调用 opencv 中的 dnn 模块的方案。

# 调用 yolo 类可能出现的问题

`opencv-python\opencv\modules\dnn\include\opencv2/dnn/shape_utils.hpp:171: error: (-215:Assertion failed) start <= (int)shape.size() && end <= (int)shape.size() && start <= end in function 'cv::dnn::dnn4_v20211220::total'`

4.7.0版本之前的  opencv 是不兼容 yolov7 的，因为之前版本的 opencv 不支持广播操作，所以会出现矩阵尺寸不对等或者是vector下标越界的问题。在 github 上有一个[参考](https://github.com/opencv/opencv/issues/21967)。

在 Linux 物理机上已经用  opencv4.7.0 调通了。
这个版本在cpu环境下只有 5 fps，速度太慢，成不了最终方案，还需要找其他方案加速(在调openVINO)。
