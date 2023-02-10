#ifndef Camera
#define Camera

#include"CameraApi.h" //相机SDK的API头文件

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <stdio.h>
#include <vector>

using namespace std;
using namespace cv;

class MVCamera{
public:
    MVCamera();
    ~MVCamera();
    bool OpenCamera();    //打开相机并返回相机是否正常打开，0代表未正常打开，1代表正常打开
    vector<Mat> ReadCamera(); //从相机中获取一帧图像
    void CloseCamera();   //关闭相机
private:
    unsigned char*                  g_pRgbBuffer;       //处理后图像数据缓存区指针
    int                             iCameraCounts = 1;  //保存实际找到的设备个数
    int                             iStatus=-1;         //用于判断相机是否初始化成功
    tSdkCameraDevInfo*              tCameraEnumList;    //设备列表数组
    int                             hCamera;            //相机的句柄,代表当前相机
    vector<int>                     hCameraVec;         //相机的句柄列表
    tSdkCameraCapbility             tCapability;        //相机的属性
    vector<tSdkCameraCapbility>     tCapabilityVec;     //相机的属性列表
    tSdkFrameHead                   sFrameInfo;         //图像的帧头信息指针
    BYTE*			                pbyBuffer;          //指向原始图像数据的缓冲区指针
    vector<Mat>                     optMatVec;          //读取的图像列表
};

#endif