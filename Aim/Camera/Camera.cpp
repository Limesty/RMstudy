#include "Camera.hpp"

MVCamera::MVCamera()
{
    // 功能描述 : 相机SDK初始化，在调用任何SDK其他接口前，必须
    //           先调用该接口进行初始化。该函数在整个进程运行
    //           0:表示英文,1:表示中文。
    CameraSdkInit(1);
    tCameraEnumList = (tSdkCameraDevInfo*)(malloc(127 * sizeof(tSdkCameraDevInfo)));
}

MVCamera::~MVCamera()
{
    CloseCamera();
}

bool MVCamera::OpenCamera()
{
    // 枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);
    // 若没有连接设备
    if (iCameraCounts == 0)
    {
        printf("未找到相机\n");
        return 0;
    }

    INT allHeightMax = 0, allWidthMax = 0;

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    while (iCameraCounts--)
    {
        iStatus = CameraInit(tCameraEnumList, -1, -1, &hCamera);
        // 初始化失败
        if (iStatus != CAMERA_STATUS_SUCCESS)
        {
            printf("有设备初始化失败\n");
            return 0;
        }
        hCameraVec.push_back(hCamera);
        tCameraEnumList++;

        // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera, &tCapability);
        tCapabilityVec.push_back(tCapability);

        allHeightMax = max(tCapability.sResolutionRange.iHeightMax, allHeightMax);
        allWidthMax = max(tCapability.sResolutionRange.iWidthMax, allWidthMax);

        /*让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
        CameraPlay(hCamera);

        // 设置CameraGetImageBuffer函数的图像处理的输出格式，此处为24位彩色图像
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    // 为图像数据缓存区分配空间，为最大高度*最大宽度*深度
    g_pRgbBuffer = (unsigned char *)malloc(allHeightMax * allWidthMax * 3);

    return true;
}

vector<Mat> MVCamera::ReadCamera()
{
    optMatVec.clear();
    for (vector<int>::iterator ithCamera = hCameraVec.begin(); ithCamera != hCameraVec.end(); ithCamera++)
    {
        hCamera = *ithCamera;
        // CameraGetImageBuffer函数功能：获得一帧图像数据，并将之存储到pbyBuffer中
        //       1000为抓取图像的超时时间。单位毫秒。在该时间内还未获得图像，
        //       则该函数会返回超时信息。
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            // 功能描述 : 将获得的相机原始输出图像数据进行处理，叠加饱和度、
            //        颜色增益和校正、降噪等处理效果，最后得到RGB888
            //        格式的图像数据。
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            optMatVec.push_back(Mat(
                Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer));

            // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
        else
        {
            printf("获得图像超时\n");
            optMatVec.push_back(Mat::zeros(500, 500, CV_8UC3));
        }
    }
    return optMatVec;
}

void MVCamera::CloseCamera()
{
    // 功能描述 : 相机反初始化。释放资源。
    CameraUnInit(hCamera);
    // 释放数据缓存区空间
    free(g_pRgbBuffer);
    free(tCameraEnumList);
}
