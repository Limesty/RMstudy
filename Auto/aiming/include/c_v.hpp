//这个是匀速直线运动算法（cv）的头文件
#include"opencv2/opencv.hpp"
#include<iostream>
#include<stdio.h>
#include<math.h>
using namespace cv;
using namespace std;
const float dt=1;//delta_t采样时间，此处假定为1秒，以真实情况为准

const float s_x2=0.1;//sigma_x的平方,x方向位置测量误差方差，下同理
const float s_y2=0.1;
const float s_z2=0.1;

const float s_ax2=0.1;//sigma_ax的平方,x方向加速度方差（在匀速模型中加速度作为误差存在）
const float s_ay2=0.1;
const float s_vz2=0.001;//默认装甲板高度不变（地图好像没有斜坡），只考虑可能的上下抖动,vz理论上很小

void imageIO() {
        cv::Mat mz = cv::Mat::zeros(5,5,CV_8UC1); // 全零矩阵
	cv::Mat mo = cv::Mat::ones(5,5,CV_8UC1);  // 全1矩阵
	cv::Mat me = cv::Mat::eye(5,5,CV_32FC1);  // 对角线为1的对角矩阵
	cout<<"mz = "<<endl<<mz<<endl<<endl;
	cout<<"mo = "<<endl<<mo<<endl<<endl;
	cout<<"me = "<<endl<<me<<endl<<endl;

}//测试用函数
void init_c_v(cv::Mat cen0,cv::Mat cen1,cv::Mat cen2,cv::Mat* px_c_v,cv::Mat* pp_c_v)//初始化函数
{
        cv::Mat M1 = (cv::Mat_<float>(5,3) << 1,0,0,0,1,0,1/dt,0,0,0,1/dt,0,0,0,1);
        cv::Mat M2 = (cv::Mat_<float>(5,3) << 0,0,0,0,0,0,-1/dt,0,0,0,-1/dt,0,0,0,0);
        cv::Mat a = (cv::Mat_<float>(5,5) << 1,0,dt,0,0,0,1,0,dt,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1);
        cv::Mat at = a.t();
        cv::Mat q = (cv::Mat_<float>(5,5) << 0.25*pow(dt,4)*s_ax2,0,0.5*pow(dt,3)*s_ax2,0,0,0,0.25*pow(dt,4)*s_ay2,0,0.5*pow(dt,3)*s_ay2,0,0.5*pow(dt,3)*s_ax2,0,pow(dt,2)*s_ax2,0,0,0,0.5*pow(dt,3)*s_ay2,0,pow(dt,2)*s_ay2,0,0,0,0,0,pow(dt,2)*s_vz2);
        *px_c_v = a*(M1*cen2+M2*cen1);
        *pp_c_v = a*(*pp_c_v)*at+q;
        cout<<"x_cv_init="<<*px_c_v<<endl;//测试用
}
 void cal_c_v(cv::Mat cen_now,cv::Mat* px_c_v,cv::Mat* pp_c_v)//预测下一帧函数
 {  
        
        cv::Mat a = (cv::Mat_<float>(5,5) << 1,0,dt,0,0,0,1,0,dt,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1);
        cv::Mat h = (cv::Mat_<float>(3,5) << 1,0,0,0,0,0,1,0,0,0,0,0,0,0,1);
        cv::Mat r = (cv::Mat_<float>(3,3) << s_x2,0,0,0,s_y2,0,0,0,s_z2);
        cv::Mat q = (cv::Mat_<float>(5,5) << 0.25*pow(dt,4)*s_ax2,0,0.5*pow(dt,3)*s_ax2,0,0,0,0.25*pow(dt,4)*s_ay2,0,0.5*pow(dt,3)*s_ay2,0,0.5*pow(dt,3)*s_ax2,0,pow(dt,2)*s_ax2,0,0,0,0.5*pow(dt,3)*s_ay2,0,pow(dt,2)*s_ay2,0,0,0,0,0,pow(dt,2)*s_vz2);
        cv::Mat at = a.t();
        cv::Mat ht = h.t();
        cv::Mat i = cv::Mat::eye(5,5,CV_32FC1);//单位阵
        //上面的命名符合一般的卡尔曼滤波形式
        //下面是卡尔曼滤波器
        cv::Mat cup1 = h*(*pp_c_v)*ht+r;//中间变量1
        cv::Mat cup1i = cup1.inv();
        cv::Mat k = (*pp_c_v)*ht*cup1i;
        *px_c_v = a*((*px_c_v)+k*(cen_now-h*(*px_c_v)));
        *pp_c_v = a*((i-k*h)*(*pp_c_v))*at+q;
        cout<<"x_cv="<<*px_c_v<<endl;
}

