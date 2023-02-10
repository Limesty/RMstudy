//这个是匀加速直线运动算法（ca）的头文件
#include"opencv2/opencv.hpp"
#include<iostream>
#include<stdio.h>
#include<math.h>
using namespace cv;
using namespace std;
//重复的数据见c_v.hpp

const float s_axdot2=0.1;//sigma_ax导的平方,x方向jerk（急剧度，加速度的导数）方差（在匀加速模型中jerk作为误差存在）
const float s_aydot2=0.1;

void init_ca(cv::Mat cen0,cv::Mat cen1,cv::Mat cen2,cv::Mat* px_ca,cv::Mat* pp_ca)//初始化函数
{
        cv::Mat M1 = (cv::Mat_<float>(7,3) << 1,0,0,0,1,0,1/dt,0,0,0,1/dt,0,1/pow(dt,2),0,0,0,1/pow(dt,2),0,0,0,1);
        cv::Mat M2 = (cv::Mat_<float>(7,3) << 0,0,0,0,0,0,-1/dt,0,0,0,-1/dt,0,-2/pow(dt,2),0,0,0,-2/pow(dt,2),0,0,0,0);
        cv::Mat M3 = (cv::Mat_<float>(7,3) << 0,0,0,0,0,0,0,0,0,0,0,0,1/pow(dt,2),0,0,0,1/pow(dt,2),0,0,0,0);
         cv::Mat a = (cv::Mat_<float>(7,7) << 1,0,dt,0,0.5*pow(dt,2),0,0,0,1,0,dt,0,0.5*pow(dt,2),0,0,0,1,0,dt,0,0,0,0,0,1,0,dt,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1);
         cv::Mat at = a.t();
         cv::Mat cup2 = (cv::Mat_<float>(7,3) << pow(dt,3)/6,0,0,0,pow(dt,3)/6,0,pow(dt,2)/2,0,0,0,pow(dt,2)/2,0,dt,0,0,0,dt,0,0,0,dt);
        cv::Mat cup2t = cup2.t();
        cv::Mat cup3 = (cv::Mat_<float>(3,3) << s_axdot2,0,0,0,s_aydot2,0,0,0,s_vz2);
        cv::Mat q = cup2*cup3*cup2t;
        *px_ca = a*(M1*cen2+M2*cen1+M3*cen0);
        *pp_ca = a*(*pp_ca)*at+q;
        cout<<"x_ca_init="<<*px_ca<<endl;//测试用
}
        

void cal_ca(cv::Mat cen_now,cv::Mat* px_ca,cv::Mat* pp_ca)//预测下一帧函数
 {  
        
        cv::Mat a = (cv::Mat_<float>(7,7) << 1,0,dt,0,0.5*pow(dt,2),0,0,0,1,0,dt,0,0.5*pow(dt,2),0,0,0,1,0,dt,0,0,0,0,0,1,0,dt,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1);
        cv::Mat h = (cv::Mat_<float>(3,7) << 1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1);
        cv::Mat r = (cv::Mat_<float>(3,3) << s_x2,0,0,0,s_y2,0,0,0,s_z2);
        cv::Mat cup2 = (cv::Mat_<float>(7,3) << pow(dt,3)/6,0,0,0,pow(dt,3)/6,0,pow(dt,2)/2,0,0,0,pow(dt,2)/2,0,dt,0,0,0,dt,0,0,0,dt);
        cv::Mat cup2t = cup2.t();
        cv::Mat cup3 = (cv::Mat_<float>(3,3) << s_axdot2,0,0,0,s_aydot2,0,0,0,s_vz2);
        cv::Mat q = cup2*cup3*cup2t;
        cv::Mat at = a.t();
        cv::Mat ht = h.t();
        cv::Mat i = cv::Mat::eye(7,7,CV_32FC1);//单位阵
        //上面的命名符合一般的卡尔曼滤波形式
        //下面是卡尔曼滤波器
        cv::Mat cup1 = h*(*pp_ca)*ht+r;//中间变量1
        cv::Mat cup1i = cup1.inv();
        cv::Mat k = (*pp_ca)*ht*cup1i;
        *px_ca = a*((*px_ca)+k*(cen_now-h*(*px_ca)));
        *pp_ca = a*((i-k*h)*(*pp_ca))*at+q;
        cout<<"x_ca="<<*px_ca<<endl;
}
