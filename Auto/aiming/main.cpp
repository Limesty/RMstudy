//version 1.0
//                 这是第1版，只有匀速运动模型，主要目的是展示程序的大致思路，如有问题，联系王晟劼
//version 1.0.5
//                 修复了匀速直线模型的亿些问题
//                 现在的模型得到的结果是下一帧的预测结果而不是这一帧的数据融合后结果
//                 增加了匀加速直线运动模型（大概没问题了）和匀速转弯模型（还有亿些问题）
//version 1.0.9
//                 现在的匀速转动模型可以通过调试了，但还是好像有些数值上的问题（也可能没有？）
//                 如果数值对了的话，就是version 1.1了，等我再调试
//version 1.1
//                 找到了数值看似异常的原因：ctrv模型在模拟匀速直线运动时具有限制，详见ctrv模型头文件
//                 现在的匀速转动模型已经可以使用了
//version 1.1.1    
//                 修改了一些ctrv模型中的错误
//version 1.2    
//                 对前面模型的部分代码作了微调，保险起见，重新上传
//                 增加了匀变速转动模型，并且大概已经可以使用了
//version 1.2.5    
//                 增加了使用无迹卡尔曼滤波的匀速转动模型，似乎能用（？）
//这是主函数
//说明：对于每一时刻，需要得到两个数据，一个是装甲板中心点世界系三维坐标，一个是装甲板左上角角点（或是左灯条上顶点）的世界系三维坐标
//角点坐标将用于进行角度方面的计算，如自转角,在没有转动的模型里只用中心点坐标
#include "opencv2/opencv.hpp"
#include <iostream>
#include<stdio.h>
#include"include/c_v.hpp"
#include"include/ca.hpp"
#include"include/ctrv_ekf.hpp"
#include"include/cca_ekf.hpp"
#include"include/ctrv_ukf.hpp"
#include<math.h>
using namespace std;
using namespace cv;
//下面的一排const是开关，控制各个滤波器的启动，0为关，1为开
//如果均为0,输出测量值
//如果只有一个为1（所有变量相加为1）,相当于单个滤波器
//如果同时启动多个滤波器，对多个滤波器的结果进行融合
//模型待补充，可能会有更多更合适的模型
const int c_v=0;//匀速直线运动模型，带有卡尔曼滤波
const int ca=0;//匀变速直线运动模型，带有卡尔曼滤波
const int ctrv_ekf=0;//恒定转弯率与速度模型，使用扩展卡尔曼滤波
const int ctrv_ukf=1;//恒定转弯率与速度模型，使用无迹卡尔曼滤波
const int cca_ekf=0;//恒定曲率和加速度模型，使用扩展卡尔曼滤波
//const int cca_pf=0;//恒定曲率和加速度模型，使用粒子滤波
//const int strange=0;//奇怪的自制模型，将装甲板运动近似视为一个直线运动与一个转动的叠加


cv::Mat x_c_v = cv::Mat::zeros(5,1,CV_32FC1);//用于储存匀速直线运动模型得到的结果,顺序x,y,vx,vy,z
cv::Mat* px_c_v = &x_c_v;//取地址，用于修改
cv::Mat p_c_v = cv::Mat::zeros(5,5,CV_32FC1);//用于储存匀速直线运动模型得到的协方差矩阵，初值自定
cv::Mat* pp_c_v = &p_c_v;//取地址，用于修改
//这一块空间或许可以只在开启cv滤波器的时候使用，下同理

cv::Mat x_ca = cv::Mat::zeros(7,1,CV_32FC1);//用于储存匀加速直线运动模型得到的结果,顺序x,y,vx,vy,ax,ay,z
cv::Mat* px_ca = &x_ca;//取地址，用于修改
cv::Mat p_ca = cv::Mat::zeros(7,7,CV_32FC1);//用于储存该模型得到的协方差矩阵，初值自定
cv::Mat* pp_ca = &p_ca;//取地址，用于修改

cv::Mat x_ct_e = cv::Mat::zeros(6,1,CV_32FC1);//用于储存匀速转动模型使用EKF得到的结果,顺序x,y,v,fai(方位角),omega(角速度）,z
cv::Mat* px_ct_e = &x_ct_e;//取地址，用于修改
cv::Mat p_ct_e = cv::Mat::zeros(6,6,CV_32FC1);//用于储存该模型得到的协方差矩阵，初值自定
cv::Mat* pp_ct_e = &p_ct_e;//取地址，用于修改

cv::Mat x_cc_e = cv::Mat::zeros(8,1,CV_32FC1);//用于储存匀变速转动模型使用EKF得到的结果,顺序x,y,v,a,fai(方位角),omega(角速度),beta(角加速度),z
cv::Mat* px_cc_e = &x_cc_e;//取地址，用于修改
cv::Mat p_cc_e = cv::Mat::zeros(8,8,CV_32FC1);//用于储存该模型得到的协方差矩阵，初值自定
cv::Mat* pp_cc_e = &p_cc_e;//取地址，用于修改

cv::Mat x_ct_u = cv::Mat::zeros(6,1,CV_32FC1);//用于储存匀速转动模型使用UKF得到的结果,顺序x,y,v,fai(方位角),omega(角速度）,z
cv::Mat* px_ct_u = &x_ct_u;//取地址，用于修改
cv::Mat p_ct_u = cv::Mat::eye(6,6,CV_32FC1);//用于储存该模型得到的协方差矩阵，初值自定
cv::Mat* pp_ct_u = &p_ct_u;//取地址，用于修改

cv::Mat result = cv::Mat::zeros(3,1,CV_32FC1);//存储对下一帧位置的最终预测

int main() 
{
    //cv::Mat cen0 = cv::Mat::zeros(3,1,CV_32FC1); 
    cv::Mat cen0 = (cv::Mat_<float>(3,1)<< 0.98255,0.99985,1);
    //cv::Mat cen1 = cv::Mat::eye(3,1,CV_32FC1); 
    cv::Mat cen1 = (cv::Mat_<float>(3,1)<< 1,1,1);
    //cv::Mat cen2 = cv::Mat::zeros(3,1,CV_32FC1); 
    cv::Mat cen2= (cv::Mat_<float>(3,1)<< 1.01745,0.99985,1.001);
    //cv::Mat cor0 = cv::Mat::zeros(3,1,CV_32FC1); 
    cv::Mat cor0 = (cv::Mat_<float>(3,1)<< 0.88213,1.02310,1);
    //cv::Mat cor1 = cv::Mat::zeros(3,1,CV_32FC1); 
    cv::Mat cor1 = (cv::Mat_<float>(3,1)<< 0.9,1.025,1);
    //cv::Mat cor2 = cv::Mat::zeros(3,1,CV_32FC1); 
    cv::Mat cor2 = (cv::Mat_<float>(3,1)<< 0.91790,1.02659,1);
    //在开始进入滤波器之前，需要有初始数据，最先的三组测量数据将经过处理作为初始数据使用
    //这里我假定已经得到了这三组测量数据，其中装甲板中心点坐标数据保存在cen0,cen1,cen2三个3*1矩阵中，角点坐标保存在cor0,cor1,cor2三个3*1矩阵中
    //这里我不会读入数据，希望得到援助
     //cv::Mat cen_now= cv::Mat::zeros(3,1,CV_32FC1); 
     cv::Mat cen_now= (cv::Mat_<float>(3,1)<< 1.03490,0.99938,0.999);//一个例子
     //cv::Mat cor_now= cv::Mat::zeros(3,1,CV_32FC1); 
     cv::Mat cor_now= (cv::Mat_<float>(3,1)<< 0.93589,1.02786,0.999);
     //cout<<"cen="<<cen_now<<endl;
     //在初始化过后不断读入新的数据，保存在cen_now与cor_now两个矩阵中，这里我还是不会读入
    if(c_v==1)
    {
        init_c_v(cen0,cen1,cen2,px_c_v,pp_c_v);//初始化
        cal_c_v(cen_now,px_c_v,pp_c_v);//计算得到下一帧的预计位置（这步需要循环，大概是用while？）
        //cout<<x_c_v<<endl;//测试用
    }
    if(ca==1)
    {
        init_ca(cen0,cen1,cen2,px_ca,pp_ca);//初始化
        cal_ca(cen_now,px_ca,pp_ca);//计算得到下一帧的预计位置
        //cout<<x_ca<<endl;//测试用
    }
    if(ctrv_ekf==1)
    {
        init_ct_e(cen1,cor1,cen2,cor2,px_ct_e,pp_ct_e);//初始化
        cal_ct_e(cen_now,cor_now,px_ct_e,pp_ct_e);//计算得到下一帧的预计位置
        //cout<<x_ct_e<<endl;//测试用
    }
    if(cca_ekf==1)
    {
        init_cc_e(cen0,cor0,cen1,cor1,cen2,cor2,px_cc_e,pp_cc_e);//初始化
        cout<<x_cc_e<<endl;
        cal_cc_e(cen_now,cor_now,px_cc_e,pp_cc_e);//计算得到下一帧的预计位置
        cout<<x_cc_e<<endl;//测试用
    }
    if(ctrv_ukf==1)
    {
        init_ct_u(cen1,cor1,cen2,cor2,px_ct_u,pp_ct_u);//初始化
        cal_ct_u(cen_now,cor_now,px_ct_u,pp_ct_u);//计算得到下一帧的预计位置
        cout<<x_ct_u<<endl;//测试用
        cout<<p_ct_u<<endl;
    }
    if(c_v+ca+ctrv_ekf+cca_ekf==0) ;//这一行是指，如果所有滤波器都关闭，预测的下一帧位置就是这一帧位置
    if(c_v+ca+ctrv_ekf+cca_ekf==1) ;//这一行是指，如果只有一个滤波器启动了，直接使用这一个滤波器的结果作为最终结果 
    if(c_v+ca+ctrv_ekf+cca_ekf>=2) ;//这一行是指，如果启动了多个滤波器，进行数据融合
    //imageIO();//测试用
    
    return 0;
}
