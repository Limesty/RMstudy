//这个是匀速转动算法使用扩展卡尔曼滤波（ctrv_ekf）的头文件
//在这个模型中，我们认为小车是对称的，换句话说，装甲板的长边被装甲板中心点旋转时的半径垂直且等分
//注意：在这个模型所得到的omega很小时，会采用匀速直线运动模型，但是这个匀速直线运动模型得到的结果是正确的仅限于小车运动方向与装甲板的长边平行时！！！因为在不平行的时候，装甲板长边与x轴夹角不等于小车运动方向与x轴夹角
//这个问题的出现源自于偏航角fai的得到方式，这里我采用了装甲板长边与x轴夹角等效作为偏航角使用
#include"opencv2/opencv.hpp"
#include<iostream>
#include<stdio.h>
#include<math.h>
#include<iostream>
using namespace cv;
using namespace std;

#define PI 3.1415926535898//圆周率，用于三角函数转化

const float s_a2 = 0.1;//加速度的标准差
const float s_beta2 = 0.1;//角加速度标准差

const float s_fai2 = 0.1;//”测量“（计算）角的误差标准差

const float armor_long = 0.2;//装甲板的长边，理论上装在步兵与哨兵上时与地面平行，数据以实际为准
const float armor_short = 0.1;//装甲板的短边，理论上装在步兵与哨兵上时与地面成一固定夹角，数据以实际为准
const float armor_angle = 60;//装甲板与地面的固定夹角，单位为度，数据以实际为准
double alpha = atan(armor_short*cos(armor_angle*PI/180)/armor_long);//参数角

const float omega_max = 100;//omega上阈值，在omega的绝对值超过此值时，认定为跨越了角度分界线产生了2PI的误差，采取措施来解决
const float omega_min = 0.01;//omega下阈值，在omega的绝对值不足此值时，将此模型近似认为是匀速直线运动模型

float get_fai(cv::Mat cen,cv::Mat cor)//利用中心点和角点（默认左上角）坐标求旋转角的函数
//旋转角以x轴非负半轴为0,逆时针为正方向，弧度为单位（也可以角度）
{
      float cen_x = cen.at<float>(0,0);
      float cen_y = cen.at<float>(1,0);
      float cor_x = cor.at<float>(0,0);
      float cor_y = cor.at<float>(1,0);
      double sita = atan((cen_y-cor_y)/(cen_x-cor_x));
      if(cen_x<cor_x) sita = sita+PI;
      cout<<"fai="<<(alpha+sita)<<endl;
      return float(alpha+sita);//来自于几何关系
}  //fai的取值范围是-PI/2+alpha到3*PI/2+alpha，在从后者逆时针转到前者或是前者顺时针转到后者的过程中会出问题，下面会解决

float get_v(cv::Mat cen1,cv::Mat cen2)//得到初始v的函数
{
      float cen1_x = cen1.at<float>(0,0);
      float cen1_y = cen1.at<float>(1,0);
      float cen2_x = cen2.at<float>(0,0);
      float cen2_y = cen2.at<float>(1,0);
      return sqrt(pow(cen2_y-cen1_y,2)+pow(cen2_x-cen1_x,2))/dt;
}

void init_ct_e(cv::Mat cen1,cv::Mat cor1,cv::Mat cen2,cv::Mat cor2,cv::Mat* px_ct_e,cv::Mat* pp_ct_e)
{
      float fai1 = get_fai(cen1,cor1);
      float fai2 = get_fai(cen2,cor2);
      float omega = (fai2-fai1)/dt;
      if(omega>=omega_max)  omega = omega-2*PI/dt;//如果omega绝对值明显过大，表明跨越了分界线，采取措施
      if(omega<=-omega_max)  omega = omega+2*PI/dt;
      if(omega>=-omega_min&&omega<=omega_min) omega = 0;//如果omega绝对值过小，为避免出现运算问题，采用匀速直线模型进行处理 (注意：这个匀速直线运动模型得到的结果是正确的仅限于小车运动方向与装甲板的长边平行时！！！）
      //cout<<"omega="<<omega<<endl;
      float v = get_v(cen1,cen2);
      //cout<<"v="<<v<<endl;
      cv::Mat cup0 = cv::Mat::zeros(6,1,CV_32FC1);
      cv::Mat ja = cv::Mat::zeros(6,6,CV_32FC1);
      if(omega==0) 
      {
            cup0 = (cv::Mat_<float>(6,1) << v*cos(fai2)*dt,v*sin(fai2)*dt,v,fai2+omega*dt,omega,0);
            ja = (cv::Mat_<float>(6,6) << 1,0,cos(fai2)*dt,-v*sin(fai2)*dt,0,0,0,1,dt*sin(fai2),v*dt*cos(fai2),0,0,0,0,1,0,0,0,0,0,0,1,dt,0,0,0,0,0,1,0,0,0,0,0,0,1);
      }     
      else 
      {
            cup0 = (cv::Mat_<float>(6,1) << v*(sin(omega*dt+fai2)-sin(fai2))/omega,v*(cos(fai2)-cos(omega*dt+fai2))/omega,v,fai2+omega*dt,omega,0);
            ja = (cv::Mat_<float>(6,6) << 1,0,(sin(omega*dt+fai2)-sin(fai2))/omega,v*(-cos(fai2)+cos(omega*dt+fai2))/omega,v*(cos(fai2+omega*dt)*dt-(sin(omega*dt+fai2)-sin(fai2))/omega)/omega,0,0,1,(cos(fai2)-cos(omega*dt+fai2))/omega,v*(sin(omega*dt+fai2)-sin(fai2))/omega,v*(sin(fai2+omega*dt)*dt-(cos(fai2)-cos(omega*dt+fai2)/omega))/omega,0,0,0,1,0,0,0,0,0,0,1,dt,0,0,0,0,0,1,0,0,0,0,0,0,1);
      }
      //cout<<"cup0="<<cup0<<endl;
      //cout<<"ja="<<ja<<endl;
      cv::Mat jat = ja.t();
      cv::Mat cup1 = (cv::Mat_<float>(6,3) << 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1);
      //cout<<"cup1="<<cup1<<endl;
      cv::Mat g = (cv::Mat_<float>(6,3) << pow(dt,2)*cos(fai2)/2,0,0,pow(dt,2)*sin(fai2)/2,0,0,dt,0,0,0,pow(dt,2)/2,0,0,dt,0,0,0,dt);
      cv::Mat gt = g.t();
      cv::Mat u = (cv::Mat_<float>(3,3) << s_a2,0,0,0,s_beta2,0,0,0,s_vz2);
      cv::Mat q = g*u*gt;
      *px_ct_e = cup1*cen2+cup0;
      *pp_ct_e = ja*(*pp_ct_e)*jat+q;
}
void cal_ct_e(cv::Mat cen_now,cv::Mat cor_now,cv::Mat* px_ct_e,cv::Mat* pp_ct_e)
{
      float fai = get_fai(cen_now,cor_now);
      cv::Mat fai_M = (cv::Mat_<float>(4,1) << 0,0,fai,0);
      cv::Mat cup9 =  (cv::Mat_<float>(4,3) << 1,0,0,0,1,0,0,0,0,0,0,1);
      cv::Mat z = cup9*cen_now+fai_M;
      cv::Mat h = (cv::Mat_<float>(4,6) << 1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1);
      cv::Mat ht = h.t();
      cv::Mat r = (cv::Mat_<float>(4,4) << s_x2,0,0,0,0,s_y2,0,0,0,0,s_fai2,0,0,0,0,s_z2);     
      cv::Mat cup8 = h*(*pp_ct_e)*ht+r;
      cv::Mat cup8i = cup8.inv();
      cv::Mat k = (*pp_ct_e)*ht*cup8i;    
      cv::Mat i = cv::Mat::eye(6,6,CV_32FC1);
      cv::Mat cup7 = *px_ct_e;
      cv::Mat cup6 = *pp_ct_e;
      cup7 = cup7+ k* (z-h*cup7);
      cup6 = (i-k*h)*cup6;
      *px_ct_e = cup7;
      *pp_ct_e = cup6;//到此为止，数据融合部分完，下面是预测下一帧
      cv::Mat g = (cv::Mat_<float>(6,3) << pow(dt,2)*cos(fai)/2,0,0,pow(dt,2)*sin(fai)/2,0,0,dt,0,0,0,pow(dt,2)/2,0,0,dt,0,0,0,dt);
      cv::Mat gt = g.t();
      cv::Mat u = (cv::Mat_<float>(3,3) << s_a2,0,0,0,s_beta2,0,0,0,s_vz2);
      cv::Mat q = g*u*gt;

      float omega = (*px_ct_e).at<float>(4,0);
      float v = (*px_ct_e).at<float>(2,0);
      if(omega>=-omega_min&&omega<=omega_min) omega = 0;
      cv::Mat cup0 = cv::Mat::zeros(6,1,CV_32FC1);
      cv::Mat ja = cv::Mat::zeros(6,6,CV_32FC1);
      if(omega==0) 
      {
            cup0 = (cv::Mat_<float>(6,1) << v*cos(fai)*dt,v*sin(fai)*dt,0,0,0,0);
            ja = (cv::Mat_<float>(6,6) << 1,0,cos(fai)*dt,-v*sin(fai)*dt,0,0,0,1,dt*sin(fai),dt*cos(fai)*v,0,0,0,0,1,0,0,0,0,0,0,1,dt,0,0,0,0,0,1,0,0,0,0,0,0,1);
      }     
      else 
      {
            cup0 = (cv::Mat_<float>(6,1) << v*(sin(omega*dt+fai)-sin(fai))/omega,v*(cos(fai)-cos(omega*dt+fai))/omega,0,omega*dt,0,0);
            ja = (cv::Mat_<float>(6,6) << 1,0,(sin(omega*dt+fai)-sin(fai))/omega,v*(-cos(fai)+cos(omega*dt+fai))/omega,v*(cos(fai+omega*dt)*dt-(sin(omega*dt+fai)-sin(fai))/omega)/omega,0,0,1,(cos(fai)-cos(omega*dt+fai))/omega,v*(sin(omega*dt+fai)-sin(fai))/omega,v*(sin(fai+omega*dt)*dt-(cos(fai)-cos(omega*dt+fai)/omega))/omega,0,0,0,1,0,0,0,0,0,0,1,dt,0,0,0,0,0,1,0,0,0,0,0,0,1);
      }
      cv::Mat jat = ja.t();
      *px_ct_e = *px_ct_e+cup0;
      *pp_ct_e = ja*(*pp_ct_e)*jat+q;
}
      
      
      
      

      
