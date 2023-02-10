//这个是匀变速转动算法使用扩展卡尔曼滤波（cca_ekf）的头文件
//由于取角方法与ctrv_ekf相同，在使用这个模型处理匀变速直线运动时会遇到相同的问题，即只能处理与装甲板长边平行方向上的运动
//目前的思路：
//omega不为0,转动模型
//omega与beta均为0,直线运动模型
//omega为0,而beta不为0,目前的想法是采用直线运动模型，这一块的处理方式可能不合适，具体调试时再说
#include"opencv2/opencv.hpp"
#include<iostream>
#include<stdio.h>
#include<math.h>
#include<iostream>
using namespace cv;
using namespace std;

#define PI 3.1415926535898//圆周率，用于三角函数转化

const float s_adot2 = 0.1;//急剧度（加速度的导数）的标准差
const float s_betadot2 = 0.1;//角加速度的导数的标准差

const float beta_max = 100;//与omega_max同理
const float beta_min = 0.01;//与omega_min同理,暂时未使用

bool rot = 1;//为了解决这个模型的一个潜在问题而设置的变量
//之前在处理ctrv模型中omega过小的问题时使用了在omega较小时视为0的操作，在控制好判断阈值的情况下，这一步在匀速转动模型中不太会出问题
//但是在匀变速转动模型里，由于beta的存在，在某些特殊情况下，可能会出现omega为0而beta不为0的情况
//这种情况下，如果阈值q>dt*beta，每进行一次迭代都会归零，造成omega“永远为0”的情况，可能会造成误差
//为了避免这种情况发生，omega在这个模型中不再归零，改用一个变量rot（rotate）来判断omega情况
//omega过小的时候，rot=0,采用直线运动模型；omega大小正常，rot=1,采用旋转模型

float get_a(cv::Mat cen0,cv::Mat cen1,cv::Mat cen2)
{
      return (get_v(cen1,cen2)-get_v(cen0,cen1))/dt;
}
void init_cc_e(cv::Mat cen0,cv::Mat cor0,cv::Mat cen1,cv::Mat cor1,cv::Mat cen2,cv::Mat cor2,cv::Mat* px_cc_e,cv::Mat* pp_cc_e)
{
      float fai0 = get_fai(cen0,cor0);
      float fai1 = get_fai(cen1,cor1);
      float fai2 = get_fai(cen2,cor2);
      float omega = (fai2-fai1)/dt;
      float beta = (fai2+fai0-2*fai1)/pow(dt,2);
      if(omega>=omega_max)  omega = omega-2*PI/dt;//如果omega绝对值明显过大，表明跨越了分界线，采取措施
      if(omega<=-omega_max)  omega = omega+2*PI/dt;
      if(beta>=beta_max)  beta = beta-2*PI/pow(dt,2);//如果beta绝对值明显过大,同理
      if(beta<=-beta_max) beta = beta+2*PI/pow(dt,2);
      if((omega>=-omega_min&&omega<=omega_min)||(omega+beta*dt<=omega_min&&omega+beta*dt>=-omega_min)) rot = 0;//如果omega绝对值过小，为避免出现运算问题，采用匀速直线模型进行处理 
      //现在不确定omega为0而beta不为0会不会有问题，后面再说
      //cout<<"omega="<<omega<<endl;
      float v = get_v(cen1,cen2);
      float a = get_a(cen0,cen1,cen2);
      //cout<<"v="<<v<<endl;
      cv::Mat cup0 = cv::Mat::zeros(8,1,CV_32FC1);
      cv::Mat ja = cv::Mat::zeros(8,8,CV_32FC1);
      if(rot==0)//这里作了近似，认为omega为0的这个单位时间以内小车进行的是匀变速直线运动，只有在beta比较小的时候才会比较精确，至于beta比较大的情况看有没有需要
   {
      cup0 = (cv::Mat_<float>(8,1) << (v*dt+a*pow(dt,2)/2)*cos(fai2),(v*dt+a*pow(dt,2)/2)*sin(fai2),v+a*dt,a,fai2+omega*dt+beta*pow(dt,2)/2,omega+dt*beta,beta,0); 
      ja = (cv::Mat_<float>(8,8) <<  1,0,dt*cos(fai2),pow(dt,2)*cos(fai2)/2,-(v*dt+a*pow(dt,2)/2)*sin(fai2),-(v*dt+a*pow(dt,2)/2)*sin(fai2)*dt,-(v*dt+a*pow(dt,2)/2)*sin(fai2)*pow(dt,2)/2,0,0,1,dt*sin(fai2),pow(dt,2)*sin(fai2)/2,(v*dt+a*pow(dt,2)/2)*cos(fai2),(v*dt+a*pow(dt,2)/2)*cos(fai2)*dt,(v*dt+a*pow(dt,2)/2)*cos(fai2)*pow(dt,2)/2,0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,dt,pow(dt,2)/2,0,0,0,0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1);
   }
      else
   {
      cup0 = (cv::Mat_<float>(8,1) << (v+a*dt)*(sin(omega*dt+fai2+beta*pow(dt,2)/2)-sin(fai2))/(omega+beta*dt),(v+a*dt)*(cos(fai2)-cos(omega*dt+fai2+pow(dt,2)*beta/2))/(omega+beta*dt),v+a*dt,a,fai2+omega*dt+beta*pow(dt,2)/2,omega+dt*beta,beta,0);
      ja = (cv::Mat_<float>(8,8) << 1,0,(sin(omega*dt+fai2+beta*pow(dt,2)/2)-sin(fai2))/(omega+beta*dt),dt*(sin(omega*dt+fai2+beta*pow(dt,2)/2)-sin(fai2))/(omega+beta*dt),(v+a*dt)*(-cos(fai2)+cos(beta*pow(dt,2)/2+omega*dt+fai2))/(omega+beta*dt),(v+dt*a)*(cos(fai2+omega*dt+beta*pow(dt,2)/2)*dt-(sin(omega*dt+fai2+beta*pow(dt,2)/2)-sin(fai2))/(omega+beta*dt))/(omega+beta*dt),(v+dt*a)*(cos(fai2+omega*dt+beta*pow(dt,2)/2)*pow(dt,2)/2-dt*(sin(omega*dt+fai2+beta*pow(dt,2)/2)-sin(fai2))/(omega+beta*dt))/(omega+beta*dt),0,0,1,(cos(fai2)-cos(omega*dt+fai2+beta*pow(dt,2)/2))/(omega+beta*dt),dt*(cos(fai2)-cos(omega*dt+fai2+beta*pow(dt,2)/2))/(omega+beta*dt),(v+a*dt)*(sin(omega*dt+fai2+beta*pow(dt,2)/2)-sin(fai2))/(omega+beta*dt),(v+a*dt)*(sin(fai2+omega*dt+beta*pow(dt,2)/2)*dt-(cos(fai2)-cos(omega*dt+fai2+beta*pow(dt,2)/2))/(omega+beta*dt))/(omega+beta*dt),(v+a*dt)*(sin(fai2+omega*dt+beta*pow(dt,2)/2)*pow(dt,2)/2-dt*(cos(fai2)-cos(omega*dt+fai2+beta*pow(dt,2)/2))/(omega+beta*dt))/(omega+beta*dt),0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,dt,pow(dt,2)/2,0,0,0,0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1);
   }
      cout<<"cup0="<<cup0<<endl;
      cout<<"ja="<<ja<<endl;
      cv::Mat jat = ja.t();
      cv::Mat cup1 = (cv::Mat_<float>(8,3) << 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
      cv::Mat g = (cv::Mat_<float>(8,3) << pow(dt,3)*cos(fai2)/6,0,0,pow(dt,3)*sin(fai2)/6,0,0,pow(dt,2)/2,0,0,dt,0,0,0,pow(dt,3)/6,0,0,pow(dt,2)/2,0,0,dt,0,0,0,dt);
      cv::Mat gt = g.t();
      cv::Mat u = (cv::Mat_<float>(3,3) << s_adot2,0,0,0,s_betadot2,0,0,0,s_vz2);
      cv::Mat q = g*u*gt;
      *px_cc_e = cup1*cen2+cup0;
      *pp_cc_e = ja*(*pp_cc_e)*jat+q;
}
void cal_cc_e(cv::Mat cen_now,cv::Mat cor_now,cv::Mat* px_cc_e,cv::Mat* pp_cc_e)
{
      float fai = get_fai(cen_now,cor_now);
      cv::Mat fai_M = (cv::Mat_<float>(4,1) << 0,0,fai,0);
      cv::Mat cup9 =  (cv::Mat_<float>(4,3) << 1,0,0,0,1,0,0,0,0,0,0,1);
      cv::Mat z = cup9*cen_now+fai_M;
      cv::Mat h = (cv::Mat_<float>(4,8) << 1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1);
      cv::Mat ht = h.t();
      cv::Mat r = (cv::Mat_<float>(4,4) << s_x2,0,0,0,0,s_y2,0,0,0,0,s_fai2,0,0,0,0,s_z2);     
      cv::Mat cup8 = h*(*pp_cc_e)*ht+r;
      cv::Mat cup8i = cup8.inv();
      cv::Mat k = (*pp_cc_e)*ht*cup8i;    
      cv::Mat i = cv::Mat::eye(8,8,CV_32FC1);
      cv::Mat cup7 = *px_cc_e;
      cv::Mat cup6 = *pp_cc_e;
      cup7 = cup7+ k* (z-h*cup7);
      cup6 = (i-k*h)*cup6;
      *px_cc_e = cup7;
      *pp_cc_e = cup6;//到此为止，数据融合部分完，下面是预测下一帧
      cv::Mat g = (cv::Mat_<float>(8,3) << pow(dt,3)*cos(fai)/6,0,0,pow(dt,3)*sin(fai)/6,0,0,pow(dt,2)/2,0,0,dt,0,0,0,pow(dt,3)/6,0,0,pow(dt,2)/2,0,0,dt,0,0,0,dt);
      cv::Mat gt = g.t();
      cv::Mat u = (cv::Mat_<float>(3,3) << s_adot2,0,0,0,s_betadot2,0,0,0,s_vz2);
      cv::Mat q = g*u*gt;
      float omega = (*px_cc_e).at<float>(5,0);
      float v = (*px_cc_e).at<float>(2,0);
      float beta = (*px_cc_e).at<float>(6,0);
      float a = (*px_cc_e).at<float>(3,0);
      if((omega>=-omega_min&&omega<=omega_min)||(omega+beta*dt<=omega_min&&omega+beta*dt>=-omega_min)) rot = 0;
      cv::Mat cup0 = cv::Mat::zeros(8,1,CV_32FC1);
      cv::Mat ja = cv::Mat::zeros(8,8,CV_32FC1);
      if(rot==0)
   {
      cup0 = (cv::Mat_<float>(8,1) << (v*dt+a*pow(dt,2)/2)*cos(fai),(v*dt+a*pow(dt,2)/2)*sin(fai),a*dt,0,omega*dt+beta*pow(dt,2)/2,beta*dt,0,0); 
      ja = (cv::Mat_<float>(8,8) <<  1,0,dt*cos(fai),pow(dt,2)*cos(fai)/2,-(v*dt+a*pow(dt,2)/2)*sin(fai),-(v*dt+a*pow(dt,2)/2)*sin(fai)*dt,-(v*dt+a*pow(dt,2)/2)*sin(fai)*pow(dt,2)/2,0,0,1,dt*sin(fai),pow(dt,2)*sin(fai)/2,(v*dt+a*pow(dt,2)/2)*cos(fai),(v*dt+a*pow(dt,2)/2)*cos(fai)*dt,(v*dt+a*pow(dt,2)/2)*cos(fai)*pow(dt,2)/2,0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,dt,pow(dt,2)/2,0,0,0,0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1);
   }
      else
   {
      cup0 = (cv::Mat_<float>(8,1) << (v+a*dt)*(sin(omega*dt+fai+beta*pow(dt,2)/2)-sin(fai))/(omega+beta*dt),(v+a*dt)*(cos(fai)-cos(omega*dt+fai+pow(dt,2)*beta/2))/(omega+beta*dt),a*dt,0,omega*dt+beta*pow(dt,2)/2,dt*beta,0,0);
      ja = (cv::Mat_<float>(8,8) << 1,0,(sin(omega*dt+fai+beta*pow(dt,2)/2)-sin(fai))/(omega+beta*dt),dt*(sin(omega*dt+fai+beta*pow(dt,2)/2)-sin(fai))/(omega+beta*dt),(v+a*dt)*(-cos(fai)+cos(beta*pow(dt,2)/2+omega*dt+fai))/(omega+beta*dt),(v+dt*a)*(cos(fai+omega*dt+beta*pow(dt,2)/2)*dt-(sin(omega*dt+fai+beta*pow(dt,2)/2)-sin(fai))/(omega+beta*dt))/(omega+beta*dt),(v+dt*a)*(cos(fai+omega*dt+beta*pow(dt,2)/2)*pow(dt,2)/2-dt*(sin(omega*dt+fai+beta*pow(dt,2)/2)-sin(fai))/(omega+beta*dt))/(omega+beta*dt),0,0,1,(cos(fai)-cos(omega*dt+fai+beta*pow(dt,2)/2))/(omega+beta*dt),dt*(cos(fai)-cos(omega*dt+fai+beta*pow(dt,2)/2))/(omega+beta*dt),(v+a*dt)*(sin(omega*dt+fai+beta*pow(dt,2)/2)-sin(fai))/(omega+beta*dt),(v+a*dt)*(sin(fai+omega*dt+beta*pow(dt,2)/2)*dt-(cos(fai)-cos(omega*dt+fai+beta*pow(dt,2)/2))/(omega+beta*dt))/(omega+beta*dt),(v+a*dt)*(sin(fai+omega*dt+beta*pow(dt,2)/2)*pow(dt,2)/2-dt*(cos(fai)-cos(omega*dt+fai+beta*pow(dt,2)/2))/(omega+beta*dt))/(omega+beta*dt),0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,dt,pow(dt,2)/2,0,0,0,0,0,0,1,dt,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1);
   }
      cv::Mat jat = ja.t();
      *px_cc_e = *px_cc_e+cup0;
      *pp_cc_e = ja*(*pp_cc_e)*jat+q;
}
