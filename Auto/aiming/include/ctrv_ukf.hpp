//这个是匀速转动算法使用无迹卡尔曼滤波（ctrv_ukf）的头文件
//我对于这个模型的理解仅限于理解原理与会照着公式写，至于这些参数怎么来的，我不会推导
//注意：需要初始的协方差矩阵是对称矩阵，因为使用了cv::eigen函数，用特征值进行了矩阵开方操作
#include"opencv2/opencv.hpp"
#include<iostream>
#include<stdio.h>
#include<math.h>
#include<iostream>
using namespace cv;
using namespace std;

#define PI 3.1415926535898//圆周率，用于三角函数转化
//这个模型的大部分数据在ctrv-ekf里
//下面是ukf的特殊数据
const int n = 8;//状态个数，模型原有的6+噪声2（加速度与角加速度）
const float alp = 0.001;//实际上是alpha，但是变量名称重复了，这一项负责调节高阶项的影响
const float lamda = 3-n;//λ越大，sigma点就越远离状态向量的均值。λ越小，sigma点就越靠近状态向量的均值——复制来的
//const float kappa = 0;
//const float lamda = pow(alp,2)*(n+kappa)-n;
const float bet = 2;//beta,但是又重复了，高斯分布下似乎取2最合适

void init_ct_u(cv::Mat cen1,cv::Mat cor1,cv::Mat cen2,cv::Mat cor2,cv::Mat* px_ct_u,cv::Mat* pp_ct_u)
{
      float fai1 = get_fai(cen1,cor1);
      float fai2 = get_fai(cen2,cor2);
      float omega0 = (fai2-fai1)/dt;
      if(omega0>=omega_max)  omega0 = omega0-2*PI/dt;
      if(omega0<=-omega_max)  omega0 = omega0+2*PI/dt;
      if(omega0>=-omega_min&&omega0<=omega_min) omega0 = 0;
      float v0 = get_v(cen1,cen2);
      cv::Mat cup1 = (cv::Mat_<float>(6,3) << 1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1);
      cv::Mat cup2 = (cv::Mat_<float>(6,1) << 0,0,v0,fai2,omega0,0);
      *px_ct_u = cup1*cen2+cup2;//初始状态，初始6*6协方差矩阵来自于输入
      //下面是ukf
       cv::Mat x_ukf = cv::Mat::zeros(8,1,CV_32FC1);
       cv::Mat cup3 = (cv::Mat_<float>(8,6) << 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1);
       cv::Mat cup3t = cup3.t();
       x_ukf = cup3*(*px_ct_u);//扩维后的状态，顺序为x,y,v,a,fai(方位角),omega(角速度),beta(角加速度),z，其中a和beta默认初始为0
       cv::Mat cup4 = (cv::Mat_<float>(8,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,s_a2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,s_beta2,0,0,0,0,0,0,0,0,0);
       //cout<<"cup4="<<cup4<<endl;
       cv::Mat cup_p = cv::Mat::zeros(8,8,CV_32FC1);
       cup_p = cup4 + cup3*(*pp_ct_u)*cup3t;
       cv::Mat p_values;
       cv::Mat p_vectors;
       cv::eigen(cup_p,p_values,p_vectors);
       cout<<p_values<<endl;
       cout<<p_vectors<<endl;
       cv::sqrt(p_values,p_values);//逐元素开方
       float va1 = p_values.at<float>(0,0);
       float va2 = p_values.at<float>(1,0);
       float va3 = p_values.at<float>(2,0);
       float va4 = p_values.at<float>(3,0);
       float va5 = p_values.at<float>(4,0);
       float va6 = p_values.at<float>(5,0);
       float va7 = p_values.at<float>(6,0);
       float va8 = p_values.at<float>(7,0);
       cv::Mat cup_va = (cv::Mat_<float>(8,8) << va1,0,0,0,0,0,0,0,0,va2,0,0,0,0,0,0,0,0,va3,0,0,0,0,0,0,0,0,va4,0,0,0,0,0,0,0,0,va5,0,0,0,0,0,0,0,0,va6,0,0,0,0,0,0,0,0,va7,0,0,0,0,0,0,0,0,va8);
       cv::Mat p_vectors_i = p_vectors.inv();
       cv::Mat sqr_p = p_vectors_i*cup_va*p_vectors;
       cout<<sqr_p<<endl;
       cv::Mat cup5 = cv::Mat::ones(1,17,CV_32FC1);
       cv::Mat cup6 = (cv::Mat_<float>(8,17) << 0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0);
       cv::Mat cup7 = (cv::Mat_<float>(8,17) << 0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
       //cout<<cup6<<endl;
       //cout<<cup7<<endl;
       cv::Mat x_ukf_17 = cv::Mat::zeros(8,17,CV_32FC1);
       x_ukf_17 = x_ukf*cup5 + sqrt(n+lamda)*sqr_p*cup6 - sqrt(n+lamda)*sqr_p*cup7;
       //cout<<x_ukf_17<<endl;
       cv::Mat cup8 = cv::Mat::zeros(8,1,CV_32FC1);
       for(int j=0;j<=16;j++)
       {
             x_ukf_17.col(j).copyTo(cup8.col(0));
             float fai = cup8.at<float>(4,0);
             float omega = cup8.at<float>(5,0);
             float v = cup8.at<float>(2,0);
             float beta = cup8.at<float>(6,0);
             float a = cup8.at<float>(3,0);
             if((omega>=-omega_min&&omega<=omega_min)||(omega+beta*dt<=omega_min&&omega+beta*dt>=-omega_min)) rot = 0;
             cv::Mat cup0 = cv::Mat::zeros(8,1,CV_32FC1);
             if(rot==0)
             {
                  cup0 = (cv::Mat_<float>(8,1) << (v*dt+a*pow(dt,2)/2)*cos(fai),(v*dt+a*pow(dt,2)/2)*sin(fai),a*dt,0,omega*dt+beta*pow(dt,2)/2,beta*dt,0,0); 
             }
             else
             {
                  cup0 = (cv::Mat_<float>(8,1) << (v+a*dt)*(sin(omega*dt+fai+beta*pow(dt,2)/2)-sin(fai))/(omega+beta*dt),(v+a*dt)*(cos(fai)-cos(omega*dt+fai+pow(dt,2)*beta/2))/(omega+beta*dt),a*dt,0,omega*dt+beta*pow(dt,2)/2,dt*beta,0,0);
             }
             cup8 = cup8+cup0;
             cup8.col(0).copyTo(x_ukf_17.col(j));
        }
        //cout<<x_ukf_17<<endl;
        cv::Mat x_ukf_f = cv::Mat::zeros(6,17,CV_32FC1);
        x_ukf_f = cup3t*x_ukf_17;
        //cout<<x_ukf_f<<endl;
        float wei_m0 =  lamda/(n+lamda);
        float wei_c0 =  lamda/(n+lamda)+1-pow(alp,2)+bet;
        float wei_oth = 1/(2*(n+lamda));
        float wei_m[17] ={ wei_m0,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth };
        //每个结果对于期望值的权重
        float wei_c[17] ={ wei_c0,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth };
        cv::Mat cup9 = (cv::Mat_<float>(17,1) << wei_m0,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth );
        *px_ct_u = x_ukf_f*cup9;
        cv::Mat cup10 = *px_ct_u;
        cv::Mat g = (cv::Mat_<float>(6,3) << pow(dt,2)*cos(fai2)/2,0,0,pow(dt,2)*sin(fai2)/2,0,0,dt,0,0,0,pow(dt,2)/2,0,0,dt,0,0,0,dt);
        cv::Mat gt = g.t();
        cv::Mat u = (cv::Mat_<float>(3,3) << s_a2,0,0,0,s_beta2,0,0,0,s_vz2);
        cv::Mat q = g*u*gt;
        cv::Mat cup11 = *pp_ct_u;
        cup11 = cup11 + q;
        cv::Mat cup12 = cv::Mat::zeros(6,1,CV_32FC1);
        for(int k=0;k<=16;k++)
        {
             x_ukf_f.col(k).copyTo(cup12.col(0));
             cup12 = cup12 - cup10;
             cv::Mat cup12t = cup12.t();
             cup11 = cup11 + wei_c[k]* cup12*cup12t ;
        }
        *pp_ct_u = cup11;
}
void cal_ct_u(cv::Mat cen_now,cv::Mat cor_now,cv::Mat* px_ct_u,cv::Mat* pp_ct_u)
{
      float fai = get_fai(cen_now,cor_now);
      cv::Mat fai_M = (cv::Mat_<float>(4,1) << 0,0,fai,0);
      cv::Mat cup_z =  (cv::Mat_<float>(4,3) << 1,0,0,0,1,0,0,0,0,0,0,1);
      cv::Mat z = cup_z*cen_now+fai_M;
      cv::Mat h = (cv::Mat_<float>(4,6) << 1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1);
      cv::Mat ht = h.t();
      cv::Mat r = (cv::Mat_<float>(4,4) << s_x2,0,0,0,0,s_y2,0,0,0,0,s_fai2,0,0,0,0,s_z2);
       cv::Mat x_ukf = cv::Mat::zeros(8,1,CV_32FC1);
       cv::Mat cup3 = (cv::Mat_<float>(8,6) << 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1);
       cv::Mat cup3t = cup3.t();
       x_ukf = cup3*(*px_ct_u);
       cv::Mat cup4 = (cv::Mat_<float>(8,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,s_a2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,s_beta2,0,0,0,0,0,0,0,0,0);
       //cout<<"cup4="<<cup4<<endl;
       cv::Mat cup_p = cv::Mat::zeros(8,8,CV_32FC1);
       cup_p = cup4 + cup3*(*pp_ct_u)*cup3t;
       cv::Mat p_values;
       cv::Mat p_vectors;
       cv::eigen(cup_p,p_values,p_vectors);
       //cout<<p_values<<endl;
       //cout<<p_vectors<<endl;
       cv::sqrt(p_values,p_values);//逐元素开方
       float va1 = p_values.at<float>(0,0);
       float va2 = p_values.at<float>(1,0);
       float va3 = p_values.at<float>(2,0);
       float va4 = p_values.at<float>(3,0);
       float va5 = p_values.at<float>(4,0);
       float va6 = p_values.at<float>(5,0);
       float va7 = p_values.at<float>(6,0);
       float va8 = p_values.at<float>(7,0);
       cv::Mat cup_va = (cv::Mat_<float>(8,8) << va1,0,0,0,0,0,0,0,0,va2,0,0,0,0,0,0,0,0,va3,0,0,0,0,0,0,0,0,va4,0,0,0,0,0,0,0,0,va5,0,0,0,0,0,0,0,0,va6,0,0,0,0,0,0,0,0,va7,0,0,0,0,0,0,0,0,va8);
       cv::Mat p_vectors_i = p_vectors.inv();
       cv::Mat sqr_p = p_vectors_i*cup_va*p_vectors;
       //cout<<sqr_p<<endl;
       cv::Mat cup5 = cv::Mat::ones(1,17,CV_32FC1);
       cv::Mat cup6 = (cv::Mat_<float>(8,17) << 0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0);
       cv::Mat cup7 = (cv::Mat_<float>(8,17) << 0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
       //cout<<cup6<<endl;
       //cout<<cup7<<endl;
       cv::Mat x_ukf_17 = cv::Mat::zeros(8,17,CV_32FC1);
       x_ukf_17 = x_ukf*cup5 + sqrt(n+lamda)*sqr_p*cup6 - sqrt(n+lamda)*sqr_p*cup7;
       cout<<x_ukf_17<<endl;
        cv::Mat x_ukf_f = cv::Mat::zeros(6,17,CV_32FC1);
        x_ukf_f = cup3t*x_ukf_17;//6*17
        float wei_m0 =  lamda/(n+lamda);
        float wei_c0 =  lamda/(n+lamda)+1-pow(alp,2)+bet;
        float wei_oth = 1/(2*(n+lamda));
        cv::Mat cup9 = (cv::Mat_<float>(17,1) << wei_m0,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth );
        float wei_m[17] ={ wei_m0,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth };
        float wei_c[17] ={ wei_c0,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth,wei_oth };
        cv::Mat z_ukf = cv::Mat::zeros(4,17,CV_32FC1);
        cv::Mat cup13 = (cv::Mat_<float>(4,6) << 1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1);
        z_ukf = cup13*x_ukf_f;//4*17
        cv::Mat z_ukf_f = z_ukf * cup9;//4*1
        cout<<z_ukf_f<<endl;
        cv::Mat p_z = r;
        cv::Mat cup14 = cv::Mat::zeros(4,1,CV_32FC1);
        for(int k=0;k<=16;k++)
        {
             z_ukf.col(k).copyTo(cup14.col(0));
             cup14 = cup14 - z_ukf_f;
             cv::Mat cup14t = cup14.t();
             p_z = p_z + wei_c[k]* cup14*cup14t ;
        }
        cout<<p_z<<endl;
        cv::Mat p_xz = cv::Mat::zeros(6,4,CV_32FC1);
        cv::Mat cup15 = cv::Mat::zeros(6,1,CV_32FC1);
        for(int k=0;k<=16;k++)
        {
             z_ukf.col(k).copyTo(cup14.col(0));
             cup14 = cup14 - z_ukf_f;
             cv::Mat cup14t = cup14.t();
             x_ukf_f.col(k).copyTo(cup15.col(0));
             cup15 = cup15 - (*px_ct_u);
             p_xz = p_xz + wei_c[k]* cup15*cup14t ;
        }
        cout<<p_xz<<endl;
        cv::Mat p_zi = p_z.inv();
        cv::Mat k_k =  p_xz*p_zi;
        cv::Mat k_kt = k_k.t();
        *px_ct_u = *px_ct_u + k_k*(z-z_ukf_f);
        *pp_ct_u = *pp_ct_u - k_k*p_z*k_kt;
}
        
        
        
        
