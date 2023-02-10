#include "Aiming_CTRV.hpp"

Aiming_CTRV::Aiming_CTRV()
{
      all_begin();
}
void Aiming_CTRV::get_matrix(float x, float y, float z, cv::Mat m)
{
      m = (cv::Mat_<float>(3, 1) << x, y, z);
}
void Aiming_CTRV::give_matrix(cv::Mat result, float &x, float &y, float &z)
{
      x = result.at<float>(0, 0);
      y = result.at<float>(1, 0);
      // fai与omega也可以从这里得到
      z = result.at<float>(5, 0);
}

void Aiming_CTRV::all_begin()
{
      cv::Mat cen = cv::Mat::zeros(3, 1, CV_32FC1);
      cv::Mat cor = cv::Mat::zeros(3, 1, CV_32FC1);
      cv::Mat cen1 = cv::Mat::zeros(3, 1, CV_32FC1);
      cv::Mat cor1 = cv::Mat::zeros(3, 1, CV_32FC1);
      cv::Mat cen2 = cv::Mat::zeros(3, 1, CV_32FC1);
      cv::Mat cor2 = cv::Mat::zeros(3, 1, CV_32FC1);
      PI = 3.1415926535898;                                                 // 圆周率，用于三角函数转化
      count = 0;                                                            // 计数参量，用来记录读取数据的次数
      s_a2 = 0.1;                                                           // 加速度的标准差
      s_beta2 = 0.1;                                                        // 角加速度标准差
      s_fai2 = 0.1;                                                         // ”测量“（计算）角的误差标准差
      armor_long = 0.2;                                                     // 装甲板的长边，理论上装在步兵与哨兵上时与地面平行
      armor_short = 0.1;                                                    // 装甲板的短边，理论上装在步兵与哨兵上时与地面成一固定夹角
      armor_angle = 60;                                                     // 装甲板与地面的固定夹角，单位为度（弧度也行）
      alpha = atan(armor_short * cos(armor_angle * PI / 180) / armor_long); // 参数角
      omega_max = 100;                                                      // omega上阈值，在omega的绝对值超过此值时，认定为跨越了角度分界线产生了2PI的误差，采取措施来解决
      omega_min = 0.01;                                                     // omega下阈值，在omega的绝对值不足此值时，将此模型近似认为是匀速直线运动模型
      dt = 1;                                                               // delta_t采样时间，此处假定为1秒，以真实情况为准
      s_x2 = 0.1;                                                           // sigma_x的平方,x方向位置测量误差方差，下同理
      s_y2 = 0.1;
      s_z2 = 0.1;
      s_vz2 = 0.001;                                   // z方向速度方差，默认装甲板高度不变（地图好像没有斜坡），只考虑可能的上下抖动,这一项理论上很小
      cv::Mat x_ct_e = cv::Mat::zeros(6, 1, CV_32FC1); // 用于储存匀速转动模型使用EKF得到的结果,顺序x,y,v,fai(方位角),omega(角速度）,z
      cv::Mat *px_ct_e = &x_ct_e;                      // 取地址，用于修改
      cv::Mat p_ct_e = cv::Mat::zeros(6, 6, CV_32FC1); // 用于储存该模型得到的协方差矩阵，初值自定
      cv::Mat *pp_ct_e = &p_ct_e;                      // 取地址，用于修改
}

float Aiming_CTRV::get_fai(cv::Mat cen, cv::Mat cor) // 利用中心点和角点（默认左上角）坐标求旋转角的函数
// 旋转角以x轴非负半轴为0,逆时针为正方向，弧度为单位（也可以角度）
{
      float cen_x = cen.at<float>(0, 0);
      float cen_y = cen.at<float>(1, 0);
      float cor_x = cor.at<float>(0, 0);
      float cor_y = cor.at<float>(1, 0);
      double sita = atan((cen_y - cor_y) / (cen_x - cor_x));
      if (cen_x < cor_x)
            sita = sita + PI;
      // cout<<"fai="<<(alpha+sita)<<endl;
      return float(alpha + sita); // 来自于几何关系
} // fai的取值范围是-PI/2+alpha到3*PI/2+alpha，在从后者逆时针转到前者或是前者顺时针转到后者的过程中会出问题，下面会解决

float Aiming_CTRV::get_v(cv::Mat cen1, cv::Mat cen2) // 得到初始v的函数
{
      float cen1_x = cen1.at<float>(0, 0);
      float cen1_y = cen1.at<float>(1, 0);
      float cen2_x = cen2.at<float>(0, 0);
      float cen2_y = cen2.at<float>(1, 0);
      return sqrt(pow(cen2_y - cen1_y, 2) + pow(cen2_x - cen1_x, 2)) / dt;
}

void Aiming_CTRV::init_ct_e(cv::Mat cen1, cv::Mat cor1, cv::Mat cen2, cv::Mat cor2, cv::Mat *px_ct_e, cv::Mat *pp_ct_e)
{
      float fai1 = get_fai(cen1, cor1);
      float fai2 = get_fai(cen2, cor2);
      float omega = (fai2 - fai1) / dt;
      if (omega >= omega_max)
            omega = omega - 2 * PI / dt; // 如果omega绝对值明显过大，表明跨越了分界线，采取措施
      if (omega <= -omega_max)
            omega = omega + 2 * PI / dt;
      if (omega >= -omega_min && omega <= omega_min)
            omega = 0; // 如果omega绝对值过小，为避免出现运算问题，采用匀速直线模型进行处理 (注意：这个匀速直线运动模型得到的结果是正确的仅限于小车运动方向与装甲板的长边平行时！！！）
      // cout<<"omega="<<omega<<endl;
      float v = get_v(cen1, cen2);
      // cout<<"v="<<v<<endl;
      cv::Mat cup0 = cv::Mat::zeros(6, 1, CV_32FC1);
      cv::Mat ja = cv::Mat::zeros(6, 6, CV_32FC1);
      if (omega == 0)
      {
            cup0 = (cv::Mat_<float>(6, 1) << v * cos(fai2) * dt, v * sin(fai2) * dt, v, fai2 + omega * dt, omega, 0);
            ja = (cv::Mat_<float>(6, 6) << 1, 0, cos(fai2) * dt, -v * sin(fai2) * dt, 0, 0, 0, 1, dt * sin(fai2), v * dt * cos(fai2), 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
      }
      else
      {
            cup0 = (cv::Mat_<float>(6, 1) << v * (sin(omega * dt + fai2) - sin(fai2)) / omega, v * (cos(fai2) - cos(omega * dt + fai2)) / omega, v, fai2 + omega * dt, omega, 0);
            ja = (cv::Mat_<float>(6, 6) << 1, 0, (sin(omega * dt + fai2) - sin(fai2)) / omega, v * (-cos(fai2) + cos(omega * dt + fai2)) / omega, v * (cos(fai2 + omega * dt) * dt - (sin(omega * dt + fai2) - sin(fai2)) / omega) / omega, 0, 0, 1, (cos(fai2) - cos(omega * dt + fai2)) / omega, v * (sin(omega * dt + fai2) - sin(fai2)) / omega, v * (sin(fai2 + omega * dt) * dt - (cos(fai2) - cos(omega * dt + fai2) / omega)) / omega, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
      }
      // cout<<"cup0="<<cup0<<endl;
      // cout<<"ja="<<ja<<endl;
      cv::Mat jat = ja.t();
      cv::Mat cup1 = (cv::Mat_<float>(6, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
      // cout<<"cup1="<<cup1<<endl;
      cv::Mat g = (cv::Mat_<float>(6, 3) << pow(dt, 2) * cos(fai2) / 2, 0, 0, pow(dt, 2) * sin(fai2) / 2, 0, 0, dt, 0, 0, 0, pow(dt, 2) / 2, 0, 0, dt, 0, 0, 0, dt);
      cv::Mat gt = g.t();
      cv::Mat u = (cv::Mat_<float>(3, 3) << s_a2, 0, 0, 0, s_beta2, 0, 0, 0, s_vz2);
      cv::Mat q = g * u * gt;
      *px_ct_e = cup1 * cen2 + cup0;
      *pp_ct_e = ja * (*pp_ct_e) * jat + q;
}
void Aiming_CTRV::cal_ct_e(cv::Mat cen_now, cv::Mat cor_now, cv::Mat *px_ct_e, cv::Mat *pp_ct_e)
{
      float fai = get_fai(cen_now, cor_now);
      cv::Mat fai_M = (cv::Mat_<float>(4, 1) << 0, 0, fai, 0);
      cv::Mat cup9 = (cv::Mat_<float>(4, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
      cv::Mat z = cup9 * cen_now + fai_M;
      cv::Mat h = (cv::Mat_<float>(4, 6) << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1);
      cv::Mat ht = h.t();
      cv::Mat r = (cv::Mat_<float>(4, 4) << s_x2, 0, 0, 0, 0, s_y2, 0, 0, 0, 0, s_fai2, 0, 0, 0, 0, s_z2);
      cv::Mat cup8 = h * (*pp_ct_e) * ht + r;
      cv::Mat cup8i = cup8.inv();
      cv::Mat k = (*pp_ct_e) * ht * cup8i;
      cv::Mat i = cv::Mat::eye(6, 6, CV_32FC1);
      cv::Mat cup7 = *px_ct_e;
      cv::Mat cup6 = *pp_ct_e;
      cup7 = cup7 + k * (z - h * cup7);
      cup6 = (i - k * h) * cup6;
      *px_ct_e = cup7;
      *pp_ct_e = cup6; // 到此为止，数据融合部分完，下面是预测下一帧
      cv::Mat g = (cv::Mat_<float>(6, 3) << pow(dt, 2) * cos(fai) / 2, 0, 0, pow(dt, 2) * sin(fai) / 2, 0, 0, dt, 0, 0, 0, pow(dt, 2) / 2, 0, 0, dt, 0, 0, 0, dt);
      cv::Mat gt = g.t();
      cv::Mat u = (cv::Mat_<float>(3, 3) << s_a2, 0, 0, 0, s_beta2, 0, 0, 0, s_vz2);
      cv::Mat q = g * u * gt;

      float omega = (*px_ct_e).at<float>(4, 0);
      float v = (*px_ct_e).at<float>(2, 0);
      if (omega >= -omega_min && omega <= omega_min)
            omega = 0;
      cv::Mat cup0 = cv::Mat::zeros(6, 1, CV_32FC1);
      cv::Mat ja = cv::Mat::zeros(6, 6, CV_32FC1);
      if (omega == 0)
      {
            cup0 = (cv::Mat_<float>(6, 1) << v * cos(fai) * dt, v * sin(fai) * dt, 0, 0, 0, 0);
            ja = (cv::Mat_<float>(6, 6) << 1, 0, cos(fai) * dt, -v * sin(fai) * dt, 0, 0, 0, 1, dt * sin(fai), dt * cos(fai) * v, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
      }
      else
      {
            cup0 = (cv::Mat_<float>(6, 1) << v * (sin(omega * dt + fai) - sin(fai)) / omega, v * (cos(fai) - cos(omega * dt + fai)) / omega, 0, omega * dt, 0, 0);
            ja = (cv::Mat_<float>(6, 6) << 1, 0, (sin(omega * dt + fai) - sin(fai)) / omega, v * (-cos(fai) + cos(omega * dt + fai)) / omega, v * (cos(fai + omega * dt) * dt - (sin(omega * dt + fai) - sin(fai)) / omega) / omega, 0, 0, 1, (cos(fai) - cos(omega * dt + fai)) / omega, v * (sin(omega * dt + fai) - sin(fai)) / omega, v * (sin(fai + omega * dt) * dt - (cos(fai) - cos(omega * dt + fai) / omega)) / omega, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
      }
      cv::Mat jat = ja.t();
      *px_ct_e = *px_ct_e + cup0;
      *pp_ct_e = ja * (*pp_ct_e) * jat + q;
}
Vec3f Aiming_CTRV::aim_ctrv(float center_x, float center_y, float center_z, float corner_x, float corner_y, float corner_z)
{
      float predict_x, predict_y, predict_z;
      aim_to_work = 0;
      count = count + 1;
      get_matrix(center_x, center_y, center_z, cen);
      get_matrix(corner_x, corner_y, corner_z, cor);

      if (count == 1)
      {
            cen1 = cen;
            cor1 = cor;
            predict_x = center_x;
            predict_y = center_y;
            predict_z = center_z;
      }
      else if (count == 2)
      {
            cen2 = cen;
            cor2 = cor;
            init_ct_e(cen1, cor1, cen2, cor2, px_ct_e, pp_ct_e);
            give_matrix(x_ct_e, predict_x, predict_y, predict_z);
      }
      else
      {
            cal_ct_e(cen, cor, px_ct_e, pp_ct_e);
            give_matrix(x_ct_e, predict_x, predict_y, predict_z);
      }
}
