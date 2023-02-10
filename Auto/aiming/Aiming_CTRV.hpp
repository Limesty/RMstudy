// 这个模型在描述直线运动时会出问题，这个问题源自于转角fai的取得方法，仅在目标装甲板与运动方向平行的时候才可以正确预测
// 关于fai的方向，我的模型中fai是偏航角，所以fai从x轴非负半轴出发，逆时针为正方向；如果需要的fai是装甲板相对于其旋转中心的夹角，则fai从y轴非正半轴出发，逆时针为正
// 使用的时候只要调用aim_ctrv一个函数就可以了
// all_begin函数里面的变量要事先根据实际情况定义好
#ifndef _AIMING_CTRV_
#define _AIMING_CTRV_

#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <iostream>
using namespace cv;
using namespace std;

class Aiming_CTRV
{
public:
    Aiming_CTRV();
    bool start_to_aim; // 启动瞄准的判断，以具体名称为准
    bool aim_to_work;  // 每当一组新的数据传入时，这个变量变为1,我的内部程序会把它重置为0
    float center_x;
    float center_y;
    float center_z;
    float corner_x;
    float corner_y;
    float corner_z; // 读入每一个时刻的三维坐标，中心点xyz与左上角点xyz
    float predict_x;
    float predict_y;
    float predict_z; // 预测下一帧中心点的xyz坐标（角度与角速度按需要,有需要可以给）
    // float predict_fai;
    // float predict_omega;
    void all_begin(); // 将所有内部数据初始化成初始定义

    Vec3f aim_ctrv(float center_x, float center_y, float center_z, float corner_x, float corner_y, float corner_z); // 调用这个函数来达成输出预测值的目的
private:
    cv::Mat cen;
    cv::Mat cor;
    cv::Mat cen1;
    cv::Mat cor1;
    cv::Mat cen2;
    cv::Mat cor2; // 参数矩阵
    float PI;     // 圆周率
    int count;    // 计数参量，用来记录读取数据的次数
    float dt;     // delta_t采样时间，此处假定为1秒，以真实情况为准
    float s_x2;   // sigma_x的平方,x方向位置测量误差方差，下同理
    float s_y2;
    float s_z2;
    float s_vz2;                                                                                                // z方向速度方差，默认装甲板高度不变（地图好像没有斜坡），只考虑可能的上下抖动,这一项理论上很小
    float s_a2;                                                                                                 // 加速度的标准差
    float s_beta2;                                                                                              // 角加速度标准差
    float s_fai2;                                                                                               // ”测量“（计算）角的误差标准差
    float armor_long;                                                                                           // 装甲板的长边，理论上装在步兵与哨兵上时与地面平行
    float armor_short;                                                                                          // 装甲板的短边，理论上装在步兵与哨兵上时与地面成一固定夹角
    float armor_angle;                                                                                          // 装甲板与地面的固定夹角，单位为度（弧度也行）
    double alpha;                                                                                               // 参数角
    float omega_max;                                                                                            // omega上阈值，在omega的绝对值超过此值时，认定为跨越了角度分界线产生了2PI的误差，采取措施来解决
    float omega_min;                                                                                            // omega下阈值，在omega的绝对值不足此值时，将此模型近似认为是匀速直线运动模型
    cv::Mat x_ct_e;                                                                                             // 用于储存匀速转动模型使用EKF得到的结果,顺序x,y,v,fai(方位角),omega(角速度）,z
    cv::Mat *px_ct_e;                                                                                           // 取地址，用于修改
    cv::Mat p_ct_e;                                                                                             // 用于储存该模型得到的协方差矩阵，初值自定
    cv::Mat *pp_ct_e;                                                                                           // 取地址，用于修改
    void get_matrix(float x, float y, float z, cv::Mat m);                                                      // 将xyz坐标拼成一个3*1矩阵
    void give_matrix(cv::Mat result, float &x, float &y, float &z);                                             // 从矩阵分离出xyz坐标
    float get_fai(cv::Mat cen, cv::Mat cor);                                                                    // 得到fai函数
    float get_v(cv::Mat cen1, cv::Mat cen2);                                                                    // 得到v函数
    void init_ct_e(cv::Mat cen1, cv::Mat cor1, cv::Mat cen2, cv::Mat cor2, cv::Mat *px_ct_e, cv::Mat *pp_ct_e); // 初始化函数
    void cal_ct_e(cv::Mat cen_now, cv::Mat cor_now, cv::Mat *px_ct_e, cv::Mat *pp_ct_e);                        // 预测下一帧函数
};

#endif
