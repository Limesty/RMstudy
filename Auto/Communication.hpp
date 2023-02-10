#pragma once

#define LEVEL_MAX 3//最高等级
#define CHASSIS_TYPE_NUM 3//底盘种类数
#define SHOOTER_TYPE_NUM 3//发射机构种类数

struct GimbalPose//云台各项角度与速度
{
    float pitch_angle;//云台俯仰角
    float pitch_speed;//云台俯仰速度
    float yaw_angle;//云台航向角
    float yaw_speed;//云台航向速度
    float roll_angle;//云台翻滚角
    float roll_speed;//云台翻滚速度
};

struct Robot_Info//向其他机器人发送的信息包
{
    int ID;//身份标识
    int HP_Cur;//当前血量
    int Heat_cur;//当前枪口热量
    int Buttet_type;//弹药种类
    int Buttet_Num;//弹药数量
    int Level;//等级
    int EXP;//经验值
    int Chassis_type;//底盘种类
    int Shooter_type;//发射机构种类
    location location;//位置
};

struct Chassis_Data//与底盘种类相关的，机器人随等级发生变化的属性
{
    int HP_Max;//血量上限
    int Power_Max;//功率上限
};

struct shooter_Data//与发射机构种类相关的，机器人随等级发生变化的属性
{
    int Heat_Max;//枪口热量上限
    int CD_pers;//枪口热量每秒冷却值
    int Shoot_Speed_Max;//射击初速度上限
};

//将等级与底盘种类、发射机构种类的关系储存在本地
struct
{
    Chassis_Data *chassis[CHASSIS_TYPE_NUM];//指向该等级的底盘数据表
    shooter_Data *shooter[SHOOTER_TYPE_NUM];//指向该等级的发射机构数据表
} Level_Data_Table[LEVEL_MAX];//按照等级分为若干组数据指针

struct location//机器人位置（二维定位）
{
    float x;//横坐标
    float y;//纵坐标
};

class Communication//通讯系统
{
    float timestamp;//时间戳，随信息包一起发送，用于收到信息时比较更新时间
    bool chassis_power;//底盘开关
    float x_speed;//x方向速度
    float y_speed;//y方向速度
    bool box;//弹药仓开关
    bool shoot;//射击开关
    int buttet_num;//剩余弹药数
    int shoot_num;//剩余要发射的弹药数
    int shoot_speed;//射击速度
    GimbalPose gimbal;//云台数据
    Robot_Info info;//机器人的当前信息包
    location location;//机器人位置

public:
    Communication();
    ~Communication();

    //底盘通讯
    void Set_Power(bool power);//开关底盘
    void Set_Chassis_Front(float speed);//设置向前方向速度
    void Set_Chassis_Back(float speed);//设置向后方向速度
    void Set_Chassis_Left(float speed);//设置向左方向速度
    void Set_Chassis_Right(float speed);//设置向右方向速度
    void Set_Chassis_by_xy(float x_speed, float y_speed);//同时设置x与y方向速度
    void Set_Chassis_by_angle(float angle, float speed);//设置相对于某固定方向的角度与速度

    //云台通讯
    void Set_Gimbal_Pitch(float pitch_angle, float pitch_speed);//设置云台俯仰角与速度
    void Set_Gimbal_Yaw(float yaw_angle, float yaw_speed);//设置云台航向角与速度
    void Set_Gimbal_Roll(float roll_angle, float roll_speed);//设置云台翻滚角与速度
    GimbalPose Get_Gimbal_info();//返回当前云台的各项角度与速度

    //射击通讯
    void Open_Box(bool);//打开与关闭弹药仓
    void Shoot_Start(bool);//开始与停止射击
    void Shoot_Bullet(int Bullet_Num, float speed);//以指定的的速度，射击指定数量的弹药

    //机器人通讯
    Robot_Info *Recv_Robot_Info();//接收其他机器人发来的信息包
    void handle_Robot_Info(Robot_Info *packet);//处理其他机器人发来的信息包
    void send_Robot_Info(int ID, Robot_Info *packet);//向指定ID的机器人发送信息包
};