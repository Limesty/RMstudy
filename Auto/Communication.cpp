#include "Communication.hpp"

//底盘通讯
void Communication::Set_Power(bool power)
{
    Communication::chassis_power = power;
    return;
}

void Communication::Set_Chassis_Front(float speed)
{
    Communication::x_speed = speed;
    return;
}

void Communication::Set_Chassis_Back(float speed)
{
    Communication::x_speed = -speed;
    return;
}

void Communication::Set_Chassis_Left(float speed)
{
    Communication::y_speed = -speed;
    return;
}

void Communication::Set_Chassis_Right(float speed)
{
    Communication::y_speed = speed;
    return;
}

void Communication::Set_Chassis_by_xy(float x_speed, float y_speed)
{
    Communication::x_speed = x_speed;
    Communication::y_speed = y_speed;
    return;
}

//云台通讯
void Communication::Set_Gimbal_Pitch(float pitch_angle, float pitch_speed)
{
    Communication::gimbal.pitch_angle = pitch_angle;
    Communication::gimbal.pitch_speed = pitch_speed;
    return;
}

void Communication::Set_Gimbal_Yaw(float yaw_angle, float yaw_speed)
{
    Communication::gimbal.yaw_angle = yaw_angle;
    Communication::gimbal.yaw_speed = yaw_speed;
    return;
}

void Communication::Set_Gimbal_Roll(float roll_angle, float roll_speed)
{
    Communication::gimbal.roll_angle = roll_angle;
    Communication::gimbal.roll_speed = roll_speed;
    return;
}

GimbalPose Communication::Get_Gimbal_info()
{
    return Communication::gimbal;
}

//射击通讯
void Communication::Open_Box(bool open)
{
    Communication::box = open;
    return;
}

void Communication::Shoot_Start(bool shoot)
{
    Communication::shoot = shoot;
    return;
}

void Communication::Shoot_Bullet(int Bullet_Num, float speed)
{
    Communication::shoot_num = Bullet_Num;
    Communication::shoot_speed = speed;
    return;
}
