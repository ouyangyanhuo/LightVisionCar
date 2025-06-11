/*
 * 
 *  Name: MotorControl.c
 *  Created on: 2025/2/23
 *  Author: Magneto
 * 
 */

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     电机初始化函数
//  返回参数     Nope
//  使用示例     Motor_Init();
//  备注信息     Nope
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    pwm_init(P21_4A, 17 * 1000, 0);
    pwm_init(P21_2B, 17 * 1000, 0);
    pwm_init(P21_5A, 17 * 1000, 0);
    pwm_init(P21_3B, 17 * 1000, 0);
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     电机控制函数
//  返回参数     Nope
//  使用示例     MotorControl(1000, 1000);
//  备注信息     推荐输出不应当超过 6000，最大不应当超过 10000
//-------------------------------------------------------------------------------------------------------------------
void MotorControl(int PWM_1, int PWM_2)
{
    // 左电机控制
    if (PWM_1 >= 0) {
        pwm_set_duty(P21_2B, 0); // 设置PWM占空比
        pwm_set_duty(P21_4A, PWM_1);    // 正转方向信号
    }
    else {
        pwm_set_duty(P21_2B, -PWM_1); // 设置PWM占空比
        pwm_set_duty(P21_4A, 0);    // 正转方向信号
    }

    // 右电机控制
    if (PWM_2 >= 0) {
        pwm_set_duty(P21_3B, PWM_2); // 设置PWM占空比
        pwm_set_duty(P21_5A, 0);    // 正转方向信号
    }
    else {
        pwm_set_duty(P21_3B, 0); // 设置PWM占空比
        pwm_set_duty(P21_5A, -PWM_2);    // 正转方向信号
    }

}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     电机转弯减速控制 PID
//  返回参数     Nope
//  使用示例     MotorTrunReduce();
//  备注信息     该 PID 计算式
//-------------------------------------------------------------------------------------------------------------------
void MotorTrunReduce(void)
{
    // 零时中间变量
    uint8_t temp_CenterLine;
    // 误差转换，使得交给 Error_M 值永远不小于 90
    if (CenterLine_Average < 90) temp_CenterLine = 180 - CenterLine_Average;
    else temp_CenterLine = CenterLine_Average;
    // PID计算,这里的 Kd 使用了转向环的 Kd
    TrunReduce.error = 90 - temp_CenterLine; // 计算当前误差
    TrunReduce.out = TrunReduce.error * Kp_MotorTrun
        + (TrunReduce.error - TrunReduce.error_last) * Servo_pid_t.Dir_Kd; // 计算PD输出
    TrunReduce.error_last = TrunReduce.error; // 保存上一次的误差

}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     PID计算函数
//  返回参数     *PiOut
//  使用示例     DEFINE_MOTOR_PI(L_Motorpi,true) & L_Motorpi
//  备注信息     true 为 左电机，false 为右电机
//-------------------------------------------------------------------------------------------------------------------
int MotorPi(int16* SetSpeed, int16* RealSpeed, float* Mkp, float* Mki, float* Merror, float* Merror1, int* PiOut, int16 PulseNow, bool isLeft)
{
    if (OutRoad_flag) Stop_flag = true;
    // 判断并设置目标速度
    if (isLeft) { 
        //                          ( 目标速度 + 转弯控速 * 缩小倍率 + （-控速速度）+/- 差速)
        *SetSpeed = Stop_flag ? 0 : (Set_Speed + TrunReduce.out * TurnReduceRace +
                                        Speed_Float - (Servo_t.Pwm_Servo - 785) * DifferentSpeedRace);
    }
    else {
        *SetSpeed = Stop_flag ? 0 : (Set_Speed + TrunReduce.out * TurnReduceRace +
                                        Speed_Float + (Servo_t.Pwm_Servo - 785) * DifferentSpeedRace);
    }
    // 获取实际速度
    *RealSpeed = PulseNow;
    // 更新误差
    *Merror1 = *Merror;               // 保存上一次误差
    *Merror = *SetSpeed - *RealSpeed; // 计算当前误差
    // PI控制器计算
    *PiOut += (*Merror - *Merror1) * *Mkp + *Merror * *Mki;

    // 限幅处理
    if (*PiOut > Max_Speed) {
        *PiOut = Max_Speed;
    }
    else if (*PiOut < -Max_Speed) {
        *PiOut = -Max_Speed;
    }

    return *PiOut;
}
// 判断函数(宏函数写法，不要在里面乱加注释)
#define DEFINE_MOTOR_PI(func_name, dir) \
int func_name(void) {                   \
    if (dir) {                          \
        return MotorPi(LEFT_ARGS, dir); \
    } else {                            \
        return MotorPi(RIGHT_ARGS, dir); \
    }                                   \
}
// 具体函数
DEFINE_MOTOR_PI(L_Motorpi,true)
DEFINE_MOTOR_PI(R_Motorpi, false)
