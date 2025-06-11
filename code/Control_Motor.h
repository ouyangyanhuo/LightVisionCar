#ifndef __CONTROL_MOTOR_H__
#define __CONTROL_MOTOR_H__

//==================================================== 左右电机参数定义 ====================================================
#define LEFT_ARGS &LeftPID.L_SetSpeed, &LeftPID.L_RealSpeed, &LeftPID.L_Mkp, &LeftPID.L_Mki, &LeftPID.L_Merror, &LeftPID.L_Merror1, &LeftPID.L_PiOut, Left_Pulse_now
#define RIGHT_ARGS &RightPID.R_SetSpeed, &RightPID.R_RealSpeed, &RightPID.R_Mkp, &RightPID.R_Mki, &RightPID.R_Merror, &RightPID.R_Merror1, &RightPID.R_PiOut, Right_Pulse_now
//==================================================== 左右电机参数定义 ====================================================

//==================================================== 函数声明 ====================================================
void Motor_Init(void);	
void MotorControl(int PWM_1, int PWM_2);
void MotorTrunReduce(void);
int MotorPi(int16* SetSpeed, int16* RealSpeed, float* Mkp, float* Mki, float* Merror, float* Merror1, int* PiOut, int16 PulseNow, bool isLeft);
int L_Motorpi(void);
int R_Motorpi(void);
//==================================================== 函数声明 ====================================================

#endif