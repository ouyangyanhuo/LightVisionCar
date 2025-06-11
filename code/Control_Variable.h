#ifndef _Control_Variable_h_
#define _Control_Variable_h_

//==================================================== 参数结构体 ====================================================
// 左电机参数结构体
typedef struct {
    int16 L_SetSpeed;    // 目标速度
    int16 L_RealSpeed;   // 实际速度
    float L_Mkp;         // 比例系数
    float L_Mki;         // 积分系数
    float L_Merror;      // 当前误差
    float L_Merror1;     // 上一次误差
    int L_PiOut;         // PI控制器输出
} leftpid;
extern leftpid LeftPID;
// 右电机参数结构体
typedef struct {
    int16 R_SetSpeed;    // 目标速度
    int16 R_RealSpeed;   // 实际速度
    float R_Mkp;         // 比例系数
    float R_Mki;         // 积分系数
    float R_Merror;      // 当前误差
    float R_Merror1;     // 上一次误差
    int R_PiOut;         // PI控制器输出
} rightpid;
extern rightpid RightPID;
// 速度环转向减速
typedef struct {
    short error;
    short error_last;
    short out;
}skperror;
extern skperror TrunReduce;
// 控速环 PID（不进行全局实例化）
typedef struct {
    float Kp;            // 比例系数
    float Kd;            // 微分系数
    float last_error;    // 上一次误差
} PID_Controller;
//==================================================== 参数结构体 ====================================================
// 舵机 PID 参数
typedef struct {
    float Dir_Kp;
    float Dir_Kd;
}SERVO_PID_PARAMETERS;
extern SERVO_PID_PARAMETERS Servo_pid_t;
// 方向环输出结构体
typedef struct {
    uint16 Out_Direction;			// 舵机方向环输出
    uint16 Pwm_Servo;				// 舵机输出
}SERVO_CONTROL;
extern SERVO_CONTROL Servo_t;

extern uint16 Servo_Duty;
#endif