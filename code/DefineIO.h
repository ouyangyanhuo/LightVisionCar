#ifndef _MotorControl_h_
#define _MotorControl_h_

//==================================================== 公共引脚定义 ====================================================
// 适用于 PWM 设置，详见 pwm_init
#define P21_4A ATOM0_CH2_P21_4	// L
#define P21_2B ATOM1_CH0_P21_2	// L
#define P21_5A ATOM0_CH3_P21_5	// R
#define P21_3B ATOM1_CH1_P21_3	// R

// 0C类使用方式 适用于 编码器，详见 zf_driver_encoder
#define P10_3_0C TIM5_ENCODER_CH1_P10_3
#define	P20_3_0C TIM6_ENCODER_CH1_P20_3

// 1C类使用方式 适用于 编码器，详见 zf_driver_encoder
#define P10_1_1C TIM5_ENCODER_CH2_P10_1
#define	P20_0_1C TIM6_ENCODER_CH2_P20_0

// D类使用方式，适用于 舵机
#define P33_9 ATOM0_CH1_P33_9

//==================================================== 公共引脚定义 ====================================================

//==================================================== 常量定义 ====================================================
#define WIFI_SSID			"OOIT"				// WIFI名称
#define WIFI_PASSWORD		"ooit1234"			// WIFI密码
#define SPI_MODE			2					// SPI模式，1 向上位机传图，2 向上位机传编码器数据，3 接收上位机指令，4 收发数据
#define RUN_Mode			1					// 运行模式，0 独立，1 跟随
#define CarStop				1					// 是否开启停车，0 关闭，1 开启
#define Find_Mode			0                   // 寻灯模式， 0 横寻， 1 竖寻
#define DELAY_BUFFER_SIZE	1					// 转弯缓冲变量，推荐 1-5，在目前运行状况下建议为 1

#define Max_Speed           7000				// 电机最大速度限幅
#define Set_Speed			119					// 目标速度
#define TurnReduceRace		0.5f				// 转向减速缩小倍率

#define Kp_Speed			0.3f				// 控速环 Kp
#define Max_Control			50					// 最大加减速速度
#define Kp_MotorTrun		0.2f				// 后轮差速 Kp
#define DifferentSpeedRace	0.5f				// 后轮差速缩小倍率

#define Servo_TurnKp		1.6f				// 转向环 偏离过大时 Kp
#define Servo_NomalKp		1.0f				// 转向环 正常时 Kp
#define Servo_NomalKd		0.0f				// 转向环、控距环、转向减速环 kd
//==================================================== 常量定义 ====================================================

//==================================================== 初始化函数 ====================================================
void InitElement_CPU0_OutWhile(void);
void InitElement_CPU1_OutWhile(void);
void InitElement_isr_ch0(void);
void InitElement_isr_ch1(void);
//==================================================== 初始化函数 ====================================================

//==================================================== 公共变量声明 ====================================================
extern bool Stop_flag,OutRoad_flag, WIFI_SPI_Con;
extern int16 Left_Pulse_now, Right_Pulse_now;
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];
extern int Speed_Float;
extern int Pwm1, Pwm2;
//==================================================== 公共变量声明 ====================================================
#endif
