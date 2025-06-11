/*
 *
 *  Name: Define.c
 *  Created on: 2025/2/23
 *  Author: Magneto
 *
 */
#include "zf_common_headfile.h"

 //==================================================== 公共变量 ====================================================
 // 停车标志位，默认为 0 (false 未停车)
bool Stop_flag = false;
// 出赛道标志位，默认为 0 (false 未出界)
bool OutRoad_flag = false;
// 开启WIFI连接，默认为 1 (true 开启)
bool WIFI_SPI_Con = false;
// 左右电机脉冲数据-实际读数
int16 Left_Pulse_now = 0, Right_Pulse_now = 0;
// 左右PWM接收
int Pwm1 = 0, Pwm2 = 0;
// 复制的图像数据（用于WiFi传输）
uint8 image_copy[MT9V03X_H][MT9V03X_W];
// 控速环 接口
int Speed_Float = 0;
//==================================================== 公共变量 ====================================================

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     CPU0所有运行在 While 外的组件初始化函数
//  返回参数     Nope
//  使用示例     InitElement_CPU0_OutWhile();
//  备注信息     Nope
//-------------------------------------------------------------------------------------------------------------------
void InitElement_CPU0_OutWhile(void)
{
	// 电机初始化
	Motor_Init();
	// 镜头初始化
	mt9v03x_init();
	// 编码器初始化
	encoder_dir_init(TIM5_ENCODER, P10_3_0C, P10_1_1C);
	encoder_dir_init(TIM6_ENCODER, P20_3_0C, P20_0_1C);
	// 舵机初始化
	pwm_init(P33_9, 50, Servo_Duty);
	// 卡尔曼滤波参数初始化
	Kalman_Init();
	// SPI 无线通信处理
	if (true == WIFI_SPI_Con) WIFIConnect();
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     中断系统所有运行在 ch0 的组件初始化函数
//  返回参数     Nope
//  使用示例     InitElement_isr_ch0();
//  备注信息     Nope
//-------------------------------------------------------------------------------------------------------------------

void InitElement_isr_ch0(void)
{
	Servo_Init();                         //放在此处是为了动态pid
	// 左右编码器获取脉冲数据
	Left_encoder();
	Right_encoder();
	// 舵机 PID 控制
	Serorpd();
	// 电机 PID 控制
	MotorTrunReduce();
	// 电机控制
	//	MotorControl(1600, 1600); //电机调试
	Pwm1 = L_Motorpi();
	Pwm2 = R_Motorpi();
	MotorControl(Pwm1, Pwm2);
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     CPU1所有运行在 While 外的组件初始化函数
//  返回参数     Nope
//  使用示例     InitElement_CPU1_OutWhile();
//  备注信息     Nope
//-------------------------------------------------------------------------------------------------------------------
void InitElement_CPU1_OutWhile(void)
{
	// SPI 无线通信初始化
	if (true == WIFI_SPI_Con) seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     中断系统所有运行在 ch1 的组件初始化函数
//  返回参数     Nope
//  使用示例     InitElement_isr_ch1();
//  备注信息     Nope
//-------------------------------------------------------------------------------------------------------------------
int Show_Cnt=0;
void InitElement_isr_ch1(void)
{
	if (true == WIFI_SPI_Con) DataHandleInt();
	if (mt9v03x_finish_flag) {
		tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
		mt9v03x_finish_flag = 0;
		if (true == WIFI_SPI_Con) DataHandle();
	}
	ImageProcess();

	// 屏幕数据显示
	// EncoderData();

//	Show_Cnt++;
//	if(Show_Cnt >=80){
//		tft180_show_int(0, 120, Pwm1, 4);
//		tft180_show_int(50, 120, Pwm2, 4);
//		tft180_show_int(0, 60, Left_Pulse_now, 3);
//		tft180_show_int(50, 60, Right_Pulse_now, 3);
//		Show_Cnt = 0;
//	}
//
//	DrawEdge(DrawTop, DrawTop2, DrawBottom, DrawBottom2, DrawLeft, DrawLeft2, DrawRight, DrawRight2);
//	tft180_show_float(0, 140, Servo_t.Pwm_Servo, 3, 1);
//  tft180_show_float(60, 140, CenterLine_Average, 3, 1);
}
