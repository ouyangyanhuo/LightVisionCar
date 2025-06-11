/*
 *
 *  Name: Image_Variable.c
 *  Created on: 2025/2/24
 *  Author: Magneto
 *
 */

#include "zf_common_headfile.h"
//==================================================== 图像变量 ====================================================
// 红外灯光阈值
uint8_t Edge_Range = 255;
// 丢灯记录
int LostLight_cnt = 0;
// DrawEdge
int DrawTop, DrawTop2, DrawBottom, DrawBottom2, DrawRight, DrawRight2, DrawLeft, DrawLeft2;
// 平均中线
uint8_t CenterLine_Average = 0;
//==================================================== 图像变量 ====================================================