/*
 *
 *  Name: Image_Variable.c
 *  Created on: 2025/2/24
 *  Author: Magneto
 *
 */

#include "zf_common_headfile.h"
//==================================================== ͼ����� ====================================================
// ����ƹ���ֵ
uint8_t Edge_Range = 255;
// ���Ƽ�¼
int LostLight_cnt = 0;
// DrawEdge
int DrawTop, DrawTop2, DrawBottom, DrawBottom2, DrawRight, DrawRight2, DrawLeft, DrawLeft2;
// ƽ������
uint8_t CenterLine_Average = 0;
//==================================================== ͼ����� ====================================================