#ifndef _Image_Variable_h_
#define _Image_Variable_h_

//==================================================== 图像参数定义 ====================================================
#define IMAGE_W         (MT9V03X_W)
#define IMAGE_H         (MT9V03X_H)
//==================================================== 图像参数定义 ====================================================
//==================================================== 图像变量声明 ====================================================
extern uint8_t CenterLine_Average;
extern uint8_t BLACK_num;
extern uint8_t Edge_Range;
extern int LostLight_cnt;
extern int DrawTop, DrawTop2, DrawBottom, DrawBottom2, DrawRight, DrawRight2, DrawLeft, DrawLeft2;
#endif
