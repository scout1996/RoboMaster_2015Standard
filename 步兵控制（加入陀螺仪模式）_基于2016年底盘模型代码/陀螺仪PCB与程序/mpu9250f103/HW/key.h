#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

#define KEY1  PAin(15)
#define KEY2  PBin(3)
#define KEY3  PBin(5)
#define KEY4  PBin(4)
#define KEY5  PBin(9)
#define KEY6  PBin(8)

#define KEY1_PRES 	1	//KEY0按下
#define KEY2_PRES	2
#define KEY3_PRES	3
#define KEY4_PRES	4
#define KEY5_PRES	5
#define KEY6_PRES	6


void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8);  	//按键扫描函数
#endif
