#ifndef __INTERFACE_H_
#define __INTERFACE_H_

#include <Arduino.h>
//IO 接口定义

//蓝牙
//int Rx = 0;
//int Tx= 1;

//红外遥控
int IRIN = 11;

//超声波模块
int Echo = 9;
int Trig = 8;

//电机驱动
int FRONT_LEFT_F_IO =  12; //左前前进IO
int FRONT_LEFT_B_IO =  13; //左前后退IO

int FRONT_RIGHT_F_IO =  19; //右前前进IO
int FRONT_RIGHT_B_IO =  18; //右前后退IO

int BEHIND_LEFT_F_IO =  15; //左后前进IO
int BEHIND_LEFT_B_IO =  14; //左后后退IO

int BEHIND_RIGHT_F_IO =  16; //右后前进IO
int BEHIND_RIGHT_B_IO =  17; //右后后退IO

//红外循迹
int SEARCH_M_IO = 4;//中间循迹IO
int SEARCH_R_IO = 3;//右循迹IO
int SEARCH_L_IO = 2;//左循迹IO
#define BLACK_AREA 1
#define WHITE_AREA 0

#define SEARCH_M_BIT digitalRead(SEARCH_M_IO)
#define SEARCH_R_BIT digitalRead(SEARCH_R_IO)
#define SEARCH_L_BIT digitalRead(SEARCH_L_IO)

//舵机IO
int DUOJI_IO = 10;//开发板默认是P0^7,这里需要转到其他IO口上

//速度码盘反馈
int FRONT_RIGHT_S_IO =  5; //右前前进IO
int FRONT_LEFT_S_IO =  6; //左前前进IO
#define FRONT_RIGHT_S_BIT digitalRead(FRONT_RIGHT_S_IO)
#define FRONT_LEFT_S_BIT digitalRead(FRONT_LEFT_S_IO)

//电机运动控制

//左前
#define FRONT_LEFT_GO    digitalWrite(FRONT_LEFT_F_IO , 1) ; digitalWrite(FRONT_LEFT_B_IO , 0)//前进
#define FRONT_LEFT_BACK  digitalWrite(FRONT_LEFT_F_IO , 0); digitalWrite(FRONT_LEFT_B_IO , 1)//后退
#define FRONT_LEFT_STOP  digitalWrite(FRONT_LEFT_F_IO , 0); digitalWrite(FRONT_LEFT_B_IO , 0)//停止

//右前
#define FRONT_RIGHT_GO     digitalWrite(FRONT_RIGHT_F_IO , 1);digitalWrite(FRONT_RIGHT_B_IO , 0)
#define FRONT_RIGHT_BACK   digitalWrite(FRONT_RIGHT_F_IO , 0);digitalWrite(FRONT_RIGHT_B_IO , 1)
#define FRONT_RIGHT_STOP   digitalWrite(FRONT_RIGHT_F_IO , 0);digitalWrite(FRONT_RIGHT_B_IO , 0)

//左后
#define BEHIND_LEFT_GO     digitalWrite(BEHIND_LEFT_F_IO , 1);digitalWrite(BEHIND_LEFT_B_IO , 0)
#define BEHIND_LEFT_BACK   digitalWrite(BEHIND_LEFT_F_IO , 0);digitalWrite(BEHIND_LEFT_B_IO , 1)
#define BEHIND_LEFT_STOP   digitalWrite(BEHIND_LEFT_F_IO , 0);digitalWrite(BEHIND_LEFT_B_IO , 0)

//右后
#define BEHIND_RIGHT_GO    digitalWrite(BEHIND_RIGHT_F_IO , 1);digitalWrite(BEHIND_RIGHT_B_IO , 0)
#define BEHIND_RIGHT_BACK  digitalWrite(BEHIND_RIGHT_F_IO , 0);digitalWrite(BEHIND_RIGHT_B_IO , 1)
#define BEHIND_RIGHT_STOP  digitalWrite(BEHIND_RIGHT_F_IO , 0);digitalWrite(BEHIND_RIGHT_B_IO , 0)

#define SPEED_DUTY 30//默认占空比 按1ms最小分辨率 周期50ms计算 30 /50  60%

//指令定义
//#define COMM_STOP  0x02//停止
//#define COMM_UP    0x62//前进
//#define COMM_DOWN  0xA8//后退
//#define COMM_RIGHT 0xC2//右转
//#define COMM_LEFT  0x22//左转

#define COMM_STOP  'I'//停止
#define COMM_UP    'A'//前进
#define COMM_DOWN  'B'//后退
#define COMM_RIGHT 'D'//右转
#define COMM_LEFT  'C'//左转

#endif

