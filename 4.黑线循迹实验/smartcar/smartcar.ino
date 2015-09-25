/********************************* 深圳市航太电子有限公司 *******************************
* 实 验 名 ：小车红外黑线循迹
* 实验说明 ：小车自动沿黑线行驶
* 实验平台 ：流星7号、arduino UNO R3 
* 连接方式 ：请参考interface.h文件
* 注    意 ：因地面和轮子的差异，左右转向时需要根据实际需要调节差速的差值
* 作    者 ：航太电子产品研发部    QQ ：1909197536
* 店    铺 ：http://shop120013844.taobao.com/
****************************************************************************************/

#include <MsTimer2.h> 
#include "interface.h"
#include <IRremote.h>
#include <Timer.h>

//变量定义
int timer_count;
unsigned int speed_count;//占空比计数器 50次一周期
char front_left_speed_duty;
char front_right_speed_duty;
char behind_left_speed_duty;
char behind_right_speed_duty;
unsigned char tick_5ms = 0;//5ms计数器，作为主函数的基本周期
unsigned char tick_1ms = 0;//1ms计数器，作为电机的基本计数器

char ctrl_comm = COMM_STOP;//控制指令
char ctrl_comm_last = COMM_STOP;
unsigned char continue_time = 0;
//IRrecv irrecv(IRIN);
Timer t;
decode_results results;
char ir_rec_flag;
char bt_rec_flag;

//函数声明
void CarMove();
void CarGo();
void CarBack();
void CarLeft();
void CarRight();
void CarStop();

//根据占空比驱动电机转动
void CarMove()
{
	//左前轮
	if(front_left_speed_duty > 0)//向前
	{
		if(speed_count < front_left_speed_duty)
		{
			FRONT_LEFT_GO;
		}else
		{
			FRONT_LEFT_STOP;
		}
	}
	else if(front_left_speed_duty < 0)//向后
	{
		if(speed_count < (-1)*front_left_speed_duty)
		{
			FRONT_LEFT_BACK;
		}else
		{
			FRONT_LEFT_STOP;
		}
	}
	else                //停止
	{
		FRONT_LEFT_STOP;
	}
	
	//右前轮
	if(front_right_speed_duty > 0)//向前
	{
		if(speed_count < front_right_speed_duty)
		{
			FRONT_RIGHT_GO;
		}else                //停止
		{
			FRONT_RIGHT_STOP;
		}
	}
	else if(front_right_speed_duty < 0)//向后
	{
		if(speed_count < (-1)*front_right_speed_duty)
		{
			FRONT_RIGHT_BACK;
		}else                //停止
		{
			FRONT_RIGHT_STOP;
		}
	}
	else                //停止
	{
		FRONT_RIGHT_STOP;
	}
	
	//左后轮
	if(behind_left_speed_duty > 0)//向前
	{
		if(speed_count < behind_left_speed_duty)
		{
			BEHIND_LEFT_GO;
		}	else                //停止
		{
			BEHIND_LEFT_STOP;
		}
	}
	else if(behind_left_speed_duty < 0)//向后
	{
		if(speed_count < (-1)*behind_left_speed_duty)
		{
			BEHIND_LEFT_BACK;
		}	else                //停止
		{
			BEHIND_LEFT_STOP;
		}
	}
	else                //停止
	{
		BEHIND_LEFT_STOP;
	}
	
	//右后轮
	if(behind_right_speed_duty > 0)//向前
	{
		if(speed_count < behind_right_speed_duty)
		{
			BEHIND_RIGHT_GO;
		}	else                //停止
		{
			BEHIND_RIGHT_STOP;
		}
	}
	else if(behind_right_speed_duty < 0)//向后
	{
		if(speed_count < (-1)*behind_right_speed_duty)
		{
			BEHIND_RIGHT_BACK;
		}	else                //停止
		{
			BEHIND_RIGHT_STOP;
		}
	}
	else                //停止
	{
		BEHIND_RIGHT_STOP;
	}
}

//向前
void CarGo()
{
	front_left_speed_duty=SPEED_DUTY;
	front_right_speed_duty=SPEED_DUTY;
	behind_left_speed_duty=SPEED_DUTY;
	behind_right_speed_duty=SPEED_DUTY;
}

//后退
void CarBack()
{
	front_left_speed_duty=-SPEED_DUTY;
	front_right_speed_duty=-SPEED_DUTY;
	behind_left_speed_duty=-SPEED_DUTY;
	behind_right_speed_duty=-SPEED_DUTY;
}

//向左
void CarLeft()
{
	front_left_speed_duty=-20;
	front_right_speed_duty=SPEED_DUTY;
	behind_left_speed_duty=-20;
	behind_right_speed_duty=SPEED_DUTY+10;//增加后轮驱动力
}

//向右
void CarRight()
{
	front_left_speed_duty=SPEED_DUTY;
	front_right_speed_duty=-20;
	behind_left_speed_duty=SPEED_DUTY+10;//增加后轮驱动力
	behind_right_speed_duty=-20;
}

//停止
void CarStop()
{
	front_left_speed_duty=0;
	front_right_speed_duty=0;
	behind_left_speed_duty=0;
	behind_right_speed_duty=0;
}

void IOInit()
{
	pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);
	pinMode(FRONT_LEFT_F_IO, OUTPUT);
	pinMode(FRONT_LEFT_B_IO, OUTPUT);
	pinMode(FRONT_RIGHT_F_IO, OUTPUT);
	pinMode(FRONT_RIGHT_B_IO, OUTPUT);
	pinMode(BEHIND_LEFT_F_IO, OUTPUT);
	pinMode(BEHIND_LEFT_B_IO, OUTPUT);
	pinMode(BEHIND_RIGHT_F_IO, OUTPUT);
	pinMode(BEHIND_RIGHT_B_IO, OUTPUT);
	
	pinMode(SEARCH_M_IO, INPUT);
	pinMode(SEARCH_R_IO, INPUT);
	pinMode(SEARCH_L_IO, INPUT);
	
	pinMode(DUOJI_IO, OUTPUT);
	
	pinMode(FRONT_RIGHT_S_IO, INPUT);
	pinMode(FRONT_LEFT_S_IO, INPUT);

}

//循迹，通过判断三个光电对管的状态来控制小车运动
void SearchRun()
{
	//三路都检测到
	if(SEARCH_M_BIT == BLACK_AREA && SEARCH_L_BIT == BLACK_AREA && SEARCH_R_BIT == BLACK_AREA)
	{
		ctrl_comm = COMM_UP;
		return;
	}
	
	if(SEARCH_R_BIT == BLACK_AREA)//右
	{
		ctrl_comm = COMM_RIGHT;
	}
	else if(SEARCH_L_BIT == BLACK_AREA)//左
	{
		ctrl_comm = COMM_LEFT;
	}
	else if(SEARCH_M_BIT == BLACK_AREA)//中
	{
		ctrl_comm = COMM_UP;
	}
}

//中断处理函数，改变灯的状态
void flash()  
{                        
	tick_5ms++;
	speed_count++;
	if(speed_count >= 50)//50ms周期 //modfied by LC 2015.09.12 9:56
	{
           //     Serial.println(speed_count,HEX);
		speed_count = 0;
                
	}
	CarMove();
}

void setup() {

	IOInit();
	// 初始化串口参数
	Serial.begin(9600);
	CarStop();
	//irrecv.enableIRIn(); // Start the receiver
    t.every(1,flash); //T = 1ms
}

void loop() {  
	//指令接收部分
	//红外遥控部分    
    t.update();

	//执行部分
	if(tick_5ms >= 5)
	{
		tick_5ms = 0;
        SearchRun();
		//do something
		if(ctrl_comm_last != ctrl_comm)//接收到红外信号
		{
			ctrl_comm_last = ctrl_comm;
			switch(ctrl_comm)
			{
			case COMM_UP:    CarGo();break;
			case COMM_DOWN:  CarBack();break;
			case COMM_LEFT:  CarLeft();break;
			case COMM_RIGHT: CarRight();break;
			case COMM_STOP:  CarStop();break;
				default : break;
			}
            Serial.println(ctrl_comm,HEX);
		}
	}             
}
