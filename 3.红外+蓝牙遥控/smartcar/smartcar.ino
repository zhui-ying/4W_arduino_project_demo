/********************************* 深圳市航太电子有限公司 *******************************
* 实 验 名 ：小车红外+蓝牙遥控实验
* 实验说明 ：同时支持红外遥控以及手机蓝牙遥控小车
* 实验平台 ：流星7号、arduino UNO R3 
* 连接方式 ：请参考interface.h文件
* 注    意 ：指令必须连续发送才会使小车动作，若停止发送指令，则小车会停止
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

char ctrl_comm;//控制指令
unsigned char continue_time = 0;
IRrecv irrecv(IRIN);
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
	irrecv.enableIRIn(); // Start the receiver
    t.every(1,flash); //T = 1ms
}

void loop() {  
	//指令接收部分
	//红外遥控部分
 	if (irrecv.decode(&results)) 
	{
		unsigned int value_temp;
		continue_time = 40;
		value_temp = (results.value & 0xffff);//取后16位
		if(value_temp != 0xffff)
		{
      		if((value_temp & 0xff00)>>8 == ( ~(value_temp & 0xff) & 0xff))//校验
      		{
      			ctrl_comm = (value_temp & 0xff00)>>8 & 0xff;
      			ir_rec_flag = 1;
      			switch(ctrl_comm)//指令转换
				{
					case 0x02: ctrl_comm = COMM_STOP; break;
					case 0x62: ctrl_comm = COMM_UP; break;
					case 0xA8: ctrl_comm = COMM_DOWN; break;
					case 0xC2: ctrl_comm = COMM_RIGHT; break;
					case 0x22: ctrl_comm = COMM_LEFT; break;
					default: break;
				}
      		}
        }
		irrecv.resume(); // Receive the next value
	}	 
	
	//蓝牙遥控部分
	  // 读取串口发送的信息:
    if (Serial.available() > 0)
	{
		char inChar;
		inChar = Serial.read();
	    if(ctrl_comm != inChar || continue_time == 1)
		{
			bt_rec_flag = 1;
			ctrl_comm = inChar;
		}
                continue_time = 40;
    }       
    t.update();

	//执行部分
	if(tick_5ms >= 5)
	{
		tick_5ms = 0;
		continue_time--;//200ms 无接收指令就停车
		if(continue_time == 0)
		{
			continue_time = 1;
			CarStop();
		}
		//do something
	if(bt_rec_flag == 1 || ir_rec_flag == 1)//接收到红外信号
		{
			bt_rec_flag = 0;
			ir_rec_flag = 0;
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
