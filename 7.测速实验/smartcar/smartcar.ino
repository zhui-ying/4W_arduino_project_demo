/********************************* 深圳市航太电子有限公司 *******************************
* 实 验 名 ：小车红外遥控测速实验
* 实验说明 ：使用红外遥控来控制小车运动,通过串口读取左轮和右轮的速度
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
unsigned char tick_200ms = 0;

char ctrl_comm;//控制指令
unsigned char continue_time = 0;
IRrecv irrecv(IRIN);
Timer t;
decode_results results;
char ir_rec_flag;

//函数声明
void CarMove();
void CarGo();
void CarBack();
void CarLeft();
void CarRight();
void CarStop();

/*电机控制部分*/
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

/*速度测试部分*/
//轮子直径66mm，光电码盘齿数为20，轮子周长 207mm = 20.7cm 
//程序采用判断高低电平变化次数计数，也就是说轮子转一周计数次数为40
//一个计数变化表示轮子跑过的距离为 20.7/40 = 0.5175cm

unsigned char front_left_speed=0;
unsigned char front_right_speed=0;

/*******************************************************************************
* 函 数 名 ：MeasureSpeed
* 函数功能 ：速度测量，计算IO变化次数，该函数必须每5ms调用一次
* 输    入 ：无
* 输    出 ：无
*******************************************************************************/
void MeasureSpeed()
{
	static unsigned char front_left_speed_temp=0;
	static unsigned char front_right_speed_temp=0;
	static unsigned char front_left_io=0;
	static unsigned char front_right_io=0;
	static unsigned char count_5ms=0;
	count_5ms++;
	if(FRONT_LEFT_S_BIT != front_left_io)//发生电平变化
	{
		front_left_speed_temp++;
		front_left_io = FRONT_LEFT_S_BIT;
	}
	
	if(FRONT_RIGHT_S_BIT != front_right_io)//发生电平变化
	{
		front_right_speed_temp++;
		front_right_io = FRONT_RIGHT_S_BIT;
	}
	
	if(count_5ms == 100)//每500ms获取一次速度
	{
		count_5ms = 0;
		front_left_speed = front_left_speed_temp *2;//获取1s的高低电平变化次数
		front_right_speed = front_right_speed_temp*2;
		front_left_speed_temp = 0;
		front_right_speed_temp = 0;
		
		front_left_speed = (unsigned char)(0.5175 * (double)front_left_speed + 0.5);//计算速度 cm/s 四舍五入
		front_right_speed = (unsigned char)(0.5175 * (double)front_right_speed + 0.5);//计算速度 cm/s 四舍五入
	}
}

/*初始化部分*/
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
	//MsTimer2::set(1, flash);        // 中断设置函数，每 1ms 进入一次中断
	//MsTimer2::start();                //开始计时
       t.every(1,flash);
}

void loop() {  
	//指令接收部分
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
         t.update();

	//执行部分
	if(tick_5ms >= 5)
	{
		tick_5ms = 0;
		tick_200ms++;
		if(tick_200ms >= 40)
		{
			tick_200ms = 0;
			Serial.print("FL:");
			Serial.print(front_left_speed);
			Serial.print("  FR:");
			Serial.print(front_right_speed);
			Serial.print("\r\n");
		}
		continue_time--;//200ms 无接收指令就停车
		if(continue_time == 0)
		{
			continue_time = 1;
			CarStop();
		}
		//do something
		MeasureSpeed();
		if(ir_rec_flag == 1)//接收到红外信号
		{
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
