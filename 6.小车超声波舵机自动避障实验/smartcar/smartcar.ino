/********************************* 深圳市航太电子有限公司 *******************************
* 实 验 名 ：超声波舵机自动避障实验
* 实验说明 ：将超声波模块放在舵机旋转轴上，通过转动舵机，来获取前方，左边以及右边的障碍物距离
* 实验平台 ：流星7号、arduino UNO R3 
* 连接方式 ：请参考interface.h文件
* 注    意 ：舵机旋转角度建议不要使用0°和180°，可以稍微往中间一点，比如选取20°和160 °，这样舵机旋转更精确
* 作    者 ：航太电子产品研发部    QQ ：1909197536
* 店    铺 ：http://shop120013844.taobao.com/
****************************************************************************************/

#include <MsTimer2.h> 
#include "interface.h"
#include <IRremote.h>
#include <Timer.h>
#include <Servo.h> 

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
unsigned char distance_cm = 0;
//IRrecv irrecv(IRIN);
Timer t;
Servo myservo;
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

void Delayms(unsigned int count)
{
	int x;
	for(x=0;x<count;x++)
	{
		delay(1);
		t.update();
	}	
}

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

void GetDistance()
{
        long IntervalTime=0; //定义一个时间变量
		digitalWrite(Trig, 1);//置高电平
		delayMicroseconds(15);//延时15us
		digitalWrite(Trig, 0);//设为低电平
		IntervalTime=pulseIn(Echo, HIGH);//用自带的函数采样反馈的高电平的宽度，单位us
		float S=IntervalTime/58.00; //使用浮点计算出距离，单位cm
		if(S != 0)//去掉采集不成功的情况
		{
			distance_cm = (unsigned char)S;
		}	
}

//获取超声波距离
void Distance()
{
	static unsigned char count = 0;
	count++;
	if(count >= 20)//每100ms 采集一次
	{
		GetDistance();
	}
}

void DuojiMid()
{
	myservo.write(90);              // 输入对应的角度值，舵机会转到此位置
	Delayms(150);//延时1s
}

void DuojiRight()
{
	myservo.write(20);              // 输入对应的角度值，舵机会转到此位置
	Delayms(150);//延时1s
}

void DuojiLeft()
{
	myservo.write(160);              // 输入对应的角度值，舵机会转到此位置
	Delayms(150);//延时1s
}

///获取三个方向的距离,进来前舵机方向为向前
void GetAllDistance(unsigned int *dis_left,unsigned int *dis_right,unsigned int *dis_direct)
{
	CarStop();
	GetDistance();
	*dis_direct = distance_cm;
	
	DuojiRight();
	Delayms(100);
	GetDistance();//获取右边距离
	*dis_right = distance_cm;
	
	DuojiMid();
	DuojiLeft();
	Delayms(100);
	GetDistance();//获取左边距离
	*dis_left = distance_cm;
	
	DuojiMid();//归位
}

void BarrierProc()
{
		if(distance_cm < 10)//前方有障碍物
	   {
		unsigned int dis_left;//左边距离
		unsigned int dis_right;//右边距离
		unsigned int dis_direct;//右边距离
		if(distance_cm < 8)
		{
			CarBack();
			Delayms(400);
		}
		
		while(1)
		{
			GetAllDistance(&dis_left,&dis_right,&dis_direct);
			if(dis_direct < 5)
			{
				CarBack();
				Delayms(300);
				continue;
			}
			else if((dis_left < 5) || (dis_right < 5))
			{
				CarBack();
				Delayms(300);
				continue;
			}
			else if(dis_direct >= dis_left && dis_direct >= dis_right)//前方距离最远
			{
				CarGo();
				Delayms(600);
				return;
			}
			else if(dis_left <= dis_right)//右转
			{
				CarRight();
				Delayms(500);
			}
			else if(dis_right < dis_left)
			{
				CarLeft();
				Delayms(500);
			}
		}
	}
	else
	{
		CarGo();
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
	myservo.attach(DUOJI_IO);  // 舵机控制信号引脚   
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
		Distance();
        BarrierProc();
	}             
}
