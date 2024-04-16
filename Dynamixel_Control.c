#include "Dynamixel_Control.h"
#include "Control_Dynamixel_Matrix.h"
#include "Dynamixel.h"
#include "motor_control.h"
#include "User_Usart.h"
#include "User_Delay.h"
#include "Dynamixel_Address.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "General_define.h"
#include "crc_ccitt.h"
#include "GO_Utree_Motor.h"
#include "serial_servo.h"
#include "serial_servo_porting.h"
#include "usbd_cdc_if.h"

//电机参数初始化

#define DeadBand 0.5f         //初始化死区

#define Periodic_Angle 360.00f	//周期角度

#define Frequency 3.0f   //旋转频率，单位Hz

#define Linear_Motor_Init_Angle 0.0f  //直线电机初始化角度

#define Kp 0.1f
#define Ki 0.0f
#define Kd 0.0f

//声明Dynamixel的Torque_Enable 数据位
#define Dynamixel_Torque_Enable 1
#define Dynamixel_Torque_Disable 0

//声明电机的数量
#define Dynamixel_Ben_Num 4


uint32_t xLastTime = 0;
uint32_t Start_Time = 0;

uint16_t Query_Angle = 0;  //查询位置

uint8_t Receive_Complete_state = 0;  //接收完成标志位 , 每一位代表一个电机 ， 当电机没有需要收到位置指令时是置1

enum Receive_Complate_State Receive_complete_State_2 = Not_Receive_XH540;	//新版接收完成标志位

//修改后第一版接收函数的接收完成标志位（现在主要用这个）
enum Receive_Complate_State Rece_Com_Statement_1 = Not_Receive_XH540;


//初始化笨笨的舵机角度参数（各个设定为基础的启动姿态）
float Ben_Angle1=90,Ben_Angle2=90,Ben_Angle3=90,Ben_Angle4=90;

extern uint8_t queue_trash[QU_LENGTH];  //队列丢弃数据

uint8_t Ben_iscontrol = 0;//笨笨的控制标志位，当此位置1时，控制舵机

//声明总的 ID_Group
uint8_t Dynamixel_ID_Group[Dynamixel_Num_XH540] = { 1 , 2 , 3 , 4 };


//声明电机结构体数组
Linear_Motor_Handler Linear_Motor_Handler_Group[Dynamixel_Num_XH540];



//声明Dynamixel的Drive_Mode状态枚举类型
//enum Drive_Mode_Statment Drive_Mode_Statment1 = Inital_Mode;

//声明角度 Group 
float Ben_Angle_Group[Dynamixel_Num_XH540];

/*-----------------------------------------------------*/
//RS485 的口的回调函数使用的状态数组
//XH540状态数组
//声明Present Position 使用的角度数组
int32_t Present_Pos_32t[4];
//声明Homing Offset使用的角度数组
int32_t Homing_Offset_Goup[4];
//声明 Profile Veolcity 的数值数组
int32_t Profile_Veolcity_Goup[4];
//声明 Torque_Enable 数组
uint8_t Torque_Enable_Goup[4];
//声明 is_Moving 数组
uint8_t is_Moving_Goup[4] = {0};
//AX-12状态数组

//Utree状态数组
MOTOR_recv Utree_M8010Rec_Group[1];

//声明幻尔舵机状态数组
int16_t HuangEr_Position_Group[1];

/*-------------------------------------------------------*/

//设计作用：和电机读参数的函数相配合 ， 当电机读参数指令发出后，就将其置0
//一般Dynamixel 是不返回角度的？

//增量PID参数
//初始化电机参数
// //电机I D//初始化状态/继续工作标志  //电机编码器最小单位输入   //电机原始角度，最小单位0.01deg ree//电机目标 角度//电机目标角度死区 上限//电机目标角度死区下限 //输出速度量//当前偏差量
//上一个偏差量//上上个偏差量//偏差累计量//PID参数//速度输出量//速度前馈量//Profile_Veolcity//Profile_Accleration
//保留四个，对应四个关节的电机
Linear_Motor_Handler Linear_Motor_1 = { 1 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f , 0.0f , 0.0f};
Linear_Motor_Handler Linear_Motor_2 = { 2 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f , 0.0f , 0.0f};
Linear_Motor_Handler Linear_Motor_3 = { 3 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f , 0.0f , 0.0f};
Linear_Motor_Handler Linear_Motor_4 = { 4 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f  , 0.0f , 0.0f};
																				
//初始化Dynamixel电机结构体数组
void Linear_Motor_Handler_Group_Init(void)
{
	Linear_Motor_Handler_Group[0] = Linear_Motor_1;
	Linear_Motor_Handler_Group[1] = Linear_Motor_2;
	Linear_Motor_Handler_Group[2] = Linear_Motor_3;
	Linear_Motor_Handler_Group[3] = Linear_Motor_4;
}
																		
																				
//初始化ID_Group
void Dynamixel_ID_Group_Init(void)
{
	//初始化总的ID_Group
	for(int i = 0 ; i < Dynamixel_Num_XH540 ; i++)
	{
		Dynamixel_ID_Group[i] = Linear_Motor_Handler_Group[i].ID;
	}
	
//	//调试部分
//	User_UART_STLINK_printf("ID_Group :%s \r\n ",Dynamixel_ID_Group);	
	
}


//初始化角度(四个Dynamixel的)
//传入结构体数组的地址
//在设置零点后执行
void Dynamixel_Angle_Group_Init(Linear_Motor_Handler* Motor_Group)
{
	Dynamixel_Read_Present_Position(Dynamixel_ID_Group , Dynamixel_Ben_Num);//读取所有的角度（当前角度）
	
	//设置角度
	//一号电机
	Motor_Group[0].Target_Angle = ((float)Present_Pos_32t[0])* Position_Map_num;
	//二号电机
	Motor_Group[1].Target_Angle = ((float)Present_Pos_32t[1])* Position_Map_num;
	//三号电机
	Motor_Group[2].Target_Angle = ((float)Present_Pos_32t[2])* Position_Map_num;
	//四号电机
	Motor_Group[3].Target_Angle = ((float)Present_Pos_32t[3])* Position_Map_num;
	
	//设置角度变量
	Ben_Angle1=90.f;
	
	Ben_Angle2=90.f;
	
	Ben_Angle3=90.f;
	
	Ben_Angle4=90.f;
//	
	//设置角度变量
	Ben_Angle_Group[0] = 90.f;
	Ben_Angle_Group[1] = 90.f;
	Ben_Angle_Group[2] = 90.f;
	Ben_Angle_Group[3] = 90.f;
	
}

/*-------------------------------------------*/
/*
	笨笨的Dynamixel 舵机的角度更新函数

	函数名：void Ben_Linear_Motor_Handler_Update_Angle()
	输入参数：需要的 结构体数组 和 电机数量 , 输入的角度数组
	作用：更新Dynamixel 结构体数组的角度值
	//修改增加调试部分
*/
/*-------------------------------------------*/												
void Ben_Linear_Motor_Handler_Update_Angle( Linear_Motor_Handler* Motor_Group , uint8_t ID_Group_Length , float* Angle_Group )
{
	//暂存ID数组
	uint8_t ID_Group_Motor_temp[ID_Group_Length];
	
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		ID_Group_Motor_temp[i] = Motor_Group[i].ID;
		//打印ID
		User_UART_STLINK_printf("UPANGLE Motor %d ID is %d \r\n" , i ,Motor_Group[i].ID);
	}
	
	Dynamixel_Read_Present_Position(ID_Group_Motor_temp , ID_Group_Length);//读取所有的角度（当前角度）
	//再依次写入更新角度
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		Motor_Group[i].Original_Angle = ((float)Present_Pos_32t[i]) * Position_Map_num;
		
	}
}



//增量式PID控制
void Angle_Incremental_PID (Linear_Motor_Handler *PID)
{
	/*计算目标值与实际值的误差*/
	PID->Err = PID->Target_Angle - PID->Original_Angle;

	/*PID算法实现*/
	PID->Speed_PID += PID->KP * ( PID->Err - PID->Err_Last )
								 + PID->KI * PID->Err
								 + PID->KD *(PID->Err - 2*PID->Err_Last + PID->Err_Last_2);

	/*传递误差*/
	PID->Err_Last_2 = PID->Err_Last;
	PID->Err_Last = PID->Err;
	
	/*实际输出转换*/
	PID->Speed_Output = PID->Speed_PID;
}



/*----------------------------------------------------------------------*/
/*
	RS485接收回调函数声明
*/
/*----------------------------------------------------------------------*/
void Present_Position_Receive_Func_XH540(void);
void Homing_Offset_Receive_Func_XH540(void);
void Profile_Veolcity_Receive_Func_XH540(void);
void Profile_Torque_Receive_Func_XH540(void);
void Moving_Receive_Func_XH540(void);
//幻尔舵机接收函数
void Position_Receive_Func_HuangEr(void);

//RS485（Dynamixel 口的回调函数）
//笨笨使用(还是使用的这个较多？)
//Size： 收到的数据长度
void Rx_RS485_func_Dynamixel_Ben(uint16_t Size ) //ISR中尽可能快，以标志位代替处理
{
	//XH540部分
	//Present Location_XH540
	if(Rece_Com_Statement_1 == Present_Position_Receive_XH540) //进入当前位置接收 ， 将解析收到的报文
	{
		Present_Position_Receive_Func_XH540();
	}
	
	//Homing Offset_XH540
	if(Rece_Com_Statement_1 == Homing_Offset_Receive_XH540) //进入当前位置接收 ， 将解析收到的报文
	{
		Homing_Offset_Receive_Func_XH540();
	}
	
	
	//Profile Veolcity_XH540
	if(Rece_Com_Statement_1 == Profile_Veolcity_Receive_XH540) //进入当前位置接收 ， 将解析收到的报文
	{
		Profile_Veolcity_Receive_Func_XH540();
	}
	
	
	//Torque_Enable_XH540
	if(Rece_Com_Statement_1 == Torque_Enable_Receive_XH540) //进入当前位置接收 ， 将解析收到的报文
	{
		Profile_Torque_Receive_Func_XH540();
	}
	
	//Moving_XH540
	if(Rece_Com_Statement_1 == Moving_Receive_XH540)//完成当前位置运动
	{
		Moving_Receive_Func_XH540();
	}
	
	//AX-12部分
	
	
	//Utree电机部分
	if(Rece_Com_Statement_1 == Receive_Data_Utree)
	{
		if(Size == 0)
		{
			User_UART_STLINK_printf(" HAL_Time_Out \r\n");
			return ;
		}
		
		if(Size != sizeof(Rx_RS485.rx_buff_temp))
		{
			User_UART_STLINK_printf(" HAL_ERROR \r\n");
			return;
		}
		
		uint8_t *rp = Rx_RS485.rx_buff_temp;//不行，这样需要取的是数组复制体
		
		
    if(rp[0] == 0xFE && rp[1] == 0xEE)//校验包头
    {
				//判断ID
				if(Utree_M8010Rec_Group[0].motor_id == (rp[2]&0xF))
				{
					
					//取出CRC码
					uint16_t CRC16_Utree = ((rp[14]&0xFF)<<8) + rp[15]&0xFF;
					//判断CRC
					if(CRC16_Utree != crc_ccitt_Utree(0, rp, 14))
					{
						User_UART_STLINK_printf(" Utree REC CRC error \r\n");
						
						return ;
					}			
					else
					{
						//解包
						Utree_M8010Rec_Group[0].motor_id = (rp[2]&0xF);
						Utree_M8010Rec_Group[0].mode = ((rp[2]>>4)&0xF);
						Utree_M8010Rec_Group[0].T = (float)(((rp[3]&0xFF)<<8) + ((rp[4]&0xFF))) / 256;//实际关节输出转矩,转化为力矩的话还要转换
						Utree_M8010Rec_Group[0].W = (float)(((rp[5]&0xFF)<<8) + ((rp[6]&0xFF))) / 256 * 2 * PI;//实际速度
						Utree_M8010Rec_Group[0].Pos = (float)(((rp[7]&0xFF)<<24) + ((rp[8]&0xFF)<<16) + ((rp[9]&0xFF)<<8) + ((rp[10]&0xFF))) / 32768 * 2 * PI;//实际关节位置
						Utree_M8010Rec_Group[0].Temp = rp[11];	//温度
						Utree_M8010Rec_Group[0].MError = rp[12]>>1;	//错误码
					}
					
				}
        
        return ;
    }
		else
		{
			User_UART_STLINK_printf(" Bag_Hander_notCorrect \r\n");
		}
		
	}
	
	//幻尔舵机部分
	//Present_Position_Receive_HuanEr
	if(Rece_Com_Statement_1 == Present_Position_Receive_HuanEr) //进入当前位置接收 ， 将解析收到的报文
	{
		//Profile_Veolcity_Receive_Func_XH540();
		Position_Receive_Func_HuangEr();
	}
	
}

/*--------------------------------------------------*/
/*
  函数作用: 将Dynamixel 当前位置设置为零点 ， 同步写入
	函数名：Dynamixel_Setting_Zero
	输入参数： 结构体数组 , 传入的ID数组的长度
	艹，这个函数由延迟上的问题！！！
*/
void Dynamixel_Setting_Zero( Linear_Motor_Handler* Linear_Motor_Handler_Group , uint8_t Linear_Motor_Handler_Group_Length)
{

	//设置ID数组
	uint8_t ID_Group[Linear_Motor_Handler_Group_Length];
	//homing_offset数组
	int32_t Homing_offset_buf[Linear_Motor_Handler_Group_Length];
	
	//设置Drive Mode
	enum Drive_Mode_Statment	Drive_Mode_Test_2 = Time_based_Profile_Mode;
	//User_UART_STLINK_printf("Drive_Mode_Test_2 : %d \r\n " , Drive_Mode_Test_2 );
	//置入ID
	//这边置入ID有问题？
	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		ID_Group[i] = Linear_Motor_Handler_Group[i].ID;
	}
	
	//将HomeOfffseting置零
	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Homing_offset_buf[i] = 0;
	  User_UART_STLINK_printf(" Homing Offset after %d is %d \r\n" , i , Homing_offset_buf[i]);
	}
	
	//失能
	Dynamixel_Set_Torque_Disable(ID_Group , Linear_Motor_Handler_Group_Length);
	
	//将home_offset 写入Dynamixel
	Dynamixel_Setting_Homing_Offset(Homing_offset_buf , ID_Group , Linear_Motor_Handler_Group_Length);
//	//读取Homing Offset
//	Homing_Offset_Goup[0] = 0;
//	Homing_Offset_Goup[1] = 0;
//	Dynamixel_Read_Homing_Offset(ID_Group , Linear_Motor_Handler_Group_Length);
	
	//读取Present Position
	//将位置接收数组置0
	for(int i = 0 ;  i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Present_Pos_32t[i] = 0;
	}
	Dynamixel_Read_Present_Position(ID_Group , Linear_Motor_Handler_Group_Length);

	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		User_UART_STLINK_printf("Dynamixel 1_%d Pos : %d \r\n " , i , Present_Pos_32t[i] );
	}
	//Homing 等于 负 Present Position
	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Homing_offset_buf[i] -= Present_Pos_32t[i];
		User_UART_STLINK_printf(" Homing Offset after2 %d is %d \r\n" , i , Homing_offset_buf[i]);
	}
	//写入Homing Offset
	Dynamixel_Setting_Homing_Offset(Homing_offset_buf , ID_Group , Linear_Motor_Handler_Group_Length);
	//读取Present Position
	//将位置接收数组置0 , 和 结构体的Origin角度置0
	for(int i = 0 ;  i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Linear_Motor_Handler_Group[i].Original_Angle = 0;
		Present_Pos_32t[i] = 0;
	}
	Dynamixel_Read_Present_Position(ID_Group , Linear_Motor_Handler_Group_Length);

	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		User_UART_STLINK_printf("Dynamixel 2_%d Pos : %d \r\n " , i , Present_Pos_32t[i] );
	}
}










/*--------------------------------------------------*/
/*
	函数作用：初始化 Dynamixel ， 同步写入 , 可以给每个舵机设置不同的Dirve Mode
	函数名：Dynamixel_Setting_Init
	输入参数： Drive_Mode , Operating_Mode  , ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Init(enum Drive_Mode_Statment Drive_Mode_St ,enum Operaing_Mode_Statment Operaing_Mode_St ,  uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	//设置Drive_Mode
//	//设置Drive_Mode状态枚举类型的值
//	//设置为Normal_Mode
//  //设置为TimeBase Mode
//	Drive_Mode_Statment1 |= Time_based_Profile;
	//设置Drive_Mode
	Dynamixel_Setting_Drive_Mode( Drive_Mode_St&0xF  ,  ID_Group ,  ID_Group_Length);
	
//设置Operating_Mode
	Dynamixel_Setting_Operating_Mode(Operaing_Mode_St&0xF ,  ID_Group ,  ID_Group_Length);	
	
}



/*--------------------------------------------------*/
/*
	函数作用：设置Profile Accleration 和 Profile Veolcity， 同步写入
	函数名：Dynamixel_Profile_Init
	输入参数： 设置的 Profile Accleration 和 Profile Veolcity 值数组, ID数组 , 传入的ID数组的长度 
	
*/
void Dynamixel_Profile_Init( int32_t* Profile_Accleration_Value ,int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
		
	//设置Profile_Accleration 通常将其设置为0？
	Dynamixel_Setting_Profile_Acceleration(Profile_Accleration_Value , ID_Group , ID_Group_Length);

	//设置Profile_Veolcity , 在timebase状态下它是到达目标点的时间
  Dynamixel_Setting_Profile_Veolcity(Profile_Veolcity_Value , ID_Group , ID_Group_Length);
	
	
}



/*--------------------------------------------------*/
/*
	函数作用：使能Dynamixel 的 Torque_Enable ， 同步读取
	函数名：Dynamixel_Setting_Torque_Enable
	输入参数： ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Set_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	//使能Toruqe_Enable
	Dynamixel_Setting_Torque_Enable( Dynamixel_Torque_Enable , ID_Group , ID_Group_Length);
	
}


void Dynamixel_Set_Torque_Disable( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	//使能Toruqe_Enable
	Dynamixel_Setting_Torque_Enable( Dynamixel_Torque_Disable , ID_Group , ID_Group_Length);
	
}




/*--------------------------------------------------*/
/*
	函数作用：设置Goal_Position， 同步写入
	函数名：Dynamixel_Setting_Goal_Position
	输入参数： 设置的 Goal_Position 值数组, ID数组 , 传入的ID数组的长度 , 传入结构体数组（需要修改其目标角度值）
	
*/
void Dynamixel_Set_Goal_Position( int32_t* Goal_Position_Value, uint8_t* ID_Group , uint8_t ID_Group_Length , Linear_Motor_Handler* Linear_Motor_Handler_Group)
{
	
		//int32_t Position_Int_Group[ID_Group_Length];// = (int32_t)(Position / Position_Map_num);
	
		//将目标角度置入结构体
		for(int i = 0  ; i < ID_Group_Length ; i++)
		{
			Linear_Motor_Handler_Group[i].Target_Angle = ((float)Goal_Position_Value[i]) * Position_Map_num; //将整数数值转换为小数角度输入结构体
			//Position_Int_Group[i] = (int32_t)(Goal_Position_Value[i] / Position_Map_num);
		}
			
		//设置Goal_Position
	 Dynamixel_Setting_Goal_Position(Goal_Position_Value , ID_Group , ID_Group_Length );

}




/*----------------------------------------------------------------------*/
/*
	RS485接收回调函数
*/
/*----------------------------------------------------------------------*/
void Present_Position_Receive_Func_XH540(void)
{
		//User_UART_STLINK_printf("IN Present_Position \r\n" );
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//检查接收到的ID是哪个电机的
			{
				Receive_Complete_state |= (1 << 0);		//将第0位置1，表示第一个电机收到信息
				//更新电机1的角度（之后四个同理）(记得在数组中更新)	
				Present_Pos_32t[0] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				Linear_Motor_Handler_Group[0].Original_Angle = (float)Present_Pos_32t[0] * Position_Map_num;
//				//打印电机的角度 不要在这里打印，应该在接收函数结束后打印
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//向右移动一位，即01 变为 10
				Present_Pos_32t[1] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				Linear_Motor_Handler_Group[1].Original_Angle = (float)Present_Pos_32t[1] * Position_Map_num;
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Present_Pos_32t[2] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				Linear_Motor_Handler_Group[2].Original_Angle = (float)Present_Pos_32t[2] * Position_Map_num;
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Present_Pos_32t[3] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				Linear_Motor_Handler_Group[3].Original_Angle = (float)Present_Pos_32t[3] * Position_Map_num;
//				//打印电机的角度
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			//Rece_Com_Statement_1 = Not_Receive;	
}

void Homing_Offset_Receive_Func_XH540(void)
{
		//	User_UART_STLINK_printf("IN Homing Offset \r\n" );
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//检查接收到的ID是哪个电机的
			{
				Receive_Complete_state |= (1 << 0);		//将第0位置1，表示第一个电机收到信息
				//更新电机1的角度（之后四个同理）(记得在数组中更新)	
				Homing_Offset_Goup[0] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				//打印电机的角度 不要在这里打印，应该在接收函数结束后打印
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//向右移动一位，即01 变为 10
				Homing_Offset_Goup[1] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Homing_Offset_Goup[2] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Homing_Offset_Goup[3] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				//打印电机的角度
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}

void Profile_Veolcity_Receive_Func_XH540(void)
{
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//检查接收到的ID是哪个电机的
			{
				Receive_Complete_state |= (1 << 0);		//将第0位置1，表示第一个电机收到信息
				//更新电机1的角度（之后四个同理）(记得在数组中更新)	
				Profile_Veolcity_Goup[0] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				//打印电机的角度 不要在这里打印，应该在接收函数结束后打印
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//向右移动一位，即01 变为 10
				Profile_Veolcity_Goup[1] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Profile_Veolcity_Goup[2] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Profile_Veolcity_Goup[3] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				//打印电机的角度
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}

void Profile_Torque_Receive_Func_XH540(void)
{
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//检查接收到的ID是哪个电机的
			{
				Receive_Complete_state |= (1 << 0);		//将第0位置1，表示第一个电机收到信息
				//更新电机1的角度（之后四个同理）(记得在数组中更新)	
				Torque_Enable_Goup[0] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				//打印电机的角度 不要在这里打印，应该在接收函数结束后打印
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//向右移动一位，即01 变为 10
				Torque_Enable_Goup[1] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Torque_Enable_Goup[2] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Torque_Enable_Goup[3] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				//打印电机的角度
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}

//使用循环控制Moving
void Moving_Receive_Func_XH540(void)
{
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//检查接收到的ID是哪个电机的
			{
				Receive_Complete_state |= (1 << 0);		//将第0位置1，表示第一个电机收到信息
				//更新电机1的角度（之后四个同理）(记得在数组中更新)	
				is_Moving_Goup[0] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				//打印电机的角度 不要在这里打印，应该在接收函数结束后打印
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//向右移动一位，即01 变为 10
				Torque_Enable_Goup[1] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				is_Moving_Goup[2] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				is_Moving_Goup[3] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				//打印电机的角度
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}



//幻尔舵机接收函数
void Position_Receive_Func_HuangEr(void)
{			
	if(Rx_RS485.rx_buff_temp[0] != 0x55 || Rx_RS485.rx_buff_temp[1] != 0x55)//检查帧头是否正确
	{
		//User_UART_STLINK_printf("HuanEr Hander Error \r\n " );
		CDC_Transmit_FS("HuanEr Hander Error \r\n",23);
	}
	
	
	if(Rx_RS485.rx_buff_temp[3] != 0x07)//检查数据长度
	{
		//User_UART_STLINK_printf("HuanEr DataLength Error \r\n " );
		CDC_Transmit_FS("HuanEr DataLength Error \r\n ",28);
	}
	
	//检查收到的ID
	switch(Rx_RS485.rx_buff_temp[2])
	{
		case 1:
			HuangEr_Position_Group[0] = (int16_t)(((int16_t)Rx_RS485.rx_buff_temp[5]) + ((int16_t)Rx_RS485.rx_buff_temp[6] << 8) );//角度低八位 和 角度高八位
			break;
		default:
			//User_UART_STLINK_printf("HuanEr DataLength Error \r\n " );
		  CDC_Transmit_FS("HuanEr DataID Error \r\n ",24);
			break;
	}
	

}





