#ifndef __DYNAMIXEL_CONTROL_H__
#define __DYNAMIXEL_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

//直线电机控制相关参数结构体
typedef struct
{
	uint8_t ID;                 //电机ID

  volatile uint8_t State;      //初始化状态/继续工作标志

	int32_t Target_Position_Orig;//电机输入的实数值，相当于电机按照最小分度值运动
	float Original_Angle;        //电机原始角度，最小单位0.01degree
  float Target_Angle;          //电机目标角度
  float Target_Angle_Upper;    //电机目标角度死区上限
  float Target_Angle_Lower;    //电机目标角度死区下限

	float Speed_PID;             //输出速度量
	float Err;                   //当前偏差量
	float Err_Last;              //上一个偏差量
	float Err_Last_2;            //上上个偏差量
  float Err_Sum;               //偏差累计量

	float KP,KI,KD;              //PID参数
	
	float Speed_Output;       //速度输出量
  float Speed_Feedforward;   //速度前馈量
	
	float Liner_Motor_Profile_Veolcity;			//速度Profile值
	float Liner_Motor_Profile_Accleration;	//加速度Profile值
	
} Linear_Motor_Handler;


//声明Drive_Mode 的 枚举变量
enum Drive_Mode_Statment{
    Inital_Mode = 0x00,
    Reverse_Mode = 0x01,
    Slave_Mode = 0x02,
    Time_based_Profile_Mode = 0x04
};

//声明 Operating_Mode 的枚举变量
enum Operaing_Mode_Statment{

	Current_Control_Mode = 0x00,
	
	Velocity_Control_Mode = 0x01,
	
	Position_Control_Mode = 0x03,	//默认模式
	
	Extended_Position_Control_Mode = 0x04,	//多圈模式
	
	Current_based_Position_Control_Mode = 0x05

};


//声明Dynamixel Receive_Complate_State 的 枚举变量
enum Receive_Complate_State{
	Not_Receive_XH540 = 0x00,
	
	Present_Position_Receive_XH540 = 0x01,		

	Drive_Mode_Receive_XH540 = 0x02,
	
	Operating_Mode_Receive_XH540 = 0x03,
	
	Homing_Offset_Receive_XH540 = 0x04,
	
	Torque_Enable_Receive_XH540 = 0x05,
	
	Profile_Accleration_Receive_XH540 = 0x06,
	
	Profile_Veolcity_Receive_XH540 = 0x07,
	
	Receive_Data_Utree = 0x08,
	
	Moving_Receive_XH540 = 0x09,
	
	Present_Position_Receive_HuanEr = 0x0A,//幻尔总线舵机接收位
};


//声明Dynamixel Receive_Complate_State 的 枚举变量
//宇树电机
enum Receive_Complate_State_Utree{
	Not_Receive_Utree = 0x00,
	
	Present_Position_Receive_Utree = 0x01,		

	Drive_Mode_Receive_Utree = 0x02,
	
	Operating_Mode_Receive_Utree = 0x03,
	
	Homing_Offset_Receive_Utree = 0x04,
	
	Torque_Enable_Receive_Utree = 0x05,
	
	Profile_Accleration_Receive_Utree = 0x06,
	
	Profile_Veolcity_Receive_Utree = 0x07,
	
};



#pragma pack(1)
struct PacketRawFrame {
    uint8_t start_byte1;
    uint8_t start_byte2;
    uint8_t function;
    uint8_t data_length;
    uint8_t data_and_checksum[257];
};
#pragma pack()



//初始化电机结构体数组
void Linear_Motor_Handler_Group_Init(void);
																
																				
//初始化ID_Group
void Dynamixel_ID_Group_Init(void);

//初始化角度(四个Dynamixel的)
//传入结构体数组的地址
void Dynamixel_Angle_Group_Init(Linear_Motor_Handler* Motor_Group);




/*-------------------------------------------*/
/*
	笨笨的Dynamixel 舵机的角度更新函数

	函数名：void Ben_Linear_Motor_Handler_Update_Angle()
	输入参数：需要的 结构体数组 和 电机数量 , 输入的角度数组
	作用：更新Dynamixel 结构体数组的角度值

*/
/*-------------------------------------------*/												
void Ben_Linear_Motor_Handler_Update_Angle( Linear_Motor_Handler* Motor_Group , uint8_t ID_Group_Length , float* Angle_Group );




/*--------------------------------------------------*/
/*
  函数作用: 将Dynamixel 当前位置设置为零点 ， 同步写入
	函数名：Dynamixel_Setting_Zero
	输入参数： 结构体数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Zero( Linear_Motor_Handler* Linear_Motor_Handler_Group , uint8_t Linear_Motor_Handler_Group_Length);




/*--------------------------------------------------*/
/*
	函数作用：初始化 Dynamixel ， 同步写入
	函数名：Dynamixel_Setting_Init
	输入参数： Drive_Mode , Operating_Mode  , ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Init(enum Drive_Mode_Statment Drive_Mode_St ,enum Operaing_Mode_Statment Operaing_Mode_St ,  uint8_t* ID_Group , uint8_t ID_Group_Length);


/*--------------------------------------------------*/
/*
	函数作用：设置Profile Accleration 和 Profile Veolcity， 同步写入
	函数名：Dynamixel_Profile_Init
	输入参数： 设置的 Profile Accleration 和 Profile Veolcity 值数组, ID数组 , 传入的ID数组的长度 
	
*/
void Dynamixel_Profile_Init( int32_t* Profile_Accleration_Value ,int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);




/*--------------------------------------------------*/
/*
	函数作用：设置Goal_Position， 同步写入
	函数名：Dynamixel_Setting_Goal_Position
	输入参数： 设置的 Goal_Position 值数组, ID数组 , 传入的ID数组的长度 , 传入结构体数组（需要修改其目标角度值）
	
*/
void Dynamixel_Set_Goal_Position( int32_t* Goal_Position_Value, uint8_t* ID_Group , uint8_t ID_Group_Length , Linear_Motor_Handler* Linear_Motor_Handler_Group);


/*--------------------------------------------------*/
/*
	函数作用：使能Dynamixel 的 Torque_Enable ， 同步读取
	函数名：Dynamixel_Setting_Torque_Enable
	输入参数： ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Set_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length );



//RS485（Dynamixel 口的回调函数）
//笨笨使用
void Rx_RS485_func_Dynamixel_Ben(uint16_t Size);

void Dynamixel_Set_Torque_Disable( uint8_t* ID_Group , uint8_t ID_Group_Length );


//RS485（Dynamixel 口的回调函数）
//笨笨使用
//Size： 收到的数据长度
//返回 Motor_Group 的 指针
Linear_Motor_Handler* Rx_RS485_func_Dynamixel_Ben_2(uint16_t Size , Linear_Motor_Handler* Motor_Group); //ISR中尽可能快，以标志位代替处理

////电机状态初始化
//void Motor_Control_Restore(void);

////电机控制任务
//void Motor_Control_Task(uint8_t State_num);

////电机控制任务测试
//void Motor_Control_Task_Test(void);

////使能电机并设置电机的模式为拓展位置模式
////设置的目标电机为1-4号电机
//void Ben_Dynamixel_Extended_PosMode_Enable(void);


////设置多个电机的角度（伪）
////实质为通过输入数个角度值并发送相同数量的指令
////笨笨专用
////输入值分别为四条腿的角度
////会调整对应电机结构体的目标角度
//void Ben_Dynamixel_Extended_PosControl(float Angle1, float Angle2, float Angle3, float Angle4);

#ifdef __cplusplus
}
#endif
#endif /*__DYNAMIXEL_CONTROL_H__ */
