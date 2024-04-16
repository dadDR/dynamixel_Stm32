#ifndef __DYNAMIXEL_H__
#define __DYNAMIXEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "Dynamixel_Control.h"
//定义Drive Mode 参数
/*-------------------*/
//Normal/Reverse Mode
#define Normal_Mode	 0
#define Reverse_Mode 1
	
//Profile的参数
#define Velocity_based_Profile 0
#define Time_based_Profile 1
/*--------------------*/	
	
	
//Torque On by Goal Update
//默认为力矩Enable后再接收指令 

#define Enable 0x01
#define Disable 0x00
//0x00 ： 对应电流模式  0x01：对应速度模式 0x03：对应位置模式 0x04：对应拓展位置模式 0x05：基于电流的位置模式 0x10 PWM控制模式
#define Electric_Mode 0x00		 //电流模式
#define Speed_Mode 0x01        //速度模式
#define Position_Mode 0x03     //位置模式
#define Extended_Position_Mode 0x04		//拓展位置模式 (经测试 ， 可用)
#define Electric_Position_Mode 0x05		//基于电流的位置模式
#define PWM_Control_Mode			 0x10		//PWM控制模式

#define Position_Map_num 0.08789f //位置映射常数  ; 意义：将电机的位置值从角度值的输入转换为0-4095的输入值输入，即调用函数时只需要写入浮点数类型的角度值
#define Accleration_Map_num //加速度映射常数 , 即将	

	
#define Speed_Map_num 0.22897f * 6.0f  //速度映射常数



/*--------------------------------------------------*/
/*
	函数作用：设置 Dynamxiel 的 Drive_Mode 寄存器 同步写入
	函数名：Dynamixel_Setting_Drive_Mode
	输入参数： 寄存器设置位(uint8_t) , ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Drive_Mode(uint8_t Drive_Mode_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length);




/*--------------------------------------------------*/
/*
	函数作用：读取 DriMode Mode 寄存器 同步读取
	函数名：Dynamixel_Reading_Drive_Mode()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	读取 DriMode Mode 寄存器的数据 返回读取数据的字节长度

*/
uint8_t Dynamixel_Reading_Drive_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length );



/*--------------------------------------------------*/
/*
	函数作用：设置 Dynamxiel 的 Operating_Mode 寄存器 同步写入
	函数名：Dynamixel_Setting_Operating_Mode
	输入参数： 寄存器设置位(uint8_t) , ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Operating_Mode(uint8_t Operating_Mode_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：读取 Operating Mode 寄存器 同步读取
	函数名：Dynamixel_Reading_Operating_Mode()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	读取 Operating Mode 寄存器的数据 返回读取数据的字节长度

*/
uint8_t Dynamixel_Reading_Operating_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length );



/*--------------------------------------------------*/
/*
	函数作用：设置 Dynamxiel 的 Torque_Enable 寄存器 同步写入
	函数名：Dynamixel_Setting_Torque_Enable
	输入参数： 寄存器设置位(uint8_t), ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Torque_Enable(uint8_t Torque_Enable_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：读取 Torque_Enable 寄存器 同步读取
	函数名：Dynamixel_Reading_Torque_Enable()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	读取 Torque_Enable 寄存器的数据 返回读取数据的字节长度

*/
uint8_t Dynamixel_Reading_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length );




/*--------------------------------------------------*/
/*
	函数作用：设置 homing offset 寄存器 同步写入
	函数名：Dynamixel_Setting_Homing_Offset()
	输入参数： float Homing_Offset 需要写入的电机ID数组，传入的数组长度
	是否需要加入调试信息打印

*/
void Dynamixel_Setting_Homing_Offset(int32_t* Homing_Offset_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 homing offset 寄存器 同步读取
	函数名：Dynamixel_Read_Homing_Offset()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Homing_Offset(uint8_t* ID_Group , uint8_t ID_Group_Length);




/*--------------------------------------------------*/
/*
	函数作用：设置 Profile Acceleration 寄存器 同步写入
	函数名：Dynamixel_Setting_Profile_Acceleration()
	输入参数： Profile Acceleration值数组， 需要写入的电机ID数组，传入的数组长度
	是否需要加入调试信息打印 , 加速度一般全部写0？

*/
void Dynamixel_Setting_Profile_Acceleration(int32_t* Profile_Acceleration_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Profile Acceleration 寄存器 同步读取
	函数名：Dynamixel_Read_Profile_Acceleration()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Profile_Acceleration(uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：设置 Profile Veolcity 寄存器 同步写入
	函数名：Dynamixel_Setting_Profile_Veolcity()
	输入参数： Profile Acceleration值数组， 需要写入的电机ID数组，传入的数组长度
	在TimeBase的情况下，它定义的是什么时候到达目标位置

*/
void Dynamixel_Setting_Profile_Veolcity(int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Profile Veolcity 寄存器 同步读取
	函数名：Dynamixel_Read_Profile_Veolcity()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Profile_Veolcity(uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：设置 Goal_Position 寄存器 同步写入
	函数名：Dynamixel_Setting_Goal_Position()
	输入参数：Goal_Position值数组， 需要写入的电机ID数组，传入的数组长度
	在TimeBase的情况下，它定义的是什么时候到达目标位置

*/
void Dynamixel_Setting_Goal_Position(int32_t* Goal_Position_Value , uint8_t* ID_Group , uint8_t ID_Group_Length );



/*--------------------------------------------------*/
/*
	函数作用：设置 Goal_Position 寄存器 单独写入 
	函数名：Dynamixel_Setting_Goal_Position()
	输入参数：Goal_Position值数组， 需要写入的电机ID数组，传入的数组长度
	在TimeBase的情况下，它定义的是什么时候到达目标位置

*/
void Dynamixel_Setting_Goal_Position_Inidvail(int32_t Goal_Position_Value , uint8_t ID);



/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Present_Position 寄存器 同步写入
	函数名：Dynamixel_Read_Present_Position ()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	是否需要加入调试信息打印 , 返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Present_Position(uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Moving 寄存器 同步读取(XH540)
	函数名：Dynamixel_Read_Moving()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	是否需要加入调试信息打印 , 返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Moving(uint8_t* ID_Group , uint8_t ID_Group_Length);





////同步读角度请求,不定长参数接收,第一个参数为电机数量,后面为ID_num个电机ID
//uint8_t Sync_Read_Motor_Angle(uint16_t ID_num, ...);

////读角度请求
//uint8_t Read_Motor_Angle(uint8_t Motor_ID);

////电机扭矩启动
//void Sync_Set_Motor_Torque_Enable(uint8_t State, uint16_t ID_num, ...);

////电机操作模式切换
//void Sync_Set_Motor_Operating_Mode(uint8_t Mode, uint16_t ID_num, ...);

////电机速度控制
//void Set_Motor_Speed(uint8_t Motor_ID, float Speed);

////电机角度控制
//void Set_Motor_Position(uint8_t Motor_ID, float Position);

////电机同步写入角度测试(待测试)
////转动到同一个角度，只输入一个角度
////同时在多个电机的角度寄存器中写入角度
//void Sync_Set_Motor_Position(float Position , uint16_t ID_num, ...);

////提前设置写入位置（待测试）
////写入后，需要执行Action指令才能驱动
//void Reg_Set_Motor_Position(float Position , uint8_t Motor_ID);

////执行提前写入设置的代码（待测试）
//void Reg_Action(uint8_t Motor_ID);

///*
//	函数名：Dynamixel_Write_OBytes (待测试)
//	函数作用：向Dynamixel的寄存器中写入二 Byte 的数据 , 不使用 Sync模式
//	输入参数：起始地址 Address ， 写入的二位数据的值(二位 直接用float截断字节吧)
//*/
//void Dynamixel_Write_OBytes( uint8_t Motor_ID, uint16_t Address , float Value);


///*
//	函数名：Dynamixel_Write_TBytes (待测试)
//	函数作用：向Dynamixel的寄存器中写入二 Byte 的数据 , 不使用 Sync模式
//	输入参数：起始地址 Address ， 写入的二位数据的值(二位 直接用float截断字节吧)
//*/
//void Dynamixel_Write_TBytes( uint8_t Motor_ID, uint16_t Address , float Value);


///*
//	函数名：Dynamixel_Write_FBytes (待测试)
//	函数作用：向Dynamixel的寄存器中写入四 Byte 的数据 , 不使用 Sync模式
//	输入参数：起始地址 Address ， 写入的四位数据的值(四位 一般是float?)
//*/
//void Dynamixel_Write_FBytes( uint8_t Motor_ID, uint16_t Address , float Value);


///*--------------------------------------------------------*/
///*
//	函数名：Drive_Mode_Write_Sync
//	函数作用：设置 Drive Mode 的 寄存器值 , 包括 Profile Configuration ， 主从机（这个暂时不设置） , 正向和反向模式
//  输入值：  Profile Configuration , 正向和方向写入
//*/
//void Drive_Mode_Write_Sync(uint8_t Profile_Configuration , uint8_t Direction , uint16_t ID_num, ...);


#ifdef __cplusplus
}
#endif
#endif /* __DYNAMIXEL_H__ */
