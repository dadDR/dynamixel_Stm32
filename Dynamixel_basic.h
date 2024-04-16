#ifndef __DYNAMIXEL_BASIC_H__
#define __DYNAMIXEL_BASIC_H__

#include "main.h"
	
#define Delay_Time_ms 10     //总线延时时间，用于补偿

	
	
/*---------------------------------*/
//基本的写入数据 包括同步写入 一 二 三部分数据 和 非同步写入一 二 三部分数据
/*
	Dynamxiel 的写寄存区指令，作用为向指定寄存器的地址写入数据
*/


/*-------------------------------------------------*/
/*
	非Sync
*/
/*-------------------------------------------------*/


/*
	函数名：Dynamixel_Write_OBytes (待测试)
	函数作用：向Dynamixel的寄存器中写入一 Byte 的数据 , 不使用 Sync模式
	输入参数：起始地址 Address ， 写入的1位数据的值(1位 直接用float截断字节吧)
*/
void Dynamixel_Write_OBytes( uint8_t Motor_ID, uint16_t Address , uint8_t Value);


/*
	函数名：Dynamixel_Write_TBytes (待测试)
	函数作用：向Dynamixel的寄存器中写入二 Byte 的数据 , 不使用 Sync模式
	输入参数：起始地址 Address ， 写入的二位数据的值(二位 直接用float截断字节吧)
*/
void Dynamixel_Write_TBytes( uint8_t Motor_ID, uint16_t Address , uint16_t Value);


/*
	函数名：Dynamixel_Write_FBytes (待测试)
	函数作用：向Dynamixel的寄存器中写入四 Byte 的数据 , 不使用 Sync模式
	输入参数：起始地址 Address ， 写入的四位数据的值(四位 一般是float?)
*/
void Dynamixel_Write_FBytes( uint8_t Motor_ID, uint16_t Address , int32_t Value);


/*----------------------------------------------------------------*/
/*
	Sync
*/
/*--------------------------------------------------------------*/



/*
	函数名：Dynamixel_Write_Sync_OBytes (待测试)
	函数作用：向Dynamixel的寄存器中写入一 Byte 的数据 , 使用 Sync模式
	输入参数：起始地址 Address ， 写入的一位数据的值(是一个float数组)   ,  ID的数量 ,
*/
void Dynamixel_Write_Sync_OBytes( uint16_t Address ,uint8_t ID_num ,uint8_t* Value_Group  , uint8_t* ID_Group );


/*
	函数名：Dynamixel_Write_Sync_TBytes (待测试)
	函数作用：向Dynamixel的寄存器中写入二 Byte 的数据 , 使用 Sync模式
	输入参数：起始地址 Address ， 写入的二位数据的值(是一个float数组)   ,  ID的数量 ,
*/
void Dynamixel_Write_Sync_TBytes( uint16_t Address ,uint8_t ID_num ,uint16_t * Value_Group  , uint8_t* ID_Group );


/*
	函数名：Dynamixel_Write_Sync_FBytes (待测试)
	函数作用：向Dynamixel的寄存器中写入四 Byte 的数据 , 使用 Sync模式
	输入参数：起始地址 Address ， 写入的四位数据的值(是一个float数组)   , ID的数量 ,
*/
void Dynamixel_Write_Sync_FBytes( uint16_t Address ,uint8_t ID_num ,int32_t* Value  , uint8_t* ID_Group );


/*---------------------------------*/
//基本的读取数据 包括同步读取 一 二 四 Byte 数据 和 非同步读取 一 二 四 Byte 数据
/*
	Dynamxiel 的写寄存区指令，作用为向指定寄存器的地址写入数据
*/


/*-------------------------------------------------*/
/*
	非Sync
*/
/*-------------------------------------------------*/













/*-------------------------------------------------*/
/*
	Sync
*/
/*-------------------------------------------------*/


/*--------------------------------------------*/
/*
	函数名称 ： Dynamixel_Read_FuncFByte
	函数作用 ： 同步读取寄存器的数据 , 数据将作为RS485数据发送回来
	函数输入值：输入电机的数量，并同时输入电机的ID值 , 输入需要读取的寄存器地址 , 输入需要读取的数据长度(读取多长的数据) , 需要读取的电机ID号数组
	待测试
*/  
void Dynamixel_Sync_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group );




/*--------------------------------------------*/
/*
	函数名称 ： Dynamixel_Sync_Fast_Read(经过测试，例程可用)
	函数作用 ： 同步读取寄存器的数据 , 数据将作为RS485数据发送回来
	函数输入值：输入电机的数量，并同时输入电机的ID值 , 输入需要读取的寄存器地址 , 输入需要读取的数据长度(读取多长的数据) , 需要读取的电机ID号数组
	待测试
*/  
void Dynamixel_Sync_Fast_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group );




/*-------------------------------------------------*/
/*
	测试程序
*/
/*-------------------------------------------------*/



/*---------------------------------*/
//数据读取函数和数据写入函数的同时测试
//采用Dynamixel 官网上的教程

void Dynamixel_Basic_Func_Pose(void);






#endif /* __DYNAMIXEL_H__ */
