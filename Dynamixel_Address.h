#ifndef __DYNAMIXEL_ADDRESS_H__
#define __DYNAMIXEL_ADDRESS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
		这个文件是用来定义 Dynamixel的宏定义的
*/
	
//电机数量
//XH540部分
#define Dynamixel_Num_XH540 4 
	
//定义电机起始角度
	
	
//目标位置寄存器起始地址
#define Goal_Position_XH540 0x0074
	
//当前位置寄存器起始地址
#define Present_Position_XH540 0x0084
	
//Drive_Mode寄存器起始地址
#define Drive_Mode_XH540 0x000A
	
//Operating_Mode 寄存器起始地址
#define Operating_Mode_XH540 0x000B
	
//Torque_Enable 寄存器起始地址
#define Torque_Enable_XH540 0x0040
	
//homing_offset 寄存器起始地址
#define homing_offset_XH540 0x0014

//Profile_Accleration 寄存器起始地址
#define Profile_Accleration_XH540 0x006C

//Profile_Veolcity 寄存器起始地址
#define Profile_Veolcity_XH540 0x0070

//Moving_Threshold 寄存器起始地址
#define Moving_Threshold_XH540 0x0018

//Moving 寄存器起始地址
#define Moving_XH540 0x007A

//AX-12

//电机数量
#define Dynamixel_Num_AX_12 1

//目标位置寄存器起始地址
#define Goal_Position_AX_12 0x001E
	
//当前位置寄存器起始地址
#define Present_Position_AX_12 0x0024
	
//当前位置最小限制寄存器
#define Minimum_Position_AX_12 0x0006
	
//当前位置最大限制寄存器
#define Maximum_Position_AX_12 0x0008
	
//Torque_Enable 寄存器起始地址
#define Torque_Enable_AX_12 0x0018

//Moving_Speed寄存器起始地址
#define Moving_Speed_AX_12 0x0020

//Present_Speed寄存器起始地址
#define Present_Speed_AX_12 0x0026




#ifdef __cplusplus
}
#endif
#endif /* __DYNAMIXEL_H__ */
