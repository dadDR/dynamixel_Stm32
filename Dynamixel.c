#include "Dynamixel.h"
#include "Dynamixel_Address.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "User_Delay.h"
#include "User_Usart.h"
#include "Dynamixel_basic.h"

//声明总的ID_Group
extern uint8_t Dynamixel_ID_Group[Dynamixel_Num_XH540];
extern int32_t Present_Pos_32t[4];
//声明接收标志位
//(Dynamixel的)
//修改后第一版接收函数的接收完成标志位（现在主要用这个）
extern enum Receive_Complate_State Rece_Com_Statement_1;

/*--------------------------------------------------*/
/*
	函数作用：设置 Dynamxiel 的 Drive_Mode 寄存器 同步写入
	函数名：Dynamixel_Setting_Drive_Mode
	输入参数： 寄存器设置位(uint8_t) , ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Drive_Mode(uint8_t Drive_Mode_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length)
{

	uint8_t temp_Value[ID_Group_Length];
	
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		temp_Value[i] = Drive_Mode_Value;
	}
	
	//Dynamixel_Write_Sync_OBytes( uint16_t Address ,uint8_t ID_num ,uint8_t* Value_Group  , uint8_t* ID_Group )
	Dynamixel_Write_Sync_OBytes( Drive_Mode_XH540 ,ID_Group_Length ,temp_Value  , ID_Group );
	
}



/*--------------------------------------------------*/
/*
	函数作用：读取 DriMode Mode 寄存器 同步读取
	函数名：Dynamixel_Reading_Drive_Mode()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	读取 DriMode Mode 寄存器的数据 返回读取数据的字节长度

*/
uint8_t Dynamixel_Reading_Drive_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	
	//Dynamixel_Sync_Fast_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group )
	Dynamixel_Sync_Read(Drive_Mode_XH540, 1 , ID_Group_Length , ID_Group );
	return 1;
}




/*--------------------------------------------------*/
/*
	函数作用：设置 Dynamxiel 的 Operating_Mode 寄存器 同步写入
	函数名：Dynamixel_Setting_Operating_Mode
	输入参数： 寄存器设置位(uint8_t) , ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Operating_Mode(uint8_t Operating_Mode_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	uint8_t temp_Value[ID_Group_Length];
	
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		temp_Value[i] = Operating_Mode_Value;
	}
	
	Dynamixel_Write_Sync_OBytes( Operating_Mode_XH540 ,ID_Group_Length ,temp_Value  , ID_Group );
	
}


/*--------------------------------------------------*/
/*
	函数作用：读取 Operating Mode 寄存器 同步读取
	函数名：Dynamixel_Reading_Operating_Mode()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	读取 Operating Mode 寄存器的数据 返回读取数据的字节长度

*/
uint8_t Dynamixel_Reading_Operating_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	//Dynamixel_Sync_Fast_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group )
	Dynamixel_Sync_Read( Operating_Mode_XH540, 1 , ID_Group_Length , ID_Group );
	return 1;
}



/*--------------------------------------------------*/
/*
	函数作用：设置 Dynamxiel 的 Torque_Enable 寄存器 同步写入
	函数名：Dynamixel_Setting_Torque_Enable
	输入参数： 寄存器设置位(uint8_t), ID数组 , 传入的ID数组的长度
	
*/
void Dynamixel_Setting_Torque_Enable(uint8_t Torque_Enable_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	uint8_t temp_Value[ID_Group_Length];
	
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		temp_Value[i] = Torque_Enable_Value;
	}
	
	Dynamixel_Write_Sync_OBytes( Torque_Enable_XH540 ,ID_Group_Length ,temp_Value  , ID_Group );
	
}


/*--------------------------------------------------*/
/*
	函数作用：读取 Torque_Enable 寄存器 同步读取
	函数名：Dynamixel_Reading_Torque_Enable()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	读取 Torque_Enable 寄存器的数据 返回读取数据的字节长度

*/
uint8_t Dynamixel_Reading_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	Rece_Com_Statement_1 = Torque_Enable_Receive_XH540;
	Dynamixel_Sync_Read( Torque_Enable_XH540 , 1 , ID_Group_Length , ID_Group );
	return 1;
}




/*--------------------------------------------------*/
/*
	函数作用：设置 homing offset 寄存器 同步写入
	函数名：Dynamixel_Setting_Homing_Offset()
	输入参数： float Homing_Offset 需要写入的电机ID数组，传入的数组长度
	是否需要加入调试信息打印

*/
void Dynamixel_Setting_Homing_Offset(int32_t* Homing_Offset_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	Dynamixel_Write_Sync_FBytes( homing_offset_XH540 , ID_Group_Length  ,Homing_Offset_Value  ,  ID_Group );

}


/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 homing offset 寄存器 同步读取
	函数名：Dynamixel_Read_Homing_Offset()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Homing_Offset(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Homing_Offset_Receive_XH540; //先将接收标志位置位
	Dynamixel_Sync_Read( homing_offset_XH540 , 4 , ID_Group_Length ,  ID_Group );
	return 4;
}





/*--------------------------------------------------*/
/*
	函数作用：设置 Profile Acceleration 寄存器 同步写入
	函数名：Dynamixel_Setting_Profile_Acceleration()
	输入参数： Profile Acceleration值数组， 需要写入的电机ID数组，传入的数组长度
	是否需要加入调试信息打印 , 加速度一般全部写0？

*/
void Dynamixel_Setting_Profile_Acceleration(int32_t* Profile_Acceleration_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	Dynamixel_Write_Sync_FBytes( Profile_Accleration_XH540 , ID_Group_Length  , Profile_Acceleration_Value  ,  ID_Group );
	
}


/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Profile Acceleration 寄存器 同步读取
	函数名：Dynamixel_Read_Profile_Acceleration()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Profile_Acceleration(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Profile_Accleration_Receive_XH540;
	Dynamixel_Sync_Read( Profile_Accleration_XH540 , 4 , ID_Group_Length ,  ID_Group );
	return 4;
}




/*--------------------------------------------------*/
/*
	函数作用：设置 Profile Veolcity 寄存器 同步写入
	函数名：Dynamixel_Setting_Profile_Veolcity()
	输入参数： Profile Acceleration值数组， 需要写入的电机ID数组，传入的数组长度
	在TimeBase的情况下，它定义的是什么时候到达目标位置

*/
void Dynamixel_Setting_Profile_Veolcity(int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	Dynamixel_Write_Sync_FBytes( Profile_Veolcity_XH540 , ID_Group_Length  , Profile_Veolcity_Value  ,  ID_Group );
	
}


/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Profile Veolcity 寄存器 同步读取
	函数名：Dynamixel_Read_Profile_Veolcity()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Profile_Veolcity(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Profile_Veolcity_Receive_XH540;
	Dynamixel_Sync_Read( Profile_Veolcity_XH540 , 4 , ID_Group_Length ,  ID_Group );
	return 4;
}




/*--------------------------------------------------*/
/*
	函数作用：设置 Goal_Position 寄存器 同步写入 (备注 ： 单独写入的话使用Set_angle)
	函数名：Dynamixel_Setting_Goal_Position()
	输入参数：Goal_Position值数组， 需要写入的电机ID数组，传入的数组长度
	在TimeBase的情况下，它定义的是什么时候到达目标位置

*/
void Dynamixel_Setting_Goal_Position(int32_t* Goal_Position_Value , uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	
	Dynamixel_Write_Sync_FBytes( Goal_Position_XH540 , ID_Group_Length  , Goal_Position_Value  ,  ID_Group );
	
}



/*--------------------------------------------------*/
/*
	函数作用：设置 Goal_Position 寄存器 单独写入 
	函数名：Dynamixel_Setting_Goal_Position()
	输入参数：Goal_Position值数组， 需要写入的电机ID数组，传入的数组长度
	在TimeBase的情况下，它定义的是什么时候到达目标位置

*/
void Dynamixel_Setting_Goal_Position_Inidvail(int32_t Goal_Position_Value , uint8_t ID)
{
	
	Dynamixel_Write_FBytes(ID ,Goal_Position_XH540 , Goal_Position_Value);
	
}




/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Present_Position 寄存器 同步写入
	函数名：Dynamixel_Read_Present_Position ()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	是否需要加入调试信息打印 , 返回 读取数据的字节长度
	//注意：读取位置前会自动清空位置函数！
*/
uint8_t Dynamixel_Read_Present_Position(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	//清空
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		Present_Pos_32t[i] = 0;
	}
	Rece_Com_Statement_1 = Present_Position_Receive_XH540;
	Dynamixel_Sync_Read( Present_Position_XH540 , 4 , ID_Group_Length ,  ID_Group );
	return 4;
}




/*--------------------------------------------------*/
/*
	函数作用：读取 Dynamxiel 的 Moving 寄存器 同步读取(XH540)
	函数名：Dynamixel_Read_Moving()
	输入参数： 需要读取的电机ID数组，传入的数组长度
	是否需要加入调试信息打印 , 返回 读取数据的字节长度

*/
uint8_t Dynamixel_Read_Moving(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Moving_Receive_XH540;
	Dynamixel_Sync_Read( Moving_XH540 , 1 , ID_Group_Length ,  ID_Group );
	return 1;
}




