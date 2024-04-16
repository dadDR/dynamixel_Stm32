#include "Dynamixel.h"
#include "Dynamixel_Address.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "User_Delay.h"
#include "User_Usart.h"
#include "Dynamixel_basic.h"

//�����ܵ�ID_Group
extern uint8_t Dynamixel_ID_Group[Dynamixel_Num_XH540];
extern int32_t Present_Pos_32t[4];
//�������ձ�־λ
//(Dynamixel��)
//�޸ĺ��һ����պ����Ľ�����ɱ�־λ��������Ҫ�������
extern enum Receive_Complate_State Rece_Com_Statement_1;

/*--------------------------------------------------*/
/*
	�������ã����� Dynamxiel �� Drive_Mode �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Drive_Mode
	��������� �Ĵ�������λ(uint8_t) , ID���� , �����ID����ĳ���
	
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
	�������ã���ȡ DriMode Mode �Ĵ��� ͬ����ȡ
	��������Dynamixel_Reading_Drive_Mode()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	��ȡ DriMode Mode �Ĵ��������� ���ض�ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Reading_Drive_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	
	//Dynamixel_Sync_Fast_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group )
	Dynamixel_Sync_Read(Drive_Mode_XH540, 1 , ID_Group_Length , ID_Group );
	return 1;
}




/*--------------------------------------------------*/
/*
	�������ã����� Dynamxiel �� Operating_Mode �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Operating_Mode
	��������� �Ĵ�������λ(uint8_t) , ID���� , �����ID����ĳ���
	
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
	�������ã���ȡ Operating Mode �Ĵ��� ͬ����ȡ
	��������Dynamixel_Reading_Operating_Mode()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	��ȡ Operating Mode �Ĵ��������� ���ض�ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Reading_Operating_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	//Dynamixel_Sync_Fast_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group )
	Dynamixel_Sync_Read( Operating_Mode_XH540, 1 , ID_Group_Length , ID_Group );
	return 1;
}



/*--------------------------------------------------*/
/*
	�������ã����� Dynamxiel �� Torque_Enable �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Torque_Enable
	��������� �Ĵ�������λ(uint8_t), ID���� , �����ID����ĳ���
	
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
	�������ã���ȡ Torque_Enable �Ĵ��� ͬ����ȡ
	��������Dynamixel_Reading_Torque_Enable()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	��ȡ Torque_Enable �Ĵ��������� ���ض�ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Reading_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	Rece_Com_Statement_1 = Torque_Enable_Receive_XH540;
	Dynamixel_Sync_Read( Torque_Enable_XH540 , 1 , ID_Group_Length , ID_Group );
	return 1;
}




/*--------------------------------------------------*/
/*
	�������ã����� homing offset �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Homing_Offset()
	��������� float Homing_Offset ��Ҫд��ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ

*/
void Dynamixel_Setting_Homing_Offset(int32_t* Homing_Offset_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	Dynamixel_Write_Sync_FBytes( homing_offset_XH540 , ID_Group_Length  ,Homing_Offset_Value  ,  ID_Group );

}


/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� homing offset �Ĵ��� ͬ����ȡ
	��������Dynamixel_Read_Homing_Offset()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Homing_Offset(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Homing_Offset_Receive_XH540; //�Ƚ����ձ�־λ��λ
	Dynamixel_Sync_Read( homing_offset_XH540 , 4 , ID_Group_Length ,  ID_Group );
	return 4;
}





/*--------------------------------------------------*/
/*
	�������ã����� Profile Acceleration �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Profile_Acceleration()
	��������� Profile Accelerationֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ , ���ٶ�һ��ȫ��д0��

*/
void Dynamixel_Setting_Profile_Acceleration(int32_t* Profile_Acceleration_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	Dynamixel_Write_Sync_FBytes( Profile_Accleration_XH540 , ID_Group_Length  , Profile_Acceleration_Value  ,  ID_Group );
	
}


/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� Profile Acceleration �Ĵ��� ͬ����ȡ
	��������Dynamixel_Read_Profile_Acceleration()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Profile_Acceleration(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Profile_Accleration_Receive_XH540;
	Dynamixel_Sync_Read( Profile_Accleration_XH540 , 4 , ID_Group_Length ,  ID_Group );
	return 4;
}




/*--------------------------------------------------*/
/*
	�������ã����� Profile Veolcity �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Profile_Veolcity()
	��������� Profile Accelerationֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	��TimeBase������£����������ʲôʱ�򵽴�Ŀ��λ��

*/
void Dynamixel_Setting_Profile_Veolcity(int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	Dynamixel_Write_Sync_FBytes( Profile_Veolcity_XH540 , ID_Group_Length  , Profile_Veolcity_Value  ,  ID_Group );
	
}


/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� Profile Veolcity �Ĵ��� ͬ����ȡ
	��������Dynamixel_Read_Profile_Veolcity()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Profile_Veolcity(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Profile_Veolcity_Receive_XH540;
	Dynamixel_Sync_Read( Profile_Veolcity_XH540 , 4 , ID_Group_Length ,  ID_Group );
	return 4;
}




/*--------------------------------------------------*/
/*
	�������ã����� Goal_Position �Ĵ��� ͬ��д�� (��ע �� ����д��Ļ�ʹ��Set_angle)
	��������Dynamixel_Setting_Goal_Position()
	���������Goal_Positionֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	��TimeBase������£����������ʲôʱ�򵽴�Ŀ��λ��

*/
void Dynamixel_Setting_Goal_Position(int32_t* Goal_Position_Value , uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	
	Dynamixel_Write_Sync_FBytes( Goal_Position_XH540 , ID_Group_Length  , Goal_Position_Value  ,  ID_Group );
	
}



/*--------------------------------------------------*/
/*
	�������ã����� Goal_Position �Ĵ��� ����д�� 
	��������Dynamixel_Setting_Goal_Position()
	���������Goal_Positionֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	��TimeBase������£����������ʲôʱ�򵽴�Ŀ��λ��

*/
void Dynamixel_Setting_Goal_Position_Inidvail(int32_t Goal_Position_Value , uint8_t ID)
{
	
	Dynamixel_Write_FBytes(ID ,Goal_Position_XH540 , Goal_Position_Value);
	
}




/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� Present_Position �Ĵ��� ͬ��д��
	��������Dynamixel_Read_Present_Position ()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ , ���� ��ȡ���ݵ��ֽڳ���
	//ע�⣺��ȡλ��ǰ���Զ����λ�ú�����
*/
uint8_t Dynamixel_Read_Present_Position(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	//���
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
	�������ã���ȡ Dynamxiel �� Moving �Ĵ��� ͬ����ȡ(XH540)
	��������Dynamixel_Read_Moving()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ , ���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Moving(uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	Rece_Com_Statement_1 = Moving_Receive_XH540;
	Dynamixel_Sync_Read( Moving_XH540 , 1 , ID_Group_Length ,  ID_Group );
	return 1;
}




