#ifndef __DYNAMIXEL_BASIC_H__
#define __DYNAMIXEL_BASIC_H__

#include "main.h"
	
#define Delay_Time_ms 10     //������ʱʱ�䣬���ڲ���

	
	
/*---------------------------------*/
//������д������ ����ͬ��д�� һ �� ���������� �� ��ͬ��д��һ �� ����������
/*
	Dynamxiel ��д�Ĵ���ָ�����Ϊ��ָ���Ĵ����ĵ�ַд������
*/


/*-------------------------------------------------*/
/*
	��Sync
*/
/*-------------------------------------------------*/


/*
	��������Dynamixel_Write_OBytes (������)
	�������ã���Dynamixel�ļĴ�����д��һ Byte ������ , ��ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д���1λ���ݵ�ֵ(1λ ֱ����float�ض��ֽڰ�)
*/
void Dynamixel_Write_OBytes( uint8_t Motor_ID, uint16_t Address , uint8_t Value);


/*
	��������Dynamixel_Write_TBytes (������)
	�������ã���Dynamixel�ļĴ�����д��� Byte ������ , ��ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д��Ķ�λ���ݵ�ֵ(��λ ֱ����float�ض��ֽڰ�)
*/
void Dynamixel_Write_TBytes( uint8_t Motor_ID, uint16_t Address , uint16_t Value);


/*
	��������Dynamixel_Write_FBytes (������)
	�������ã���Dynamixel�ļĴ�����д���� Byte ������ , ��ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д�����λ���ݵ�ֵ(��λ һ����float?)
*/
void Dynamixel_Write_FBytes( uint8_t Motor_ID, uint16_t Address , int32_t Value);


/*----------------------------------------------------------------*/
/*
	Sync
*/
/*--------------------------------------------------------------*/



/*
	��������Dynamixel_Write_Sync_OBytes (������)
	�������ã���Dynamixel�ļĴ�����д��һ Byte ������ , ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д���һλ���ݵ�ֵ(��һ��float����)   ,  ID������ ,
*/
void Dynamixel_Write_Sync_OBytes( uint16_t Address ,uint8_t ID_num ,uint8_t* Value_Group  , uint8_t* ID_Group );


/*
	��������Dynamixel_Write_Sync_TBytes (������)
	�������ã���Dynamixel�ļĴ�����д��� Byte ������ , ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д��Ķ�λ���ݵ�ֵ(��һ��float����)   ,  ID������ ,
*/
void Dynamixel_Write_Sync_TBytes( uint16_t Address ,uint8_t ID_num ,uint16_t * Value_Group  , uint8_t* ID_Group );


/*
	��������Dynamixel_Write_Sync_FBytes (������)
	�������ã���Dynamixel�ļĴ�����д���� Byte ������ , ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д�����λ���ݵ�ֵ(��һ��float����)   , ID������ ,
*/
void Dynamixel_Write_Sync_FBytes( uint16_t Address ,uint8_t ID_num ,int32_t* Value  , uint8_t* ID_Group );


/*---------------------------------*/
//�����Ķ�ȡ���� ����ͬ����ȡ һ �� �� Byte ���� �� ��ͬ����ȡ һ �� �� Byte ����
/*
	Dynamxiel ��д�Ĵ���ָ�����Ϊ��ָ���Ĵ����ĵ�ַд������
*/


/*-------------------------------------------------*/
/*
	��Sync
*/
/*-------------------------------------------------*/













/*-------------------------------------------------*/
/*
	Sync
*/
/*-------------------------------------------------*/


/*--------------------------------------------*/
/*
	�������� �� Dynamixel_Read_FuncFByte
	�������� �� ͬ����ȡ�Ĵ��������� , ���ݽ���ΪRS485���ݷ��ͻ���
	��������ֵ������������������ͬʱ��������IDֵ , ������Ҫ��ȡ�ļĴ�����ַ , ������Ҫ��ȡ�����ݳ���(��ȡ�೤������) , ��Ҫ��ȡ�ĵ��ID������
	������
*/  
void Dynamixel_Sync_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group );




/*--------------------------------------------*/
/*
	�������� �� Dynamixel_Sync_Fast_Read(�������ԣ����̿���)
	�������� �� ͬ����ȡ�Ĵ��������� , ���ݽ���ΪRS485���ݷ��ͻ���
	��������ֵ������������������ͬʱ��������IDֵ , ������Ҫ��ȡ�ļĴ�����ַ , ������Ҫ��ȡ�����ݳ���(��ȡ�೤������) , ��Ҫ��ȡ�ĵ��ID������
	������
*/  
void Dynamixel_Sync_Fast_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group );




/*-------------------------------------------------*/
/*
	���Գ���
*/
/*-------------------------------------------------*/



/*---------------------------------*/
//���ݶ�ȡ����������д�뺯����ͬʱ����
//����Dynamixel �����ϵĽ̳�

void Dynamixel_Basic_Func_Pose(void);






#endif /* __DYNAMIXEL_H__ */
