#ifndef __DYNAMIXEL_H__
#define __DYNAMIXEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "Dynamixel_Control.h"
//����Drive Mode ����
/*-------------------*/
//Normal/Reverse Mode
#define Normal_Mode	 0
#define Reverse_Mode 1
	
//Profile�Ĳ���
#define Velocity_based_Profile 0
#define Time_based_Profile 1
/*--------------------*/	
	
	
//Torque On by Goal Update
//Ĭ��Ϊ����Enable���ٽ���ָ�� 

#define Enable 0x01
#define Disable 0x00
//0x00 �� ��Ӧ����ģʽ  0x01����Ӧ�ٶ�ģʽ 0x03����Ӧλ��ģʽ 0x04����Ӧ��չλ��ģʽ 0x05�����ڵ�����λ��ģʽ 0x10 PWM����ģʽ
#define Electric_Mode 0x00		 //����ģʽ
#define Speed_Mode 0x01        //�ٶ�ģʽ
#define Position_Mode 0x03     //λ��ģʽ
#define Extended_Position_Mode 0x04		//��չλ��ģʽ (������ �� ����)
#define Electric_Position_Mode 0x05		//���ڵ�����λ��ģʽ
#define PWM_Control_Mode			 0x10		//PWM����ģʽ

#define Position_Map_num 0.08789f //λ��ӳ�䳣��  ; ���壺�������λ��ֵ�ӽǶ�ֵ������ת��Ϊ0-4095������ֵ���룬�����ú���ʱֻ��Ҫд�븡�������͵ĽǶ�ֵ
#define Accleration_Map_num //���ٶ�ӳ�䳣�� , ����	

	
#define Speed_Map_num 0.22897f * 6.0f  //�ٶ�ӳ�䳣��



/*--------------------------------------------------*/
/*
	�������ã����� Dynamxiel �� Drive_Mode �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Drive_Mode
	��������� �Ĵ�������λ(uint8_t) , ID���� , �����ID����ĳ���
	
*/
void Dynamixel_Setting_Drive_Mode(uint8_t Drive_Mode_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length);




/*--------------------------------------------------*/
/*
	�������ã���ȡ DriMode Mode �Ĵ��� ͬ����ȡ
	��������Dynamixel_Reading_Drive_Mode()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	��ȡ DriMode Mode �Ĵ��������� ���ض�ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Reading_Drive_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length );



/*--------------------------------------------------*/
/*
	�������ã����� Dynamxiel �� Operating_Mode �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Operating_Mode
	��������� �Ĵ�������λ(uint8_t) , ID���� , �����ID����ĳ���
	
*/
void Dynamixel_Setting_Operating_Mode(uint8_t Operating_Mode_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã���ȡ Operating Mode �Ĵ��� ͬ����ȡ
	��������Dynamixel_Reading_Operating_Mode()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	��ȡ Operating Mode �Ĵ��������� ���ض�ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Reading_Operating_Mode( uint8_t* ID_Group , uint8_t ID_Group_Length );



/*--------------------------------------------------*/
/*
	�������ã����� Dynamxiel �� Torque_Enable �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Torque_Enable
	��������� �Ĵ�������λ(uint8_t), ID���� , �����ID����ĳ���
	
*/
void Dynamixel_Setting_Torque_Enable(uint8_t Torque_Enable_Value  , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã���ȡ Torque_Enable �Ĵ��� ͬ����ȡ
	��������Dynamixel_Reading_Torque_Enable()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	��ȡ Torque_Enable �Ĵ��������� ���ض�ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Reading_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length );




/*--------------------------------------------------*/
/*
	�������ã����� homing offset �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Homing_Offset()
	��������� float Homing_Offset ��Ҫд��ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ

*/
void Dynamixel_Setting_Homing_Offset(int32_t* Homing_Offset_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� homing offset �Ĵ��� ͬ����ȡ
	��������Dynamixel_Read_Homing_Offset()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Homing_Offset(uint8_t* ID_Group , uint8_t ID_Group_Length);




/*--------------------------------------------------*/
/*
	�������ã����� Profile Acceleration �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Profile_Acceleration()
	��������� Profile Accelerationֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ , ���ٶ�һ��ȫ��д0��

*/
void Dynamixel_Setting_Profile_Acceleration(int32_t* Profile_Acceleration_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� Profile Acceleration �Ĵ��� ͬ����ȡ
	��������Dynamixel_Read_Profile_Acceleration()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Profile_Acceleration(uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã����� Profile Veolcity �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Profile_Veolcity()
	��������� Profile Accelerationֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	��TimeBase������£����������ʲôʱ�򵽴�Ŀ��λ��

*/
void Dynamixel_Setting_Profile_Veolcity(int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� Profile Veolcity �Ĵ��� ͬ����ȡ
	��������Dynamixel_Read_Profile_Veolcity()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Profile_Veolcity(uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã����� Goal_Position �Ĵ��� ͬ��д��
	��������Dynamixel_Setting_Goal_Position()
	���������Goal_Positionֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	��TimeBase������£����������ʲôʱ�򵽴�Ŀ��λ��

*/
void Dynamixel_Setting_Goal_Position(int32_t* Goal_Position_Value , uint8_t* ID_Group , uint8_t ID_Group_Length );



/*--------------------------------------------------*/
/*
	�������ã����� Goal_Position �Ĵ��� ����д�� 
	��������Dynamixel_Setting_Goal_Position()
	���������Goal_Positionֵ���飬 ��Ҫд��ĵ��ID���飬��������鳤��
	��TimeBase������£����������ʲôʱ�򵽴�Ŀ��λ��

*/
void Dynamixel_Setting_Goal_Position_Inidvail(int32_t Goal_Position_Value , uint8_t ID);



/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� Present_Position �Ĵ��� ͬ��д��
	��������Dynamixel_Read_Present_Position ()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ , ���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Present_Position(uint8_t* ID_Group , uint8_t ID_Group_Length);



/*--------------------------------------------------*/
/*
	�������ã���ȡ Dynamxiel �� Moving �Ĵ��� ͬ����ȡ(XH540)
	��������Dynamixel_Read_Moving()
	��������� ��Ҫ��ȡ�ĵ��ID���飬��������鳤��
	�Ƿ���Ҫ���������Ϣ��ӡ , ���� ��ȡ���ݵ��ֽڳ���

*/
uint8_t Dynamixel_Read_Moving(uint8_t* ID_Group , uint8_t ID_Group_Length);





////ͬ�����Ƕ�����,��������������,��һ������Ϊ�������,����ΪID_num�����ID
//uint8_t Sync_Read_Motor_Angle(uint16_t ID_num, ...);

////���Ƕ�����
//uint8_t Read_Motor_Angle(uint8_t Motor_ID);

////���Ť������
//void Sync_Set_Motor_Torque_Enable(uint8_t State, uint16_t ID_num, ...);

////�������ģʽ�л�
//void Sync_Set_Motor_Operating_Mode(uint8_t Mode, uint16_t ID_num, ...);

////����ٶȿ���
//void Set_Motor_Speed(uint8_t Motor_ID, float Speed);

////����Ƕȿ���
//void Set_Motor_Position(uint8_t Motor_ID, float Position);

////���ͬ��д��ǶȲ���(������)
////ת����ͬһ���Ƕȣ�ֻ����һ���Ƕ�
////ͬʱ�ڶ������ĽǶȼĴ�����д��Ƕ�
//void Sync_Set_Motor_Position(float Position , uint16_t ID_num, ...);

////��ǰ����д��λ�ã������ԣ�
////д�����Ҫִ��Actionָ���������
//void Reg_Set_Motor_Position(float Position , uint8_t Motor_ID);

////ִ����ǰд�����õĴ��루�����ԣ�
//void Reg_Action(uint8_t Motor_ID);

///*
//	��������Dynamixel_Write_OBytes (������)
//	�������ã���Dynamixel�ļĴ�����д��� Byte ������ , ��ʹ�� Syncģʽ
//	�����������ʼ��ַ Address �� д��Ķ�λ���ݵ�ֵ(��λ ֱ����float�ض��ֽڰ�)
//*/
//void Dynamixel_Write_OBytes( uint8_t Motor_ID, uint16_t Address , float Value);


///*
//	��������Dynamixel_Write_TBytes (������)
//	�������ã���Dynamixel�ļĴ�����д��� Byte ������ , ��ʹ�� Syncģʽ
//	�����������ʼ��ַ Address �� д��Ķ�λ���ݵ�ֵ(��λ ֱ����float�ض��ֽڰ�)
//*/
//void Dynamixel_Write_TBytes( uint8_t Motor_ID, uint16_t Address , float Value);


///*
//	��������Dynamixel_Write_FBytes (������)
//	�������ã���Dynamixel�ļĴ�����д���� Byte ������ , ��ʹ�� Syncģʽ
//	�����������ʼ��ַ Address �� д�����λ���ݵ�ֵ(��λ һ����float?)
//*/
//void Dynamixel_Write_FBytes( uint8_t Motor_ID, uint16_t Address , float Value);


///*--------------------------------------------------------*/
///*
//	��������Drive_Mode_Write_Sync
//	�������ã����� Drive Mode �� �Ĵ���ֵ , ���� Profile Configuration �� ���ӻ��������ʱ�����ã� , ����ͷ���ģʽ
//  ����ֵ��  Profile Configuration , ����ͷ���д��
//*/
//void Drive_Mode_Write_Sync(uint8_t Profile_Configuration , uint8_t Direction , uint16_t ID_num, ...);


#ifdef __cplusplus
}
#endif
#endif /* __DYNAMIXEL_H__ */
