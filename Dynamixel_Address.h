#ifndef __DYNAMIXEL_ADDRESS_H__
#define __DYNAMIXEL_ADDRESS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
		����ļ����������� Dynamixel�ĺ궨���
*/
	
//�������
//XH540����
#define Dynamixel_Num_XH540 4 
	
//��������ʼ�Ƕ�
	
	
//Ŀ��λ�üĴ�����ʼ��ַ
#define Goal_Position_XH540 0x0074
	
//��ǰλ�üĴ�����ʼ��ַ
#define Present_Position_XH540 0x0084
	
//Drive_Mode�Ĵ�����ʼ��ַ
#define Drive_Mode_XH540 0x000A
	
//Operating_Mode �Ĵ�����ʼ��ַ
#define Operating_Mode_XH540 0x000B
	
//Torque_Enable �Ĵ�����ʼ��ַ
#define Torque_Enable_XH540 0x0040
	
//homing_offset �Ĵ�����ʼ��ַ
#define homing_offset_XH540 0x0014

//Profile_Accleration �Ĵ�����ʼ��ַ
#define Profile_Accleration_XH540 0x006C

//Profile_Veolcity �Ĵ�����ʼ��ַ
#define Profile_Veolcity_XH540 0x0070

//Moving_Threshold �Ĵ�����ʼ��ַ
#define Moving_Threshold_XH540 0x0018

//Moving �Ĵ�����ʼ��ַ
#define Moving_XH540 0x007A

//AX-12

//�������
#define Dynamixel_Num_AX_12 1

//Ŀ��λ�üĴ�����ʼ��ַ
#define Goal_Position_AX_12 0x001E
	
//��ǰλ�üĴ�����ʼ��ַ
#define Present_Position_AX_12 0x0024
	
//��ǰλ����С���ƼĴ���
#define Minimum_Position_AX_12 0x0006
	
//��ǰλ��������ƼĴ���
#define Maximum_Position_AX_12 0x0008
	
//Torque_Enable �Ĵ�����ʼ��ַ
#define Torque_Enable_AX_12 0x0018

//Moving_Speed�Ĵ�����ʼ��ַ
#define Moving_Speed_AX_12 0x0020

//Present_Speed�Ĵ�����ʼ��ַ
#define Present_Speed_AX_12 0x0026




#ifdef __cplusplus
}
#endif
#endif /* __DYNAMIXEL_H__ */
