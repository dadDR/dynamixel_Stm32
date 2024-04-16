#ifndef __DYNAMIXEL_CONTROL_H__
#define __DYNAMIXEL_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

//ֱ�ߵ��������ز����ṹ��
typedef struct
{
	uint8_t ID;                 //���ID

  volatile uint8_t State;      //��ʼ��״̬/����������־

	int32_t Target_Position_Orig;//��������ʵ��ֵ���൱�ڵ��������С�ֶ�ֵ�˶�
	float Original_Angle;        //���ԭʼ�Ƕȣ���С��λ0.01degree
  float Target_Angle;          //���Ŀ��Ƕ�
  float Target_Angle_Upper;    //���Ŀ��Ƕ���������
  float Target_Angle_Lower;    //���Ŀ��Ƕ���������

	float Speed_PID;             //����ٶ���
	float Err;                   //��ǰƫ����
	float Err_Last;              //��һ��ƫ����
	float Err_Last_2;            //���ϸ�ƫ����
  float Err_Sum;               //ƫ���ۼ���

	float KP,KI,KD;              //PID����
	
	float Speed_Output;       //�ٶ������
  float Speed_Feedforward;   //�ٶ�ǰ����
	
	float Liner_Motor_Profile_Veolcity;			//�ٶ�Profileֵ
	float Liner_Motor_Profile_Accleration;	//���ٶ�Profileֵ
	
} Linear_Motor_Handler;


//����Drive_Mode �� ö�ٱ���
enum Drive_Mode_Statment{
    Inital_Mode = 0x00,
    Reverse_Mode = 0x01,
    Slave_Mode = 0x02,
    Time_based_Profile_Mode = 0x04
};

//���� Operating_Mode ��ö�ٱ���
enum Operaing_Mode_Statment{

	Current_Control_Mode = 0x00,
	
	Velocity_Control_Mode = 0x01,
	
	Position_Control_Mode = 0x03,	//Ĭ��ģʽ
	
	Extended_Position_Control_Mode = 0x04,	//��Ȧģʽ
	
	Current_based_Position_Control_Mode = 0x05

};


//����Dynamixel Receive_Complate_State �� ö�ٱ���
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
	
	Present_Position_Receive_HuanEr = 0x0A,//�ö����߶������λ
};


//����Dynamixel Receive_Complate_State �� ö�ٱ���
//�������
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



//��ʼ������ṹ������
void Linear_Motor_Handler_Group_Init(void);
																
																				
//��ʼ��ID_Group
void Dynamixel_ID_Group_Init(void);

//��ʼ���Ƕ�(�ĸ�Dynamixel��)
//����ṹ������ĵ�ַ
void Dynamixel_Angle_Group_Init(Linear_Motor_Handler* Motor_Group);




/*-------------------------------------------*/
/*
	������Dynamixel ����ĽǶȸ��º���

	��������void Ben_Linear_Motor_Handler_Update_Angle()
	�����������Ҫ�� �ṹ������ �� ������� , ����ĽǶ�����
	���ã�����Dynamixel �ṹ������ĽǶ�ֵ

*/
/*-------------------------------------------*/												
void Ben_Linear_Motor_Handler_Update_Angle( Linear_Motor_Handler* Motor_Group , uint8_t ID_Group_Length , float* Angle_Group );




/*--------------------------------------------------*/
/*
  ��������: ��Dynamixel ��ǰλ������Ϊ��� �� ͬ��д��
	��������Dynamixel_Setting_Zero
	��������� �ṹ������ , �����ID����ĳ���
	
*/
void Dynamixel_Setting_Zero( Linear_Motor_Handler* Linear_Motor_Handler_Group , uint8_t Linear_Motor_Handler_Group_Length);




/*--------------------------------------------------*/
/*
	�������ã���ʼ�� Dynamixel �� ͬ��д��
	��������Dynamixel_Setting_Init
	��������� Drive_Mode , Operating_Mode  , ID���� , �����ID����ĳ���
	
*/
void Dynamixel_Setting_Init(enum Drive_Mode_Statment Drive_Mode_St ,enum Operaing_Mode_Statment Operaing_Mode_St ,  uint8_t* ID_Group , uint8_t ID_Group_Length);


/*--------------------------------------------------*/
/*
	�������ã�����Profile Accleration �� Profile Veolcity�� ͬ��д��
	��������Dynamixel_Profile_Init
	��������� ���õ� Profile Accleration �� Profile Veolcity ֵ����, ID���� , �����ID����ĳ��� 
	
*/
void Dynamixel_Profile_Init( int32_t* Profile_Accleration_Value ,int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length);




/*--------------------------------------------------*/
/*
	�������ã�����Goal_Position�� ͬ��д��
	��������Dynamixel_Setting_Goal_Position
	��������� ���õ� Goal_Position ֵ����, ID���� , �����ID����ĳ��� , ����ṹ�����飨��Ҫ�޸���Ŀ��Ƕ�ֵ��
	
*/
void Dynamixel_Set_Goal_Position( int32_t* Goal_Position_Value, uint8_t* ID_Group , uint8_t ID_Group_Length , Linear_Motor_Handler* Linear_Motor_Handler_Group);


/*--------------------------------------------------*/
/*
	�������ã�ʹ��Dynamixel �� Torque_Enable �� ͬ����ȡ
	��������Dynamixel_Setting_Torque_Enable
	��������� ID���� , �����ID����ĳ���
	
*/
void Dynamixel_Set_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length );



//RS485��Dynamixel �ڵĻص�������
//����ʹ��
void Rx_RS485_func_Dynamixel_Ben(uint16_t Size);

void Dynamixel_Set_Torque_Disable( uint8_t* ID_Group , uint8_t ID_Group_Length );


//RS485��Dynamixel �ڵĻص�������
//����ʹ��
//Size�� �յ������ݳ���
//���� Motor_Group �� ָ��
Linear_Motor_Handler* Rx_RS485_func_Dynamixel_Ben_2(uint16_t Size , Linear_Motor_Handler* Motor_Group); //ISR�о����ܿ죬�Ա�־λ���洦��

////���״̬��ʼ��
//void Motor_Control_Restore(void);

////�����������
//void Motor_Control_Task(uint8_t State_num);

////��������������
//void Motor_Control_Task_Test(void);

////ʹ�ܵ�������õ����ģʽΪ��չλ��ģʽ
////���õ�Ŀ����Ϊ1-4�ŵ��
//void Ben_Dynamixel_Extended_PosMode_Enable(void);


////���ö������ĽǶȣ�α��
////ʵ��Ϊͨ�����������Ƕ�ֵ��������ͬ������ָ��
////����ר��
////����ֵ�ֱ�Ϊ�����ȵĽǶ�
////�������Ӧ����ṹ���Ŀ��Ƕ�
//void Ben_Dynamixel_Extended_PosControl(float Angle1, float Angle2, float Angle3, float Angle4);

#ifdef __cplusplus
}
#endif
#endif /*__DYNAMIXEL_CONTROL_H__ */
