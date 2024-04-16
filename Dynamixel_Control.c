#include "Dynamixel_Control.h"
#include "Control_Dynamixel_Matrix.h"
#include "Dynamixel.h"
#include "motor_control.h"
#include "User_Usart.h"
#include "User_Delay.h"
#include "Dynamixel_Address.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "General_define.h"
#include "crc_ccitt.h"
#include "GO_Utree_Motor.h"
#include "serial_servo.h"
#include "serial_servo_porting.h"
#include "usbd_cdc_if.h"

//���������ʼ��

#define DeadBand 0.5f         //��ʼ������

#define Periodic_Angle 360.00f	//���ڽǶ�

#define Frequency 3.0f   //��תƵ�ʣ���λHz

#define Linear_Motor_Init_Angle 0.0f  //ֱ�ߵ����ʼ���Ƕ�

#define Kp 0.1f
#define Ki 0.0f
#define Kd 0.0f

//����Dynamixel��Torque_Enable ����λ
#define Dynamixel_Torque_Enable 1
#define Dynamixel_Torque_Disable 0

//�������������
#define Dynamixel_Ben_Num 4


uint32_t xLastTime = 0;
uint32_t Start_Time = 0;

uint16_t Query_Angle = 0;  //��ѯλ��

uint8_t Receive_Complete_state = 0;  //������ɱ�־λ , ÿһλ����һ����� �� �����û����Ҫ�յ�λ��ָ��ʱ����1

enum Receive_Complate_State Receive_complete_State_2 = Not_Receive_XH540;	//�°������ɱ�־λ

//�޸ĺ��һ����պ����Ľ�����ɱ�־λ��������Ҫ�������
enum Receive_Complate_State Rece_Com_Statement_1 = Not_Receive_XH540;


//��ʼ�������Ķ���ǶȲ����������趨Ϊ������������̬��
float Ben_Angle1=90,Ben_Angle2=90,Ben_Angle3=90,Ben_Angle4=90;

extern uint8_t queue_trash[QU_LENGTH];  //���ж�������

uint8_t Ben_iscontrol = 0;//�����Ŀ��Ʊ�־λ������λ��1ʱ�����ƶ��

//�����ܵ� ID_Group
uint8_t Dynamixel_ID_Group[Dynamixel_Num_XH540] = { 1 , 2 , 3 , 4 };


//��������ṹ������
Linear_Motor_Handler Linear_Motor_Handler_Group[Dynamixel_Num_XH540];



//����Dynamixel��Drive_Mode״̬ö������
//enum Drive_Mode_Statment Drive_Mode_Statment1 = Inital_Mode;

//�����Ƕ� Group 
float Ben_Angle_Group[Dynamixel_Num_XH540];

/*-----------------------------------------------------*/
//RS485 �ĿڵĻص�����ʹ�õ�״̬����
//XH540״̬����
//����Present Position ʹ�õĽǶ�����
int32_t Present_Pos_32t[4];
//����Homing Offsetʹ�õĽǶ�����
int32_t Homing_Offset_Goup[4];
//���� Profile Veolcity ����ֵ����
int32_t Profile_Veolcity_Goup[4];
//���� Torque_Enable ����
uint8_t Torque_Enable_Goup[4];
//���� is_Moving ����
uint8_t is_Moving_Goup[4] = {0};
//AX-12״̬����

//Utree״̬����
MOTOR_recv Utree_M8010Rec_Group[1];

//�����ö����״̬����
int16_t HuangEr_Position_Group[1];

/*-------------------------------------------------------*/

//������ã��͵���������ĺ�������� �� �����������ָ����󣬾ͽ�����0
//һ��Dynamixel �ǲ����ؽǶȵģ�

//����PID����
//��ʼ���������
// //���I D//��ʼ��״̬/����������־  //�����������С��λ����   //���ԭʼ�Ƕȣ���С��λ0.01deg ree//���Ŀ�� �Ƕ�//���Ŀ��Ƕ����� ����//���Ŀ��Ƕ��������� //����ٶ���//��ǰƫ����
//��һ��ƫ����//���ϸ�ƫ����//ƫ���ۼ���//PID����//�ٶ������//�ٶ�ǰ����//Profile_Veolcity//Profile_Accleration
//�����ĸ�����Ӧ�ĸ��ؽڵĵ��
Linear_Motor_Handler Linear_Motor_1 = { 1 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f , 0.0f , 0.0f};
Linear_Motor_Handler Linear_Motor_2 = { 2 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f , 0.0f , 0.0f};
Linear_Motor_Handler Linear_Motor_3 = { 3 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f , 0.0f , 0.0f};
Linear_Motor_Handler Linear_Motor_4 = { 4 , 1 ,0, 0.0f , Linear_Motor_Init_Angle , Linear_Motor_Init_Angle + DeadBand , Linear_Motor_Init_Angle - DeadBand , 0 , 0 , 0 , 0 , 0
																				, Kp , Ki , Kd
																				, 0.0f , 0.0f  , 0.0f , 0.0f};
																				
//��ʼ��Dynamixel����ṹ������
void Linear_Motor_Handler_Group_Init(void)
{
	Linear_Motor_Handler_Group[0] = Linear_Motor_1;
	Linear_Motor_Handler_Group[1] = Linear_Motor_2;
	Linear_Motor_Handler_Group[2] = Linear_Motor_3;
	Linear_Motor_Handler_Group[3] = Linear_Motor_4;
}
																		
																				
//��ʼ��ID_Group
void Dynamixel_ID_Group_Init(void)
{
	//��ʼ���ܵ�ID_Group
	for(int i = 0 ; i < Dynamixel_Num_XH540 ; i++)
	{
		Dynamixel_ID_Group[i] = Linear_Motor_Handler_Group[i].ID;
	}
	
//	//���Բ���
//	User_UART_STLINK_printf("ID_Group :%s \r\n ",Dynamixel_ID_Group);	
	
}


//��ʼ���Ƕ�(�ĸ�Dynamixel��)
//����ṹ������ĵ�ַ
//����������ִ��
void Dynamixel_Angle_Group_Init(Linear_Motor_Handler* Motor_Group)
{
	Dynamixel_Read_Present_Position(Dynamixel_ID_Group , Dynamixel_Ben_Num);//��ȡ���еĽǶȣ���ǰ�Ƕȣ�
	
	//���ýǶ�
	//һ�ŵ��
	Motor_Group[0].Target_Angle = ((float)Present_Pos_32t[0])* Position_Map_num;
	//���ŵ��
	Motor_Group[1].Target_Angle = ((float)Present_Pos_32t[1])* Position_Map_num;
	//���ŵ��
	Motor_Group[2].Target_Angle = ((float)Present_Pos_32t[2])* Position_Map_num;
	//�ĺŵ��
	Motor_Group[3].Target_Angle = ((float)Present_Pos_32t[3])* Position_Map_num;
	
	//���ýǶȱ���
	Ben_Angle1=90.f;
	
	Ben_Angle2=90.f;
	
	Ben_Angle3=90.f;
	
	Ben_Angle4=90.f;
//	
	//���ýǶȱ���
	Ben_Angle_Group[0] = 90.f;
	Ben_Angle_Group[1] = 90.f;
	Ben_Angle_Group[2] = 90.f;
	Ben_Angle_Group[3] = 90.f;
	
}

/*-------------------------------------------*/
/*
	������Dynamixel ����ĽǶȸ��º���

	��������void Ben_Linear_Motor_Handler_Update_Angle()
	�����������Ҫ�� �ṹ������ �� ������� , ����ĽǶ�����
	���ã�����Dynamixel �ṹ������ĽǶ�ֵ
	//�޸����ӵ��Բ���
*/
/*-------------------------------------------*/												
void Ben_Linear_Motor_Handler_Update_Angle( Linear_Motor_Handler* Motor_Group , uint8_t ID_Group_Length , float* Angle_Group )
{
	//�ݴ�ID����
	uint8_t ID_Group_Motor_temp[ID_Group_Length];
	
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		ID_Group_Motor_temp[i] = Motor_Group[i].ID;
		//��ӡID
		User_UART_STLINK_printf("UPANGLE Motor %d ID is %d \r\n" , i ,Motor_Group[i].ID);
	}
	
	Dynamixel_Read_Present_Position(ID_Group_Motor_temp , ID_Group_Length);//��ȡ���еĽǶȣ���ǰ�Ƕȣ�
	//������д����½Ƕ�
	for(int i = 0 ; i < ID_Group_Length ; i++)
	{
		Motor_Group[i].Original_Angle = ((float)Present_Pos_32t[i]) * Position_Map_num;
		
	}
}



//����ʽPID����
void Angle_Incremental_PID (Linear_Motor_Handler *PID)
{
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
	PID->Err = PID->Target_Angle - PID->Original_Angle;

	/*PID�㷨ʵ��*/
	PID->Speed_PID += PID->KP * ( PID->Err - PID->Err_Last )
								 + PID->KI * PID->Err
								 + PID->KD *(PID->Err - 2*PID->Err_Last + PID->Err_Last_2);

	/*�������*/
	PID->Err_Last_2 = PID->Err_Last;
	PID->Err_Last = PID->Err;
	
	/*ʵ�����ת��*/
	PID->Speed_Output = PID->Speed_PID;
}



/*----------------------------------------------------------------------*/
/*
	RS485���ջص���������
*/
/*----------------------------------------------------------------------*/
void Present_Position_Receive_Func_XH540(void);
void Homing_Offset_Receive_Func_XH540(void);
void Profile_Veolcity_Receive_Func_XH540(void);
void Profile_Torque_Receive_Func_XH540(void);
void Moving_Receive_Func_XH540(void);
//�ö�������պ���
void Position_Receive_Func_HuangEr(void);

//RS485��Dynamixel �ڵĻص�������
//����ʹ��(����ʹ�õ�����϶ࣿ)
//Size�� �յ������ݳ���
void Rx_RS485_func_Dynamixel_Ben(uint16_t Size ) //ISR�о����ܿ죬�Ա�־λ���洦��
{
	//XH540����
	//Present Location_XH540
	if(Rece_Com_Statement_1 == Present_Position_Receive_XH540) //���뵱ǰλ�ý��� �� �������յ��ı���
	{
		Present_Position_Receive_Func_XH540();
	}
	
	//Homing Offset_XH540
	if(Rece_Com_Statement_1 == Homing_Offset_Receive_XH540) //���뵱ǰλ�ý��� �� �������յ��ı���
	{
		Homing_Offset_Receive_Func_XH540();
	}
	
	
	//Profile Veolcity_XH540
	if(Rece_Com_Statement_1 == Profile_Veolcity_Receive_XH540) //���뵱ǰλ�ý��� �� �������յ��ı���
	{
		Profile_Veolcity_Receive_Func_XH540();
	}
	
	
	//Torque_Enable_XH540
	if(Rece_Com_Statement_1 == Torque_Enable_Receive_XH540) //���뵱ǰλ�ý��� �� �������յ��ı���
	{
		Profile_Torque_Receive_Func_XH540();
	}
	
	//Moving_XH540
	if(Rece_Com_Statement_1 == Moving_Receive_XH540)//��ɵ�ǰλ���˶�
	{
		Moving_Receive_Func_XH540();
	}
	
	//AX-12����
	
	
	//Utree�������
	if(Rece_Com_Statement_1 == Receive_Data_Utree)
	{
		if(Size == 0)
		{
			User_UART_STLINK_printf(" HAL_Time_Out \r\n");
			return ;
		}
		
		if(Size != sizeof(Rx_RS485.rx_buff_temp))
		{
			User_UART_STLINK_printf(" HAL_ERROR \r\n");
			return;
		}
		
		uint8_t *rp = Rx_RS485.rx_buff_temp;//���У�������Ҫȡ�������鸴����
		
		
    if(rp[0] == 0xFE && rp[1] == 0xEE)//У���ͷ
    {
				//�ж�ID
				if(Utree_M8010Rec_Group[0].motor_id == (rp[2]&0xF))
				{
					
					//ȡ��CRC��
					uint16_t CRC16_Utree = ((rp[14]&0xFF)<<8) + rp[15]&0xFF;
					//�ж�CRC
					if(CRC16_Utree != crc_ccitt_Utree(0, rp, 14))
					{
						User_UART_STLINK_printf(" Utree REC CRC error \r\n");
						
						return ;
					}			
					else
					{
						//���
						Utree_M8010Rec_Group[0].motor_id = (rp[2]&0xF);
						Utree_M8010Rec_Group[0].mode = ((rp[2]>>4)&0xF);
						Utree_M8010Rec_Group[0].T = (float)(((rp[3]&0xFF)<<8) + ((rp[4]&0xFF))) / 256;//ʵ�ʹؽ����ת��,ת��Ϊ���صĻ���Ҫת��
						Utree_M8010Rec_Group[0].W = (float)(((rp[5]&0xFF)<<8) + ((rp[6]&0xFF))) / 256 * 2 * PI;//ʵ���ٶ�
						Utree_M8010Rec_Group[0].Pos = (float)(((rp[7]&0xFF)<<24) + ((rp[8]&0xFF)<<16) + ((rp[9]&0xFF)<<8) + ((rp[10]&0xFF))) / 32768 * 2 * PI;//ʵ�ʹؽ�λ��
						Utree_M8010Rec_Group[0].Temp = rp[11];	//�¶�
						Utree_M8010Rec_Group[0].MError = rp[12]>>1;	//������
					}
					
				}
        
        return ;
    }
		else
		{
			User_UART_STLINK_printf(" Bag_Hander_notCorrect \r\n");
		}
		
	}
	
	//�ö��������
	//Present_Position_Receive_HuanEr
	if(Rece_Com_Statement_1 == Present_Position_Receive_HuanEr) //���뵱ǰλ�ý��� �� �������յ��ı���
	{
		//Profile_Veolcity_Receive_Func_XH540();
		Position_Receive_Func_HuangEr();
	}
	
}

/*--------------------------------------------------*/
/*
  ��������: ��Dynamixel ��ǰλ������Ϊ��� �� ͬ��д��
	��������Dynamixel_Setting_Zero
	��������� �ṹ������ , �����ID����ĳ���
	ܳ������������ӳ��ϵ����⣡����
*/
void Dynamixel_Setting_Zero( Linear_Motor_Handler* Linear_Motor_Handler_Group , uint8_t Linear_Motor_Handler_Group_Length)
{

	//����ID����
	uint8_t ID_Group[Linear_Motor_Handler_Group_Length];
	//homing_offset����
	int32_t Homing_offset_buf[Linear_Motor_Handler_Group_Length];
	
	//����Drive Mode
	enum Drive_Mode_Statment	Drive_Mode_Test_2 = Time_based_Profile_Mode;
	//User_UART_STLINK_printf("Drive_Mode_Test_2 : %d \r\n " , Drive_Mode_Test_2 );
	//����ID
	//�������ID�����⣿
	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		ID_Group[i] = Linear_Motor_Handler_Group[i].ID;
	}
	
	//��HomeOfffseting����
	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Homing_offset_buf[i] = 0;
	  User_UART_STLINK_printf(" Homing Offset after %d is %d \r\n" , i , Homing_offset_buf[i]);
	}
	
	//ʧ��
	Dynamixel_Set_Torque_Disable(ID_Group , Linear_Motor_Handler_Group_Length);
	
	//��home_offset д��Dynamixel
	Dynamixel_Setting_Homing_Offset(Homing_offset_buf , ID_Group , Linear_Motor_Handler_Group_Length);
//	//��ȡHoming Offset
//	Homing_Offset_Goup[0] = 0;
//	Homing_Offset_Goup[1] = 0;
//	Dynamixel_Read_Homing_Offset(ID_Group , Linear_Motor_Handler_Group_Length);
	
	//��ȡPresent Position
	//��λ�ý���������0
	for(int i = 0 ;  i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Present_Pos_32t[i] = 0;
	}
	Dynamixel_Read_Present_Position(ID_Group , Linear_Motor_Handler_Group_Length);

	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		User_UART_STLINK_printf("Dynamixel 1_%d Pos : %d \r\n " , i , Present_Pos_32t[i] );
	}
	//Homing ���� �� Present Position
	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Homing_offset_buf[i] -= Present_Pos_32t[i];
		User_UART_STLINK_printf(" Homing Offset after2 %d is %d \r\n" , i , Homing_offset_buf[i]);
	}
	//д��Homing Offset
	Dynamixel_Setting_Homing_Offset(Homing_offset_buf , ID_Group , Linear_Motor_Handler_Group_Length);
	//��ȡPresent Position
	//��λ�ý���������0 , �� �ṹ���Origin�Ƕ���0
	for(int i = 0 ;  i < Linear_Motor_Handler_Group_Length ; i++)
	{
		Linear_Motor_Handler_Group[i].Original_Angle = 0;
		Present_Pos_32t[i] = 0;
	}
	Dynamixel_Read_Present_Position(ID_Group , Linear_Motor_Handler_Group_Length);

	for(int i = 0 ; i < Linear_Motor_Handler_Group_Length ; i++)
	{
		User_UART_STLINK_printf("Dynamixel 2_%d Pos : %d \r\n " , i , Present_Pos_32t[i] );
	}
}










/*--------------------------------------------------*/
/*
	�������ã���ʼ�� Dynamixel �� ͬ��д�� , ���Ը�ÿ��������ò�ͬ��Dirve Mode
	��������Dynamixel_Setting_Init
	��������� Drive_Mode , Operating_Mode  , ID���� , �����ID����ĳ���
	
*/
void Dynamixel_Setting_Init(enum Drive_Mode_Statment Drive_Mode_St ,enum Operaing_Mode_Statment Operaing_Mode_St ,  uint8_t* ID_Group , uint8_t ID_Group_Length)
{
	
	//����Drive_Mode
//	//����Drive_Mode״̬ö�����͵�ֵ
//	//����ΪNormal_Mode
//  //����ΪTimeBase Mode
//	Drive_Mode_Statment1 |= Time_based_Profile;
	//����Drive_Mode
	Dynamixel_Setting_Drive_Mode( Drive_Mode_St&0xF  ,  ID_Group ,  ID_Group_Length);
	
//����Operating_Mode
	Dynamixel_Setting_Operating_Mode(Operaing_Mode_St&0xF ,  ID_Group ,  ID_Group_Length);	
	
}



/*--------------------------------------------------*/
/*
	�������ã�����Profile Accleration �� Profile Veolcity�� ͬ��д��
	��������Dynamixel_Profile_Init
	��������� ���õ� Profile Accleration �� Profile Veolcity ֵ����, ID���� , �����ID����ĳ��� 
	
*/
void Dynamixel_Profile_Init( int32_t* Profile_Accleration_Value ,int32_t* Profile_Veolcity_Value , uint8_t* ID_Group , uint8_t ID_Group_Length)
{
		
	//����Profile_Accleration ͨ����������Ϊ0��
	Dynamixel_Setting_Profile_Acceleration(Profile_Accleration_Value , ID_Group , ID_Group_Length);

	//����Profile_Veolcity , ��timebase״̬�����ǵ���Ŀ����ʱ��
  Dynamixel_Setting_Profile_Veolcity(Profile_Veolcity_Value , ID_Group , ID_Group_Length);
	
	
}



/*--------------------------------------------------*/
/*
	�������ã�ʹ��Dynamixel �� Torque_Enable �� ͬ����ȡ
	��������Dynamixel_Setting_Torque_Enable
	��������� ID���� , �����ID����ĳ���
	
*/
void Dynamixel_Set_Torque_Enable( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	//ʹ��Toruqe_Enable
	Dynamixel_Setting_Torque_Enable( Dynamixel_Torque_Enable , ID_Group , ID_Group_Length);
	
}


void Dynamixel_Set_Torque_Disable( uint8_t* ID_Group , uint8_t ID_Group_Length )
{
	//ʹ��Toruqe_Enable
	Dynamixel_Setting_Torque_Enable( Dynamixel_Torque_Disable , ID_Group , ID_Group_Length);
	
}




/*--------------------------------------------------*/
/*
	�������ã�����Goal_Position�� ͬ��д��
	��������Dynamixel_Setting_Goal_Position
	��������� ���õ� Goal_Position ֵ����, ID���� , �����ID����ĳ��� , ����ṹ�����飨��Ҫ�޸���Ŀ��Ƕ�ֵ��
	
*/
void Dynamixel_Set_Goal_Position( int32_t* Goal_Position_Value, uint8_t* ID_Group , uint8_t ID_Group_Length , Linear_Motor_Handler* Linear_Motor_Handler_Group)
{
	
		//int32_t Position_Int_Group[ID_Group_Length];// = (int32_t)(Position / Position_Map_num);
	
		//��Ŀ��Ƕ�����ṹ��
		for(int i = 0  ; i < ID_Group_Length ; i++)
		{
			Linear_Motor_Handler_Group[i].Target_Angle = ((float)Goal_Position_Value[i]) * Position_Map_num; //��������ֵת��ΪС���Ƕ�����ṹ��
			//Position_Int_Group[i] = (int32_t)(Goal_Position_Value[i] / Position_Map_num);
		}
			
		//����Goal_Position
	 Dynamixel_Setting_Goal_Position(Goal_Position_Value , ID_Group , ID_Group_Length );

}




/*----------------------------------------------------------------------*/
/*
	RS485���ջص�����
*/
/*----------------------------------------------------------------------*/
void Present_Position_Receive_Func_XH540(void)
{
		//User_UART_STLINK_printf("IN Present_Position \r\n" );
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//�����յ���ID���ĸ������
			{
				Receive_Complete_state |= (1 << 0);		//����0λ��1����ʾ��һ������յ���Ϣ
				//���µ��1�ĽǶȣ�֮���ĸ�ͬ��(�ǵ��������и���)	
				Present_Pos_32t[0] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				Linear_Motor_Handler_Group[0].Original_Angle = (float)Present_Pos_32t[0] * Position_Map_num;
//				//��ӡ����ĽǶ� ��Ҫ�������ӡ��Ӧ���ڽ��պ����������ӡ
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//�����ƶ�һλ����01 ��Ϊ 10
				Present_Pos_32t[1] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				Linear_Motor_Handler_Group[1].Original_Angle = (float)Present_Pos_32t[1] * Position_Map_num;
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Present_Pos_32t[2] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				Linear_Motor_Handler_Group[2].Original_Angle = (float)Present_Pos_32t[2] * Position_Map_num;
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Present_Pos_32t[3] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				Linear_Motor_Handler_Group[3].Original_Angle = (float)Present_Pos_32t[3] * Position_Map_num;
//				//��ӡ����ĽǶ�
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			//Rece_Com_Statement_1 = Not_Receive;	
}

void Homing_Offset_Receive_Func_XH540(void)
{
		//	User_UART_STLINK_printf("IN Homing Offset \r\n" );
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//�����յ���ID���ĸ������
			{
				Receive_Complete_state |= (1 << 0);		//����0λ��1����ʾ��һ������յ���Ϣ
				//���µ��1�ĽǶȣ�֮���ĸ�ͬ��(�ǵ��������и���)	
				Homing_Offset_Goup[0] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				//��ӡ����ĽǶ� ��Ҫ�������ӡ��Ӧ���ڽ��պ����������ӡ
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//�����ƶ�һλ����01 ��Ϊ 10
				Homing_Offset_Goup[1] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Homing_Offset_Goup[2] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Homing_Offset_Goup[3] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				//��ӡ����ĽǶ�
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}

void Profile_Veolcity_Receive_Func_XH540(void)
{
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//�����յ���ID���ĸ������
			{
				Receive_Complete_state |= (1 << 0);		//����0λ��1����ʾ��һ������յ���Ϣ
				//���µ��1�ĽǶȣ�֮���ĸ�ͬ��(�ǵ��������и���)	
				Profile_Veolcity_Goup[0] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				//��ӡ����ĽǶ� ��Ҫ�������ӡ��Ӧ���ڽ��պ����������ӡ
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//�����ƶ�һλ����01 ��Ϊ 10
				Profile_Veolcity_Goup[1] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) ); 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Profile_Veolcity_Goup[2] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Profile_Veolcity_Goup[3] = (int32_t)( ((int32_t)Rx_RS485.rx_buff_temp[9])+(((int32_t)Rx_RS485.rx_buff_temp[10])<<8)+(((int32_t)Rx_RS485.rx_buff_temp[11])<<16)+(((int32_t)Rx_RS485.rx_buff_temp[12])<<24) );
				
//				//��ӡ����ĽǶ�
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}

void Profile_Torque_Receive_Func_XH540(void)
{
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//�����յ���ID���ĸ������
			{
				Receive_Complete_state |= (1 << 0);		//����0λ��1����ʾ��һ������յ���Ϣ
				//���µ��1�ĽǶȣ�֮���ĸ�ͬ��(�ǵ��������и���)	
				Torque_Enable_Goup[0] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				//��ӡ����ĽǶ� ��Ҫ�������ӡ��Ӧ���ڽ��պ����������ӡ
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//�����ƶ�һλ����01 ��Ϊ 10
				Torque_Enable_Goup[1] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				Torque_Enable_Goup[2] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				Torque_Enable_Goup[3] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				//��ӡ����ĽǶ�
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}

//ʹ��ѭ������Moving
void Moving_Receive_Func_XH540(void)
{
			if(Rx_RS485.rx_buff_temp[4] == 0x01)		//�����յ���ID���ĸ������
			{
				Receive_Complete_state |= (1 << 0);		//����0λ��1����ʾ��һ������յ���Ϣ
				//���µ��1�ĽǶȣ�֮���ĸ�ͬ��(�ǵ��������и���)	
				is_Moving_Goup[0] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				//��ӡ����ĽǶ� ��Ҫ�������ӡ��Ӧ���ڽ��պ����������ӡ
//				User_UART_STLINK_printf("Angle1 : %f \r\n " , Linear_Motor_1.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x02)
			{
				Receive_Complete_state |= (1 << 1);		//�����ƶ�һλ����01 ��Ϊ 10
				Torque_Enable_Goup[1] = (uint8_t)Rx_RS485.rx_buff_temp[9]; 
				
//				User_UART_STLINK_printf("Angle2 : %f \r\n " , Linear_Motor_2.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x03)
			{	
				Receive_Complete_state |= (1 << 2);
				is_Moving_Goup[2] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				User_UART_STLINK_printf("Angle3 : %f \r\n " , Linear_Motor_3.Original_Angle);
			}
			if(Rx_RS485.rx_buff_temp[4] == 0x04)
			{
				Receive_Complete_state |= (1 << 3);
				is_Moving_Goup[3] = (uint8_t)Rx_RS485.rx_buff_temp[9];
				
//				//��ӡ����ĽǶ�
//				User_UART_STLINK_printf("Angle4 : %f \r\n " , Linear_Motor_4.Original_Angle);
			}		

			
			//Rece_Com_Statement_1 = Not_Receive;
}



//�ö�������պ���
void Position_Receive_Func_HuangEr(void)
{			
	if(Rx_RS485.rx_buff_temp[0] != 0x55 || Rx_RS485.rx_buff_temp[1] != 0x55)//���֡ͷ�Ƿ���ȷ
	{
		//User_UART_STLINK_printf("HuanEr Hander Error \r\n " );
		CDC_Transmit_FS("HuanEr Hander Error \r\n",23);
	}
	
	
	if(Rx_RS485.rx_buff_temp[3] != 0x07)//������ݳ���
	{
		//User_UART_STLINK_printf("HuanEr DataLength Error \r\n " );
		CDC_Transmit_FS("HuanEr DataLength Error \r\n ",28);
	}
	
	//����յ���ID
	switch(Rx_RS485.rx_buff_temp[2])
	{
		case 1:
			HuangEr_Position_Group[0] = (int16_t)(((int16_t)Rx_RS485.rx_buff_temp[5]) + ((int16_t)Rx_RS485.rx_buff_temp[6] << 8) );//�ǶȵͰ�λ �� �Ƕȸ߰�λ
			break;
		default:
			//User_UART_STLINK_printf("HuanEr DataLength Error \r\n " );
		  CDC_Transmit_FS("HuanEr DataID Error \r\n ",24);
			break;
	}
	

}





