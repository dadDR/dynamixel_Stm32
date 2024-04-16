#include "Dynamixel_basic.h"
#include "Dynamixel_Control.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "User_Delay.h"
#include "User_Usart.h"
#include "Dynamixel_Address.h"
#include "Dynamixel.h"

//����������ɱ�־λ
extern enum Receive_Complate_State Rece_Com_Statement_1;	//�°������ɱ�־λ
extern int8_t Receive_Complete_state;//����������ձ�־λ

extern uint8_t Dynamixel_ID_Group[Dynamixel_Num_XH540];

//���� is_Moving ����
extern uint8_t is_Moving_Goup[4];

//����Dynamixel����ṹ������
extern Linear_Motor_Handler Linear_Motor_Handler_Group[Dynamixel_Num_XH540];


//CRCУ��
unsigned short update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_accum = 0;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}


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
void Dynamixel_Write_OBytes( uint8_t Motor_ID, uint16_t Address , uint8_t Value)
{
		
	//uint8_t Value_Int = (uint8_t)Value;	//��ʽת����ת��Ϊ����
	
	//�����������ݵ�����																								0x74 0x00�����ַ
	uint8_t Tx_DataBuf[13] = {0xFF,0xFF,0xFD,0x00,0x00,0x07,0x00,0x03,0x74,0x00,0x00,0x00,0x00};	//�ھź͵�ʮλΪ��ַλ
	//��id��ֵ
	Tx_DataBuf[4] = Motor_ID;
	//����ַ��ֵ
	Tx_DataBuf[8] = Address & 0xFF;	//��ַ�Ͱ�λ
  Tx_DataBuf[9] = (Address>>8) & 0xFF; //��ַ�߰�λ
	
	//���Բ���
	//User_UART_STLINK_printf("High: %d  Low: %d \r\n " , Tx_DataBuf[9] , Tx_DataBuf[8]);
	
	//�����ݸ�ֵ
	Tx_DataBuf[10] = Value & 0xFF;	//�ӵ�ʮһλ��ʼΪ��ֵ
//  Tx_DataBuf[11] = *((uint8_t *)&Value_Int + 1);
//  Tx_DataBuf[12] = *((uint8_t *)&Value_Int + 2);
//  Tx_DataBuf[13] = *((uint8_t *)&Value_Int + 3);
	
	uint16_t CRC_num = update_crc(Tx_DataBuf, 11);
	
  Tx_DataBuf[11] = CRC_num & 0xFF;
  Tx_DataBuf[12] = (CRC_num>>8) & 0xFF;
	
//	//���Բ���
//	User_UART_STLINK_printf("OByte_write: " , Tx_DataBuf);
//	
//		for(int i = 0 ; i < 13 ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	
	
	RS485_Send_Data(Tx_DataBuf, 13);
  
  User_Delay_ms(Delay_Time_ms);
	
}


/*
	��������Dynamixel_Write_TBytes (������)
	�������ã���Dynamixel�ļĴ�����д��� Byte ������ , ��ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д��Ķ�λ���ݵ�ֵ(��λ ֱ����float�ض��ֽڰ�)
*/
void Dynamixel_Write_TBytes( uint8_t Motor_ID, uint16_t Address , uint16_t Value)
{
		
	//uint16_t Value_Int = (uint16_t)Value;	//��ʽת����ת��Ϊ����
	
	//�����������ݵ�����																								0x74 0x00�����ַ
	uint8_t Tx_DataBuf[14] = {0xFF,0xFF,0xFD,0x00,0x00,0x07,0x00,0x03,0x74,0x00,0x00,0x00,0x00,0x00};	//�ھź͵�ʮλΪ��ַλ
	//��id��ֵ
	Tx_DataBuf[4] = Motor_ID;
	//����ַ��ֵ
	Tx_DataBuf[8] = Address & 0xFF;	//��ַ�Ͱ�λ
  Tx_DataBuf[9] = (Address>>8) & 0xFF; //��ַ�߰�λ
	
	//���Բ���
	//User_UART_STLINK_printf("High: %d  Low: %d \r\n " , Tx_DataBuf[9] , Tx_DataBuf[8]);
	
	//�����ݸ�ֵ
	Tx_DataBuf[10] = Value & 0xFF;	//�ӵ�ʮһλ��ʼΪ��ֵ
  Tx_DataBuf[11] = (Value>>8) & 0xFF;
//  Tx_DataBuf[12] = *((uint8_t *)&Value_Int + 2);
//  Tx_DataBuf[13] = *((uint8_t *)&Value_Int + 3);
	
	uint16_t CRC_num = update_crc(Tx_DataBuf, 12);
	
  Tx_DataBuf[12] = CRC_num & 0xFF;
  Tx_DataBuf[13] = (CRC_num>>8) & 0xFF;
	
//	//���Բ���
//	User_UART_STLINK_printf("TBytes : " , Tx_DataBuf);
//	
//	for(int i = 0 ; i < 14 ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	
	RS485_Send_Data(Tx_DataBuf, 14);
  
  User_Delay_ms(Delay_Time_ms);
	
}


/*
	��������Dynamixel_Write_FBytes (�����ԣ����̿���)
	�������ã���Dynamixel�ļĴ�����д���� Byte ������ , ��ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д�����λ���ݵ�ֵ(��λ һ����float?)
*/
void Dynamixel_Write_FBytes( uint8_t Motor_ID, uint16_t Address , int32_t Value)
{
		
	//int32_t Value_Int = (int32_t)Value;	//��ʽת����ת��Ϊ����
	
	//�����������ݵ�����																								0x74 0x00�����ַ
	uint8_t Tx_DataBuf[16] = {0xFF,0xFF,0xFD,0x00,0x00,0x09,0x00,0x03,0x74,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	//�ھź͵�ʮλΪ��ַλ
	//��id��ֵ
	Tx_DataBuf[4] = Motor_ID;
	//����ַ��ֵ
//	Tx_DataBuf[8] = *((uint8_t *)&Address + 1);	//��ַ�Ͱ�λ
//  Tx_DataBuf[9] = *((uint8_t *)&Address + 0); //��ַ�߰�λ
	
	Tx_DataBuf[8] = Address & 0xFF;	//��ַ�Ͱ�λ
  Tx_DataBuf[9] = (Address>>8) & 0xFF; //��ַ�߰�λ
		
	
	//���Բ���
	//User_UART_STLINK_printf("High: %d  Low: %d \r\n " , Tx_DataBuf[9] , Tx_DataBuf[8]);
	
	//�����ݸ�ֵ
	Tx_DataBuf[10] = Value & 0xFF;	//�ӵ�ʮһλ��ʼΪ��ֵ
  Tx_DataBuf[11] = (Value>>8) & 0xFF;
  Tx_DataBuf[12] = (Value>>16) & 0xFF;
  Tx_DataBuf[13] = (Value>>24) & 0xFF;
	
	uint16_t CRC_num = update_crc(Tx_DataBuf, 14);
	
  Tx_DataBuf[14] = CRC_num & 0xFF;
  Tx_DataBuf[15] = (CRC_num>>8) & 0xFF;
	
//	//���Բ���
//	User_UART_STLINK_printf("FBytes : " );
//	
//	for(int i = 0 ; i < 16 ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	
	RS485_Send_Data(Tx_DataBuf, 16);
  
  User_Delay_ms(Delay_Time_ms);
	
}

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
void Dynamixel_Write_Sync_OBytes( uint16_t Address ,uint8_t ID_num ,uint8_t* Value_Group  , uint8_t* ID_Group )
{
//	uint8_t Value_Int_Group[ID_num];
	uint8_t Value_Int;
	//��ʽת����ת��Ϊ����
//	for(int i = 0 ; i < ID_num ; i++)
//	{
//		Value_Int_Group[i] = Value[i];
//	}
	
		
  uint8_t Tx_DataBuf[100] = {0xFF,0xFF,0xFD,0x00,0xFE,0x00,0x00,0x83,0x40,0x00,0x01,0x00};
	
  uint8_t ID = 0;
  uint8_t ID_count = 0;

//  va_list arg_ptr;
//  va_start(arg_ptr, ID_num);

  Tx_DataBuf[5] = (7 + ID_num*2) & 0xFF;			//���ȵͰ�λ(����5����Ϊ��һ��id����4������λ) , //��7��ԭ��һ��INSTλ��P1-P4 �� ����CRCλ
  Tx_DataBuf[6] = ((7 + ID_num*2)>>8) & 0xFF;	//���ȸ�8λ
	
	//����ַ��ֵ
	Tx_DataBuf[8] = Address & 0xFF;	//��ַ�Ͱ�λ
  Tx_DataBuf[9] = (Address>>8) & 0xFF; //��ַ�߰�λ
	
	//�������ݳ���(��λ)
	Tx_DataBuf[10] = 0x01;
  Tx_DataBuf[11] = 0x00;		//����P3 �� P4 �� ��Ҫ��ȡ������λ����Byte�� , P3 �ǳ��ȵĵͰ�λ��P4�ǳ��ȵĸ߰�λ
	

	//�ֱ��������
  for(ID_count = 0; ID_count < ID_num; ID_count++)
  {
    ID = ID_Group[ID_count];
		Value_Int = Value_Group[ID_count];
		
    Tx_DataBuf[12 + ID_count*2] = ID & 0xFF;		//����ID 
		//���ò���
		Tx_DataBuf[12 + ID_count*2 + 1] = Value_Int & 0xFF;
//		Tx_DataBuf[12 + ID_count*2 + 2] = *((uint8_t *)&Value_Int + 1);
//		Tx_DataBuf[12 + ID_count*5 + 3] = *((uint8_t *)&Value_Int + 2);
//		Tx_DataBuf[12 + ID_count*5 + 4] = *((uint8_t *)&Value_Int + 3);
  }
	
	uint16_t CRC_num = update_crc(Tx_DataBuf, 12 + ID_num*2);
	
	Tx_DataBuf[12 + ID_num*2] = CRC_num & 0xFF;
  Tx_DataBuf[13 + ID_num*2] = (CRC_num>>8) & 0xFF;
	
//	//���Բ���
//	User_UART_STLINK_printf("TBytes_Sync_Write : " , Tx_DataBuf);
//	
//	for(int i = 0 ; i < 14 + ID_num*2 ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	
	
	RS485_Send_Data(Tx_DataBuf, 14 + ID_num*2);
	
	User_Delay_ms(Delay_Time_ms);
	
}





/*
	��������Dynamixel_Write_Sync_TBytes (������)
	�������ã���Dynamixel�ļĴ�����д��� Byte ������ , ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д��Ķ�λ���ݵ�ֵ(��һ��float����)   ,  ID������ ,
*/
void Dynamixel_Write_Sync_TBytes( uint16_t Address ,uint8_t ID_num ,uint16_t * Value_Group  , uint8_t* ID_Group )
{
//	uint16_t Value_Int_Group[ID_num];
	uint16_t Value_Int;
//	//��ʽת����ת��Ϊ����
//	for(int i = 0 ; i < ID_num ; i++)
//	{
//		Value_Int_Group[i] = Value[i];
//	}
	
		
  uint8_t Tx_DataBuf[100] = {0xFF,0xFF,0xFD,0x00,0xFE,0x00,0x00,0x83,0x40,0x00,0x01,0x00};
	
  uint8_t ID = 0;
  uint8_t ID_count = 0;


  Tx_DataBuf[5] = (7 + ID_num*3) & 0xFF;			//���ȵͰ�λ(����5����Ϊ��һ��id����4������λ) , //��7��ԭ��һ��INSTλ��P1-P4 �� ����CRCλ
  Tx_DataBuf[6] = ((7 + ID_num*3)>>8) & 0xFF;	//���ȸ�8λ
	
	//����ַ��ֵ
	Tx_DataBuf[8] = Address & 0xFF;	//��ַ�Ͱ�λ
  Tx_DataBuf[9] = (Address>>8) & 0xFF; //��ַ�߰�λ
	
	//�������ݳ���(��λ)
	Tx_DataBuf[10] = 0x02;
  Tx_DataBuf[11] = 0x00;		//����P3 �� P4 �� ��Ҫ��ȡ������λ����Byte�� , P3 �ǳ��ȵĵͰ�λ��P4�ǳ��ȵĸ߰�λ
	

	//�ֱ��������
  for(ID_count = 0; ID_count < ID_num; ID_count++)
  {
    ID = ID_Group[ID_count];
		Value_Int = Value_Group[ID_count];
		
    Tx_DataBuf[12 + ID_count*3] = ID & 0xFF;		//����ID 
		//���ò���
		Tx_DataBuf[12 + ID_count*3 + 1] = Value_Int & 0xFF;
		Tx_DataBuf[12 + ID_count*3 + 2] = (Value_Int >> 8) & 0xFF;
//		Tx_DataBuf[12 + ID_count*5 + 3] = *((uint8_t *)&Value_Int + 2);
//		Tx_DataBuf[12 + ID_count*5 + 4] = *((uint8_t *)&Value_Int + 3);
  }
	
	uint16_t CRC_num = update_crc(Tx_DataBuf, 12 + ID_num*3);
	
	Tx_DataBuf[12 + ID_num*3] = CRC_num & 0xFF;
  Tx_DataBuf[13 + ID_num*3] = (CRC_num>>8) & 0xFF;
	
//	//���Բ���
//	User_UART_STLINK_printf("TBytes_Sync_Write : " , Tx_DataBuf);
//	
//	for(int i = 0 ; i < 14 + ID_num*3 ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	
	RS485_Send_Data(Tx_DataBuf, 14 + ID_num*3);
	
	User_Delay_ms(Delay_Time_ms);
	
}



/*
	��������Dynamixel_Write_Sync_FBytes (������ -> �������ԣ����̿���)
	�������ã���Dynamixel�ļĴ�����д���� Byte ������ , ʹ�� Syncģʽ
	�����������ʼ��ַ Address �� д�����λ���ݵ�ֵ(��һ��float����)   , ID������ ,
*/
void Dynamixel_Write_Sync_FBytes( uint16_t Address ,uint8_t ID_num ,int32_t* Value  , uint8_t* ID_Group )
{
//	int32_t Value_Int_Group[ID_num];
  	int32_t Value_Int;
//	//��ʽת����ת��Ϊ����
//	for(int i = 0 ; i < ID_num ; i++)
//	{
//		Value_Int_Group[i] = (int32_t)Value[i];
//	}
	
		
  uint8_t Tx_DataBuf[100] = {0xFF,0xFF,0xFD,0x00,0xFE,0x00,0x00,0x83,0x40,0x00,0x01,0x00};
	
  uint8_t ID = 0;
  uint8_t ID_count = 0;

//  va_list arg_ptr;
//  va_start(arg_ptr, ID_num);

  Tx_DataBuf[5] = (7 + ID_num*5) & 0xFF;			//���ȵͰ�λ(����5����Ϊ��һ��id����4������λ) , //��7��ԭ��һ��INSTλ��P1-P4 �� ����CRCλ
  Tx_DataBuf[6] = ((7 + ID_num*5)>>8) & 0xFF;	//���ȸ�8λ
	
	//����ַ��ֵ
	Tx_DataBuf[8] = Address & 0xFF;	//��ַ�Ͱ�λ
  Tx_DataBuf[9] = (Address>>8) & 0xFF; //��ַ�߰�λ
	
	//�������ݳ���(��λ)
	Tx_DataBuf[10] = 0x04;
  Tx_DataBuf[11] = 0x00;		//����P3 �� P4 �� ��Ҫ��ȡ������λ����Byte�� , P3 �ǳ��ȵĵͰ�λ��P4�ǳ��ȵĸ߰�λ
	

	//�ֱ��������
  for(ID_count = 0; ID_count < ID_num; ID_count++)
  {
    ID = ID_Group[ID_count];
		Value_Int = Value[ID_count];
		
    Tx_DataBuf[12 + ID_count*5] = ID & 0xFF;		//����ID 
		//���ò���
		Tx_DataBuf[12 + ID_count*5 + 1] = Value_Int & 0xFF;
		Tx_DataBuf[12 + ID_count*5 + 2] = (Value_Int >> 8) & 0xFF;
		Tx_DataBuf[12 + ID_count*5 + 3] = (Value_Int >> 16) & 0xFF;
		Tx_DataBuf[12 + ID_count*5 + 4] = (Value_Int >> 24) & 0xFF;
  }
	
	uint16_t CRC_num = update_crc(Tx_DataBuf, 12 + ID_num*5);
	
	Tx_DataBuf[12 + ID_num*5] = CRC_num & 0xFF;
  Tx_DataBuf[13 + ID_num*5] = (CRC_num>>8) & 0xFF;
	
//	//���Բ���
//	User_UART_STLINK_printf("FBytes_Sync_Write : " , Tx_DataBuf);
//	
//	for(int i = 0 ; i < 14 + ID_num*5 ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	
	
	RS485_Send_Data(Tx_DataBuf, 14 + ID_num*5);
	
	User_Delay_ms(Delay_Time_ms);
	
}



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
	�������� �� Dynamixel_Read_FuncFByte(�������ԣ����̿���)
	�������� �� ͬ����ȡ�Ĵ��������� , ���ݽ���ΪRS485���ݷ��ͻ���
	��������ֵ������������������ͬʱ��������IDֵ , ������Ҫ��ȡ�ļĴ�����ַ , ������Ҫ��ȡ�����ݳ���(��ȡ�೤������) , ��Ҫ��ȡ�ĵ��ID������
	������
*/  
void Dynamixel_Sync_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group )
{
	uint8_t Tx_DataBuf[100] = {0xFF,0xFF,0xFD,0x00,0xFE,0x00,0x00,0x82,0x84,0x00,0x04,0x00};	//ǰ�ĸ��ֽ��ǹ̶��ģ��ֱ�����ʼ��ַ�ĸߵ�λ�����ݳ��ȸߵ�λ

	uint8_t ID = 0;
  uint8_t ID_count = 0;	
	
	//����ID������
  Tx_DataBuf[5] = (7 + ID_num) & 0xFF;
  Tx_DataBuf[6] = ((7 + ID_num)>>8) & 0xFF;		//Len 1 �������壺���ȵĵ�8λ �� Len2 ���ȸ�8λ
	
  Tx_DataBuf[8] = (Address) & 0xFF;
  Tx_DataBuf[9] = (Address>>8) & 0xFF;		//����P1 �� P2 �� ��Ҫ��ȡ�ĵ�ַλ�� , P1 �ǵ�ַ�ĵͰ�λ��P2�ǵ�ַ�ĸ߰�λ
	
  Tx_DataBuf[10] = (Data_length) & 0xFF;
  Tx_DataBuf[11] = (Data_length>>8) & 0xFF;		//����P3 �� P4 �� ��Ҫ��ȡ������λ����Byte�� , P3 �ǳ��ȵĵͰ�λ��P4�ǳ��ȵĸ߰�λ
	
  for(ID_count = 0; ID_count < ID_num; ID_count++)
  {
		ID = ID_Group[ID_count];
		
    Tx_DataBuf[12 + ID_count] = ID & 0xFF;	//�����ID�ź���ӽ�������
  }

  uint16_t CRC_num = update_crc(Tx_DataBuf, 12 + ID_num);		//����CRCУ��

  Tx_DataBuf[12 + ID_num] = CRC_num & 0xFF;									//��CRCУ��������
  Tx_DataBuf[13 + ID_num] = (CRC_num>>8) & 0xFF;						//��CRCУ��������

//	//���Բ���
//	User_UART_STLINK_printf("Read_Sync : " , Tx_DataBuf);	
//	
//	for(int i = 0 ; i < 14 + ID_num ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	

  RS485_Send_Data(Tx_DataBuf, 14 + ID_num);									//���͵ĳ���Ϊ14+ID������
	
	//���յ����ݱ�־λ�ö�Ӧ����
//	Rece_Com_Statement_1 = Present_Position_Receive; //����־λ��Ϊ�յ�Present Position
	Receive_Complete_state = 0;	//����������״̬
		
	//���߹�����ʱ
	User_Delay_ms(Delay_Time_ms);
}





/*--------------------------------------------*/
/*
	�������� �� Dynamixel_Sync_Fast_Read(�������ԣ����̿���)
	�������� �� ͬ����ȡ�Ĵ��������� , ���ݽ���ΪRS485���ݷ��ͻ���
	��������ֵ������������������ͬʱ��������IDֵ , ������Ҫ��ȡ�ļĴ�����ַ , ������Ҫ��ȡ�����ݳ���(��ȡ�೤������) , ��Ҫ��ȡ�ĵ��ID������
	������
*/  
void Dynamixel_Sync_Fast_Read(uint16_t Address, uint8_t Data_length ,uint16_t ID_num,  uint8_t* ID_Group )
{
	uint8_t Tx_DataBuf[100] = {0xFF,0xFF,0xFD,0x00,0xFE};	//ǰ�ĸ��ֽ��ǹ̶��ģ��ֱ�����ʼ��ַ�ĸߵ�λ�����ݳ��ȸߵ�λ

	uint8_t ID = 0;
  uint8_t ID_count = 0;	
	
	//����ID������
	//Len1
  Tx_DataBuf[5] = (7 + ID_num) & 0xFF;
	//Len2
  Tx_DataBuf[6] = ((7 + ID_num)>>8) & 0xFF;		//Len 1 �������壺���ȵĵ�8λ �� Len2 ���ȸ�8λ
	
	//����INST
	Tx_DataBuf[7] = 0x8A;
	
  Tx_DataBuf[8] = (Address) & 0xFF;
  Tx_DataBuf[9] = (Address>>8) & 0xFF;		//����P1 �� P2 �� ��Ҫ��ȡ�ĵ�ַλ�� , P1 �ǵ�ַ�ĵͰ�λ��P2�ǵ�ַ�ĸ߰�λ
	
  Tx_DataBuf[10] = (Data_length) & 0xFF;
  Tx_DataBuf[11] = (Data_length>>8) & 0xFF;		//����P3 �� P4 �� ��Ҫ��ȡ������λ����Byte�� , P3 �ǳ��ȵĵͰ�λ��P4�ǳ��ȵĸ߰�λ
	
  for(ID_count = 0; ID_count < ID_num; ID_count++)
  {
		ID = ID_Group[ID_count];
		
    Tx_DataBuf[12 + ID_count] = ID & 0xFF;	//�����ID�ź���ӽ�������
  }

  uint16_t CRC_num = update_crc(Tx_DataBuf, 12 + ID_num);		//����CRCУ��

  Tx_DataBuf[12 + ID_num] = CRC_num & 0xFF;									//��CRCУ��������
  Tx_DataBuf[13 + ID_num] = (CRC_num>>8) & 0xFF;						//��CRCУ��������

//	//���Բ���
//	User_UART_STLINK_printf("Read_Sync_Fast : " , Tx_DataBuf);	
//	
//	for(int i = 0 ; i < 14 + ID_num ; i++)
//	{
//		User_UART_STLINK_printf(" %02X " , Tx_DataBuf[i] );
//	}
//	
//	User_UART_STLINK_printf("\r\n " );
	

  RS485_Send_Data(Tx_DataBuf, 14 + ID_num);									//���͵ĳ���Ϊ14+ID������
	
	//���߹�����ʱ
	User_Delay_ms(Delay_Time_ms);
}

//���ͬ��д��ǶȲ���(������)
//ת����ͬһ���Ƕȣ�ֻ����һ���Ƕ�
//ͬʱ�ڶ������ĽǶȼĴ�����д��Ƕ�
void Sync_Set_Motor_Position(float Position , uint16_t ID_num, ...)
{
	int32_t Position_Int = (int32_t)(Position / Position_Map_num);
	//LEN1 LEN2 ��ʾ����ֵ����LEN2����ֽ�����
	//												   �����̶�λ      RSRV ��ID   LEN1  LEN2  
	uint8_t Tx_DataBuf[100] = {0xFF ,0xFF,	0xFD,	0x00,	0xFE,	0x11,	0x00,	0x83,	0x74,	0x00,	0x04,	0x00, 0x01,	0x82,	0x87};
	
  uint8_t ID = 0;
  uint8_t ID_count = 0;		
	
	va_list arg_ptr;						//���岻�������б��ָ�����
  va_start(arg_ptr, ID_num);	//�Թ̶������ĵ�ַΪ���ȷ����ε��ڴ���ʼ��ַ , ID_num ���Ǵ������Ĳ����б�ĵ�һ������

	//����LEN1 �� LEN2
  Tx_DataBuf[5] = (7 + ID_num*4) & 0xFF;
  Tx_DataBuf[6] = ((7 + ID_num*4)>>8) & 0xFF;		//Len 1 �������壺���ȵĵ�8λ �� Len2 ���ȸ�8λ

	//��ӦID���ĵĲ���
  for(ID_count = 0; ID_count < ID_num; ID_count++)
  {
    ID = va_arg(arg_ptr, int);	//�õ���һ���ɱ������ֵ , ��ID��������

    Tx_DataBuf[12 + ID_count*5 - 4] = ID & 0xFF;	//�����ID�ź���ӽ�������
		//��λ��ֵ��ӽ�������
		Tx_DataBuf[12 + ID_count*5 - 3] = *((uint8_t *)&Position_Int + 0);	//�ӵ�ʮһλ��ʼΪλ��ֵ
		Tx_DataBuf[12 + ID_count*5 - 2] = *((uint8_t *)&Position_Int + 1);
		Tx_DataBuf[12 + ID_count*5 - 1] = *((uint8_t *)&Position_Int + 2);
		Tx_DataBuf[12 + ID_count*5 - 0] = *((uint8_t *)&Position_Int + 3);

  }

  va_end(arg_ptr);							//va_start ����ʹ�ã������ƺ�

  uint16_t CRC_num = update_crc(Tx_DataBuf, 12 + ID_num*5);		//����CRCУ��

  Tx_DataBuf[12 + ID_num*5] = CRC_num & 0xFF;									//��CRCУ��������
  Tx_DataBuf[13 + ID_num*5] = (CRC_num>>8) & 0xFF;						//��CRCУ��������

  RS485_Send_Data(Tx_DataBuf, 14 + ID_num*5);									//���͵ĳ���Ϊ14+ID������
	
	User_Delay_ms(Delay_Time_ms);

}


//�����Ȧ�Ƕȿ���
//��λ�üĴ�����д��λ��ֵ
//4096ΪһȦ
//��չλ��ģʽ�¿������볬��һȦ��ֵ����ͨλ��ģʽ��ֻ�����벻����һȦ��ֵ
//�˺�����������չλ��ģʽ��������
void Set_Motor_Position(uint8_t Motor_ID, float Position)
{
  int32_t Position_Int = (int32_t)(Position / Position_Map_num);

  uint8_t Tx_DataBuf[16] = {0xFF,0xFF,0xFD,0x00,0x00,0x09,0x00,0x03,0x74,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	//�ھź͵�ʮλΪ��ַλ

  Tx_DataBuf[4] = Motor_ID;

  Tx_DataBuf[10] = *((uint8_t *)&Position_Int + 0);	//�ӵ�ʮһλ��ʼΪλ��ֵ
  Tx_DataBuf[11] = *((uint8_t *)&Position_Int + 1);
  Tx_DataBuf[12] = *((uint8_t *)&Position_Int + 2);
  Tx_DataBuf[13] = *((uint8_t *)&Position_Int + 3);

  uint16_t CRC_num = update_crc(Tx_DataBuf, 14);

  Tx_DataBuf[14] = CRC_num & 0xFF;
  Tx_DataBuf[15] = (CRC_num>>8) & 0xFF;

	RS485_Send_Data(Tx_DataBuf, 16);
  
  User_Delay_ms(Delay_Time_ms);
}



/*---------------------------------*/
//���ݶ�ȡ����������д�뺯����ͬʱ����
//����Dynamixel �����ϵĽ̳�
//���ڸ���ΪDynamixel�ĳ�ʼλ�˺���
//Ч��Ϊת����ָ��λ�� �� ���ȴ��˶���ɣ�(���������Ҫ����)
void Dynamixel_Basic_Func_Pose(void)
{
		//����������趨��ʼλ�ˣ���λΪ�ȣ�
		//float Dynamixel_Goal_Position[4] = { -120 , 120 , -120 , 120 };		//��ʼλ�ã�ȫΪ120�ȣ�//�ڰ�����
		float Dynamixel_Goal_Position[4] = { -65 , 65 , -65 , 65};//�������ƣ�
		//float Dynamixel_Goal_Position[4] = { -85 , 85 , -85 , 85};//��ֱ��������
		
		
		
		
		//���Ƕ�ֵλ��ת��Ϊ�����4096
		int32_t Dynamixel_Goal_Int[4];
		for(int i = 0 ; i < 4 ; i++)
		{
			Dynamixel_Goal_Int[i] = (int32_t)(Dynamixel_Goal_Position[i] / Position_Map_num);
		}
			
		int32_t Dynamixel_Test_Veol[4] = {0 , 0 , 0 , 0}; 
		
		//����DriveMode �� Operating Mode(������ʽ)
		enum Drive_Mode_Statment	Drive_Mode_Test = Inital_Mode | Time_based_Profile_Mode;
		enum Drive_Mode_Statment	Drive_Mode_Test2 = Inital_Mode | Time_based_Profile_Mode;
		enum Operaing_Mode_Statment Operaing_Mode_Test = Extended_Position_Control_Mode;
		//��ʼ��Drive_Mode
		enum Drive_Mode_Statment Dreve_Mode_Test_Group[4] = {Inital_Mode | Time_based_Profile_Mode  ,
																												  Inital_Mode | Time_based_Profile_Mode | Reverse_Mode ,
																												   Inital_Mode | Time_based_Profile_Mode ,
																												    Inital_Mode | Time_based_Profile_Mode| Reverse_Mode};
		
	//Dynamixel_Sync_Read(Present_Position, 4 ,2,  ID_Group );//��ȡλ��ָ������Ժ���ã�����Ӧ�ó��ڻص������� 
	//��ӡ��Ҫ������
	//����Ϊ��չλ��ģʽ �� ʱ�� base ģʽ
	Dynamixel_Setting_Init(Drive_Mode_Test , Operaing_Mode_Test ,  Dynamixel_ID_Group , 4);
	//���������
	Dynamixel_Setting_Zero( Linear_Motor_Handler_Group , 4);
		//����Profile Accleration �� Profile Velocity
		//����Profile_Veolcity , ��timebase״̬�����ǵ���Ŀ����ʱ��
	Dynamixel_Setting_Profile_Veolcity(Dynamixel_Test_Veol , Dynamixel_ID_Group , 4);
	//ʹ������
	Dynamixel_Set_Torque_Enable(  Dynamixel_ID_Group , 4 );
	//ת����ָ���ĽǶ�
	Dynamixel_Set_Goal_Position( Dynamixel_Goal_Int, Dynamixel_ID_Group , 4 , Linear_Motor_Handler_Group);
//	//�ȴ��˶����
//	while(is_Moving_Goup[0] == 0 && is_Moving_Goup[1] == 0 && is_Moving_Goup[2] == 0 && is_Moving_Goup[3] == 0)
//	{
//		Dynamixel_Read_Moving(Dynamixel_ID_Group , 4);
//		User_Delay_ms(1);
//	}
	while(is_Moving_Goup[0] == 0 && is_Moving_Goup[1] == 0 && is_Moving_Goup[2] == 0 && is_Moving_Goup[3] == 0)
	{
		//��ӡ��ֵ
		Dynamixel_Read_Moving(Dynamixel_ID_Group , 4);
		User_Delay_ms(1);
		User_UART_STLINK_printf("isMove1:%d ,isMove2:%d ,isMove3:%d ,isMove4:%d ,  \r\n" ,is_Moving_Goup[0] , is_Moving_Goup[1] , is_Moving_Goup[2] , is_Moving_Goup[3] );
	}
}






