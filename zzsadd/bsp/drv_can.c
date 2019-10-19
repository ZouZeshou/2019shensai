#include "drv_can.h"
#include "can.h"
#include "chassis.h"
#include "drv_uart.h"
#include "math.h"
struct s_FPS s_fps;
/**
 * @brief Enable Can1 and Can2(��can1��can2���г�ʼ��)
 * @param None
 * @return None
 * @attention None
 */
void CAN_Enable(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Start(hcan);//��can���м���
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ��can�����ж�
	CANFilter_Enable(hcan);//ʹ���˲���
}

/**
 * @brief interrupt function in IRQ��can�����жϻص�������
 * @param None
 * @return None
 * @attention None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	static uint8_t RxData1[8],RxData2[8];//�������ڽ�����Ϣ������
	CAN_RxHeaderTypeDef Can1Header,Can2Header;//������պ�����Ҫ�õ��ľ��
	//�����can1
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&Can1Header,RxData1 );//��FIFO�����ж�ȡ��Ϣ��RxData1
		//�ж�can����ID����Ӧ���
		switch(Can1Header.StdId)
		{
			case 0x201:
			{
				s_leftmotor.back_position = RxData1[0]<<8|RxData1[1];
				s_leftmotor.back_speed = RxData1[2]<<8|RxData1[3];
				if(abs(s_leftmotor.back_speed)<=5)
				{
					s_leftmotor.back_speed = 0;
				}
				s_fps.chassis_1++;
				break;
			}
			case 0x202:
			{
				s_rightmotor.back_position = RxData1[0]<<8|RxData1[1];
				s_rightmotor.back_speed = RxData1[2]<<8|RxData1[3];
				if(abs(s_rightmotor.back_speed)<=5)
				{
					s_rightmotor.back_speed = 0;
				}
				s_fps.chassis_2++;
				break;
			}
			case 0x203:
			{
				s_trans_motor.back_pos_last = s_trans_motor.back_position;
				s_trans_motor.back_position = RxData1[0]<<8|RxData1[1];
				s_trans_motor.back_speed = RxData1[2]<<8|RxData1[3];
				continue_motor_pos(&s_trans_motor);
				s_fps.trans++;
				break;
			}
		}
	}
	//�����can2
	if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0,&Can2Header,RxData2 );//��FIFO�н�����Ϣ��RxData2
		
		if(Can2Header.StdId==0x202)
		{
		}	
	}
}
/**
 * @brief Enable filter(0 for can1 and 14 for can2)�˲�����ʼ������
 * @param None
 * @return None
 * @attention None
 */
void CANFilter_Enable(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef filter1;
	CAN_FilterTypeDef filter2;
	if(hcan->Instance == CAN1)
	{
		filter1.FilterActivation=ENABLE;
		filter1.FilterBank=0U;
		filter1.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter1.FilterIdHigh=0x0000;
		filter1.FilterIdLow=0x0000;
		filter1.FilterMaskIdHigh=0x0000;
		filter1.FilterMaskIdLow=0x0000;
		filter1.FilterMode=CAN_FILTERMODE_IDMASK;
		filter1.FilterScale=CAN_FILTERSCALE_32BIT;
		filter1.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan1,&filter1);
	}
	if(hcan->Instance == CAN2)
	{
		filter2.FilterActivation=ENABLE;
		filter2.FilterBank=14;
		filter2.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter2.FilterIdHigh=0x0000;
		filter2.FilterIdLow=0x0000;
		filter2.FilterMaskIdHigh=0x0000;
		filter2.FilterMaskIdLow=0x0000;
		filter2.FilterMode=CAN_FILTERMODE_IDMASK;
		filter2.FilterScale=CAN_FILTERSCALE_32BIT;
		filter2.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan2,&filter2);
	}
	
}
/**
 * @brief Send the message by Can1��can1���ͺ�����
 * @param None
 * @return None
 * @attention None
 */
void Can_SendMsg(CAN_HandleTypeDef *hcan,uint32_t id,int16_t data1,int16_t data2,int16_t data3,int16_t data4)
{	
	CAN_TxHeaderTypeDef Txmsg1;
	uint8_t TxData1[8];
	
	Txmsg1.DLC=0x08;
	Txmsg1.IDE=CAN_ID_STD;
	Txmsg1.RTR=CAN_RTR_DATA;
	Txmsg1.StdId=id;
	
	TxData1[0]=(unsigned char)(data1>>8);
	TxData1[1]=(unsigned char)(data1);
	TxData1[2]=(unsigned char)(data2>>8);
	TxData1[3]=(unsigned char)(data2);
	TxData1[4]=(unsigned char)(data3>>8);
	TxData1[5]=(unsigned char)(data3);
	TxData1[6]=(unsigned char)(data4>>8);
	TxData1[7]=(unsigned char)(data4);
	
	HAL_CAN_AddTxMessage(hcan,&Txmsg1,TxData1,(uint32_t *)CAN_TX_MAILBOX0);
	
}

