#include "drv_robotservo.h"
#include "usart.h"
#include "drv_uart.h"

//CDS5516��������
/*
֡ͷ   ID �� ָ�� ��ַ ����.. У���
FF FF  01 04  03   03   00      F4
*/
/**
 * @brief ���ö����ID��Ϊ1
 * @param None
 * @return None
 * @attention None
 */
void Set_Num_1(void) //���ö����ID��Ϊ1
{
    USART1->CR1 &= ~0x4;
    USART_Send_Char(&huart1,0xFF);
    USART_Send_Char(&huart1,0XFF);
    USART_Send_Char(&huart1,0XFE);
    USART_Send_Char(&huart1,0X04);
    USART_Send_Char(&huart1,0X03);
    USART_Send_Char(&huart1,0X03);
    USART_Send_Char(&huart1,0X01);
    USART_Send_Char(&huart1,0XF6);
    USART1->CR1 |= 0x4;
}
/**
 * @brief ���ö����ת��Ŀ��Ƕ�
 * @param None
 * @return None
 * @attention None
 */
void Set_Num_Speed(uint8_t id,uint32_t arg)      //���ö����ת��Ŀ��Ƕ�
{
    static uint8_t i = 0; 
    uint8_t sum = 0x00; 
    uint8_t dat[] = {0xFF,0xFF,0,5,0x03,0x1E,0,0,0xFF};  //����һ�����ݰ�
    dat[2] = id;                          //����ID��
    dat[6] = (arg*0x3FF/300)&0xFF;        //����Ŀ��Ƕȵĵ��ֽ�
    dat[7] = (arg*0x3FF/300)>>8;          //����Ŀ��Ƕȵĸ��ֽ�
    for(i=2;i<8;i++)
    {
        sum+=dat[i];                    //����У���
    } 
    dat[8] = ~sum;                      
    
    for(i=0;i<9;i++)
    {
        USART_Send_Char(&huart1,dat[i]);       //�����ݰ�����
    }
}



