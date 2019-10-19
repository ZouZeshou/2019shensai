#include "drv_uart.h"
#include "usart.h"
#include "STMGood.h"
uint8_t Usart1buff[100]={0};
uint8_t Usart2buff[100]={0};
uint8_t Usart3buff[100]={0};
uint8_t Uart4buff[100]={0};
uint8_t Usart6buff[100]={0};
/**
 * @brief Enable USART
 * @param None
 * @return None
 * @attention None
 */
void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr)
{
	HAL_UART_Receive_IT(huart,buffer_addr,1);
	__HAL_UART_ENABLE_IT(huart,UART_IT_ERR);	
}
/**
* @brief usart send char
* @param argument: Not used
* @retval None
*/
void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char)
{
		while((huart->Instance->SR&0X40)==0); 
		huart->Instance->DR = u8_char;
//	HAL_UART_Transmit_IT(&huart1,&u8_char,1);
}
/**
 * @brief Enable the Usart DMA
 * @param None
 * @return None
 * @attention  None
 */
void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size)
{
	 HAL_DMA_Start(hdma,(uint32_t)huart->Instance->DR, (uint32_t)buffer_addr,data_size);
	 huart->Instance->CR3 |= USART_CR3_DMAR;								/*!<DMA Enable Receiver         */
	 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);								/*!<ʹ�ܴ��ڵ��ж�Ϊ�����ж�    */
	 HAL_UART_Receive_DMA(huart,buffer_addr,data_size);								/*!<DMA Receive data            */
	 __HAL_UART_ENABLE_IT(huart,UART_IT_ERR);								/*!<Enable Usart Error IT      	*/
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART_IDLE_IRQ(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t data_size)
{
	
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(huart);	//�����־λ��SR��DR�Ĵ���
			HAL_UART_DMAStop(huart);
			HAL_UART_Receive_DMA(huart,buffer_addr,data_size);//�����а�����������DMA
	}
}

/**
 * @brief Error Callback function�������жϻص�������
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->ErrorCode == HAL_UART_ERROR_ORE)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //��������־λ�����SR��DR�Ĵ���
	}
}
/**
 * @brief Redirect function for printf����printf�������ض��壩
 * @param None
 * @return None
 * @attention  The printf function could not be usedwithout this function
 */
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0); 
	USART3->DR = (uint8_t) ch;      
	return ch;
}

/**
 * @brief rx callbackfunction  �����ڽ����жϻص�������
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		Dealdata(Usart3buff[0]);//�����ʽ������λ���ĸ�ʽ�������Ӧ�������и�ֵ��P��I��D��
		//printf("receive\r\n");
		__HAL_UART_CLEAR_PEFLAG(&huart3);//����жϱ�־λ
		HAL_UART_Receive_IT(&huart3,Usart3buff,1);//ʹ�ܴ���3
	}
}