#include "bsp_uart.h"
#include "gimbal_task.h"
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "usart.h"
#include "vision_task.h"
#include <math.h>

//union Receive_data{int get_data[2];char char_data[8];}receive_data;
uint8_t receive_data[40];
union Send_Data{int data[12];char char_data[8];} send_data;
uint8_t start_receive_flag = 0;

short int F_move_flag=0;		//上位机控制底盘移动的标识符，上位机用来测试的
short int L_move_flag=0;
short int TIME_;
float VISION_SHOOT_TIME=0.0f;
static int i = 0;
float a,b = 0;

uint8_t ch;
int int_get[2];
int iii=-1;
uint8_t dma_rx_buff[40];

void uart_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN); 	
}

uint8_t length;
int temp_1;
void USART1_IRQHandler(void)
{
	length=0;
	int j;
	 if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	 {
		 __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		 HAL_UART_DMAStop(&huart1);
		 length=DMA_REC_LEN-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		 
		 for(j=0;i<length;j++)
		 {
			 if(dma_rx_buff[j] == ';')//结束接收
				{		
					temp_1=0;
					for(int k=0;k<i;k++)
					{
						temp_1+=(receive_data[k]-'0')*pow(10,i-k-1);
					}
					vision_sent.yaw.target_angle =(float)temp_1/100.0f-360.0f;
					memset(receive_data,0,sizeof(receive_data));
					start_receive_flag = 0;
					i = 0;

				}
				else if(dma_rx_buff[j] == '%')//结束接收
				{
					temp_1=0;
					for(int k=0;k<i;k++)
					{
						temp_1+=(receive_data[k]-'0')*pow(10,i-k-1);
					}
					vision_sent.pitch.target_angle =(float)temp_1/100.0f-180.0f;
					
					memset(receive_data,0,sizeof(receive_data));
					start_receive_flag = 0;
					i = 0;
					break;
				}
			 if(start_receive_flag == 1)   //进行数据转移
      { 
				if(i<length)
				{
					receive_data[i]=dma_rx_buff[j];
				  i++;
				}

	    }
	    if((dma_rx_buff[j] == '*')||(dma_rx_buff[j] == '#'))//开始接收
	    {
		    start_receive_flag = 1;
	    }
		 }
		 memset(dma_rx_buff,0,sizeof(dma_rx_buff));
		 HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
	 }
}

//void USART1_IRQHandler(void)
//{
//	length=0;
//	int j;
//	 if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
//	 {
//		 __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//		 HAL_UART_DMAStop(&huart1);
//		 length=DMA_REC_LEN-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
//		 for(j=0;i<length;j++)
//		 {
//			 if(dma_rx_buff[j] == ';')//结束接收
//				{		
//					vision_sent.yaw.target_angle =(int16_t)(receive_data[1]<<8|receive_data[0])/100.0f;
//					vision_sent.pitch.target_angle =(int16_t)(receive_data[3]<<8|receive_data[2])/100.0f;

//					start_receive_flag = 0;
//					i = 0;
//					break;
//				}
//			 if(start_receive_flag == 1)   //进行数据转移
//      { 
//				if(i<9)
//				{
//					receive_data[i]=dma_rx_buff[j];
//				  i++;
//				}
//	    }
//	    if(dma_rx_buff[j] == '*')//开始接收
//	    {
//		    start_receive_flag = 1;
//	    }

//		 }
//		 HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
//	 }
//}

uint8_t Buffer[8],status;
VISION_RESET VISION_RESET_FLAG=OFF;
int Sent_dataA = 0;
int Sent_dataB = 0;
int Sent_dataC = 0;
int Sent_dataD = 0;

void DMA_Send(void)
{ 
	Sent_dataA=100*gimbal_y.IMU_actual_angle;
	Sent_dataB=100*gimbal_p.IMU_actual_angle;
	Sent_dataC=VISION_RESET_FLAG; //重启

	
	Buffer[0] = '*';
	Buffer[2] = (Sent_dataA>>8);
	Buffer[1] =  Sent_dataA&0xff;
	Buffer[4] = (Sent_dataB>>8);
	Buffer[3] =  Sent_dataB&0xff;
	Buffer[6] = (Sent_dataC>>8);
	Buffer[5] =  Sent_dataC&0xff;
//


//	  for(int i=7;i<13;i++)
//	{
//	  Buffer[i] = 0;
//	}

//  for(int i=1;i<(7-1);i++)
//	{
//	  Buffer[i] = send_data.data[i-1];
//	}
	Buffer[7] = ';';
	status=HAL_UART_Transmit(&huart1,Buffer,8,0xff);

}
