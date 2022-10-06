#ifndef __BSP_UART_H
#define __BSP_UART_H


#define DMA_REC_LEN    20
typedef enum
{
	ON_RESET=1,
	OFF=0,
}VISION_RESET; //重启标志位


extern short int F_move_flag;
extern short int L_move_flag;
void uart_init(void);
void DMA_Send(void);
extern VISION_RESET VISION_RESET_FLAG;

extern short int TIME_;
extern float VISION_SHOOT_TIME;
#endif

