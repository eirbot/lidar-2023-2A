#ifndef __USART_H__
#define __USART_H__


#include "tools.h"
#include "mbed.h"


/* Includes ------------------------------------------------------------------*/


extern UART_HandleTypeDef huart1;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void MX_USART1_UART_Init(void);


#endif /* __USART_H__ */

