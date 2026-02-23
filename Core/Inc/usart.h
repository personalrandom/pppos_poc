#ifndef _USART_
#define _USART_

#include "stm32h7xx_hal.h"

/* Buffers used by DMA in TX and RX, they're non cacheable 
	and shareable */
#define USART_RX_DMA_BUFF (2*1024)
#define USART_TX_DMA_BUFF (2*1024)

/* Intermediate buffer to store RX data from DMA non cacheable 
	memory to DTCM RAM */
#define USART_RX_BUFF (2*1024)

/* Function to be defined by consumer application :
  - Executes in UART/RX_DMA ISR context : so processing should be minimal */
typedef void (*Uart2RxCallback_t)(uint8_t *rx_buffer, size_t rx_len);

/* Called before Uart hardware is configured */
void usart_preInit(void);
void usart_Open(Uart2RxCallback_t rxCallback);
/* Blocking send if UART is busy. 
	Should only be used from one context.
	!! Not thread safe !! */
void usart_Send(uint8_t* bArray, uint32_t size_bArray);

#endif /* _USART_ */
