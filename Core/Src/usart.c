#include "usart.h"
#include <stdio.h>
#include "cmsis_os.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
#define BUF_MIN(x, y)                   ((x) < (y) ? (x) : (y))
#define BUF_MAX(x, y)                   ((x) > (y) ? (x) : (y))
/* Nonzero if either X or Y is not aligned on a "long" boundary.  */
#define UNALIGNED(X, Y) \
  (((long)X & (sizeof (long) - 1)) | ((long)Y & (sizeof (long) - 1)))
/* How many bytes are copied each iteration of the 4X unrolled loop.  */
#define BIGBLOCKSIZE    (sizeof (long) << 2)
/* How many bytes are copied each iteration of the word copy loop.  */
#define LITTLEBLOCKSIZE (sizeof (long))
/* Threshhold for punting to the byte copier.  */
#define TOO_SMALL(LEN)  ((LEN) < BIGBLOCKSIZE)

/* Memory is in a shareable and non cacheable zone */
#define __ATTR_USART_RX_DMA   __attribute__((section(".UsartDma"), aligned(4)))
#define __ATTR_USART_TX_DMA   __attribute__((section(".UsartDma"), aligned(4)))
/* Memory is in a fast access zone : DTCM RAM */
#define __ATTR_USART_RX_DTCM   __attribute__((section(".RxUsart2Dtcm"), aligned(4)))

extern UART_HandleTypeDef huart2;
extern void Error_Handler(void);
/* Buffers used for DMA */
static volatile uint8_t UsartRxDmaBuff[USART_RX_DMA_BUFF] __ATTR_USART_RX_DMA = {0};
static volatile uint8_t UsartTxDmaBuff[USART_TX_DMA_BUFF] __ATTR_USART_TX_DMA = {0};
/* Buffer used to consume RX DMA */
static uint8_t UsartRxBuff[USART_RX_BUFF] __ATTR_USART_RX_DTCM = {0};
/* Tracks DMA received bytes position */
static uint16_t old_pos = 0;
/* Semaphore to signal UART is ready to Transmit */
static osSemaphoreId UsartTxReady = NULL;   
/* Uart RX buffer consumer callback to be defined by consumer app */
static Uart2RxCallback_t g_rxCallback = NULL;

static uint16_t usart_Recv(uint8_t* bArray, uint32_t ndtr);
static volatile void *fastmemcpy(volatile void *dst0, const volatile void *src0, size_t len0);

void usart_preInit(void)
{
  /* Init operations. Called before UART hardware is configured */
  osSemaphoreDef(UsartTxSem);
  UsartTxReady = osSemaphoreCreate(osSemaphore(UsartTxSem), 1);
}

void usart_Open(Uart2RxCallback_t rxCallback)
{
  /* Initializes Rx sequence using Reception To Idle event API.
     As DMA channel associated to UART Rx is configured as Circular,
     reception is endless.
     If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.

     Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
     user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
     following events :
     - DMA RX Half Transfer event (HT)
     - DMA RX Transfer Complete event (TC)
     - IDLE event on UART Rx line (indicating a pause is UART reception flow)
  */
  if (rxCallback == NULL)
  {
    /* Consumer app needs to define an RX CB */
    printf("You have to define an UART RX callback to consume received buffer in ISR context !\r\n");
    Error_Handler();
  }
  g_rxCallback = rxCallback;
  if (ARRAY_LEN(UsartRxBuff) != ARRAY_LEN(UsartRxDmaBuff))
  {
    /* Intermediate buffer should have the same size as RX DMA buffer in current
     design */
    printf("Different size for RX dma buffer and RX buffer is not supported !\r\n");
    Error_Handler();
  }
  old_pos = 0;
  if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(
    &huart2, (uint8_t *)UsartRxDmaBuff, ARRAY_LEN(UsartRxDmaBuff)))
  {
    Error_Handler();
  }
}

void usart_Send(uint8_t* bArray, uint32_t size_bArray)
{
  uint32_t send_size, remaining;

  remaining = size_bArray;

  while(remaining)
  {
      /* Wait for the end of current transfer */
      osSemaphoreWait(UsartTxReady, osWaitForever);
      send_size = BUF_MIN(remaining, ARRAY_LEN(UsartTxDmaBuff));
      fastmemcpy(UsartTxDmaBuff, &bArray[size_bArray - remaining], send_size);
      if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)UsartTxDmaBuff, send_size)!= HAL_OK)
      {
          Error_Handler();
      }
      remaining -= send_size;
  }
}

static uint16_t usart_Recv(uint8_t* bArray, uint32_t ndtr)
{
  size_t pos;
	uint16_t length = 0;

  /* check for new data available */
  pos = ARRAY_LEN(UsartRxDmaBuff) - ndtr;
  if (pos != old_pos) {                       /* Check change in received data */
      if (pos > old_pos) {                    /* Current position is over previous one */
          /*
           * Processing is done in "linear" mode.
           *
           * Application processing is fast with single data block,
           * length is simply calculated by subtracting pointers
           *
           * [   0   ]
           * [   1   ] <- old_pos |------------------------------------|
           * [   2   ]            |                                    |
           * [   3   ]            | Single block (len = pos - old_pos) |
           * [   4   ]            |                                    |
           * [   5   ]            |------------------------------------|
           * [   6   ] <- pos
           * [   7   ]
           * [ N - 1 ]
           */
        length = pos - old_pos;
        fastmemcpy(bArray, &UsartRxDmaBuff[old_pos], length);
      } else {
          /*
           * Processing is done in "overflow" mode..
           *
           * Application must process data twice,
           * since there are 2 linear memory blocks to handle
           *
           * [   0   ]            |---------------------------------|
           * [   1   ]            | Second block (len = pos)        |
           * [   2   ]            |---------------------------------|
           * [   3   ] <- pos
           * [   4   ] <- old_pos |---------------------------------|
           * [   5   ]            |                                 |
           * [   6   ]            | First block (len = N - old_pos) |
           * [   7   ]            |                                 |
           * [ N - 1 ]            |---------------------------------|
           */
        length = ARRAY_LEN(UsartRxDmaBuff) - old_pos;
        fastmemcpy(bArray, &UsartRxDmaBuff[old_pos], length);
        if (pos > 0) {
            fastmemcpy(&bArray[length], &UsartRxDmaBuff[0], pos);
            length += pos;
        }
      }
      old_pos = pos;                          /* Save current position as old for next transfers */
  }
	return length;
}

/**
  * @brief  User implementation of the Reception Event Callback
  *         (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  size_t rx_len;
  uint32_t ndtr;

  if((huart->RxEventType == HAL_UART_RXEVENT_IDLE) ||
     (huart->RxEventType == HAL_UART_RXEVENT_HT) ||
     (huart->RxEventType == HAL_UART_RXEVENT_TC))
  {
    /* Get current DMA last written Byte position */
    ndtr = ((DMA_Stream_TypeDef *)huart2.hdmarx->Instance)->NDTR;
    /* Fill RX intermediate buffer */
    rx_len = usart_Recv(UsartRxBuff, ndtr);
    /* Send to consumer application */
    g_rxCallback(UsartRxBuff, rx_len);
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Transfer complete : UART TX DMA is ready for next transfer */
  osSemaphoreRelease(UsartTxReady);
}

/**
  * @brief  block memcpy with volatile
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
volatile void *fastmemcpy(volatile void *dst0, const volatile void *src0, size_t len0)
{
  volatile char *dst = dst0;
  const volatile char *src = src0;
  volatile long *aligned_dst;
  const volatile long *aligned_src;

  /* If the size is small, or either SRC or DST is unaligned,
     then punt into the byte copy loop.  This should be rare.  */
  if (!TOO_SMALL(len0) && !UNALIGNED (src, dst))
    {
      aligned_dst = (volatile long*)dst;
      aligned_src = (volatile long*)src;

      /* Copy 4X long words at a time if possible.  */
      while (len0 >= BIGBLOCKSIZE)
        {
          *aligned_dst++ = *aligned_src++;
          *aligned_dst++ = *aligned_src++;
          *aligned_dst++ = *aligned_src++;
          *aligned_dst++ = *aligned_src++;
          len0 -= BIGBLOCKSIZE;
        }

      /* Copy one long word at a time if possible.  */
      while (len0 >= LITTLEBLOCKSIZE)
        {
          *aligned_dst++ = *aligned_src++;
          len0 -= LITTLEBLOCKSIZE;
        }

       /* Pick up any residual with a byte copier.  */
      dst = (volatile char*)aligned_dst;
      src = (volatile char*)aligned_src;
    }

  while (len0--)
    *dst++ = *src++;

  return dst0;
}
