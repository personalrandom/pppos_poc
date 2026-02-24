/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include "lwrb.h"
#include "pppos.h"
#include "pppapi.h"


/* Define this buffer as much as you want to send during 1 second cycle */
#define TX_RB_BUFF_SIZE (16 * 1024)
/* Memory is in a fast access zone : DTCM RAM */
#define __ATTR_USART_TX_DTCM   __attribute__((section(".TxUsart2Dtcm"), aligned(4)))
/* Simplified design : sleep for time it take to send 2KB @1M baudrate = 21 ms */
#define WAIT_2KB_IN_MS 21

static void pppConnect(void);
static u32_t ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx);
static void UartTxTask(void const * argument);
static void UartRxCbWrb(uint8_t *rx_buffer, size_t rx_len);
static void ppp_link_status_cb(ppp_pcb *pcb, int err_code, void *ctx);

/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
static void ethernet_link_status_updated(struct netif *netif);
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */
static ppp_pcb *ppp;
static struct netif pppos_netif;
/* UART TX task : only context that uses hardware */
static osThreadId TaskHandle;
static uint32_t TaskBuffer[1024];
static osStaticThreadDef_t TaskControlBlock;
/* Ring buffer instance for TX data */
static lwrb_t usart_tx_rb;
/* Ring buffer data array for TX DMA */
static uint8_t usart_tx_rb_data[TX_RB_BUFF_SIZE] __ATTR_USART_TX_DTCM = {0};
/* Semaphore to signal to UART task new data */
static osSemaphoreId TxBuffNew = NULL;   

/* signal that connection is established */
osSemaphoreId pppconnected = NULL;
/* USER CODE END 1 */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

/* USER CODE BEGIN 2 */

static void pppConnect(void)
{
  // Todo : refactor this into usart.c ?
  /* Initialize ringbuff for TX */
  lwrb_init(&usart_tx_rb, usart_tx_rb_data, sizeof(usart_tx_rb_data));

  /* Create semaphore used for informing UartTxTask of new data */
  osSemaphoreDef(TxBuffNewSem);
  TxBuffNew = osSemaphoreCreate(osSemaphore(TxBuffNewSem), 1);

  osSemaphoreDef(pppconnectedSem);
  pppconnected = osSemaphoreCreate(osSemaphore(pppconnectedSem), 1);

  /* Decrease the semaphore's initial count from 1 to 0 */
  osSemaphoreWait(TxBuffNew, 0);
   osSemaphoreWait(pppconnected, 0);

  ppp = pppapi_pppos_create(&pppos_netif, ppp_output_cb, ppp_link_status_cb, NULL);
  pppapi_set_default(ppp);

  ip4_addr_t our_ip, his_ip;

  IP4_ADDR(&our_ip,  192,168,200,1);
  IP4_ADDR(&his_ip,  192,168,200,2);
  ppp_set_ipcp_ouraddr(ppp, &our_ip);
  ppp_set_ipcp_hisaddr(ppp, &his_ip);
  ppp_set_silent(ppp, 1);
  // ppp_set_auth_required(ppp, 0);
  /* Uart can now receive and transmit.
    If done before pppapi_pppos_create, received UART DMA will try to pppos_input
    into a non created ppp instance */
  usart_Open(UartRxCbWrb);
  osThreadStaticDef(UartTxTask, UartTxTask, osPriorityNormal, 0, sizeof(TaskBuffer)/sizeof(&TaskBuffer[0]),
   TaskBuffer, &TaskControlBlock);
  TaskHandle = osThreadCreate(osThread(UartTxTask), NULL);

  pppapi_connect(ppp,0);
}

/* 
  Blocks if intermediate TX buffer is full, which means lwip TCPIP core thread will block.
  There is no risk of losing new RX packets since they're copied to pbuff in RX IRQ context 
  To never block, define TX_RB_BUFF_SIZE to expected max burst during 1 second cycle
   */
static u32_t ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{

  LWIP_UNUSED_ARG(pcb);
  LWIP_UNUSED_ARG(ctx);
  u32_t written;
  u32_t remaining = len;

  while(remaining)
  {
    written = lwrb_write(&usart_tx_rb, data, remaining);
    osSemaphoreRelease(TxBuffNew);
    remaining -= written;
    if(remaining)
    {
      /* Sleep for duration needed to send 2KB */
      osDelay(WAIT_2KB_IN_MS);
    }
  }
  return len;
}

static void UartTxTask(void const * argument)
{

  u32_t len;

  while(1)
  {
    osSemaphoreWait(TxBuffNew, osWaitForever);
    while((len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0)
    {
      /* There is data in ring buffer */
      usart_Send(lwrb_get_linear_block_read_address(&usart_tx_rb), len);
      /* move read buffer */
      lwrb_skip(&usart_tx_rb, len);
    }
  }
}

static void UartRxCbWrb(uint8_t *rx_buffer, size_t rx_len)
{
  /* Called in IRQ context. Feed UART rx bytes into ppp */
  pppos_input(ppp, rx_buffer, rx_len);
}


static void ppp_link_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
    struct netif *pppif = ppp_netif(pcb);
    LWIP_UNUSED_ARG(ctx);

    switch(err_code)
    {
      case PPPERR_NONE:               /* No error. */
      {
        printf("ppp_link_status_cb: PPPERR_NONE\n\r");
        printf("   our_ip4addr = %s\n\r", ip4addr_ntoa(netif_ip4_addr(pppif)));
        printf("   his_ipaddr  = %s\n\r", ip4addr_ntoa(netif_ip4_gw(pppif)));
        printf("   netmask     = %s\n\r", ip4addr_ntoa(netif_ip4_netmask(pppif)));
        osSemaphoreRelease(pppconnected);
      }
      break;

      case PPPERR_PARAM:             /* Invalid parameter. */
          printf("ppp_link_status_cb: PPPERR_PARAM\n");
          break;

      case PPPERR_OPEN:              /* Unable to open PPP session. */
          printf("ppp_link_status_cb: PPPERR_OPEN\n");
          break;

      case PPPERR_DEVICE:            /* Invalid I/O device for PPP. */
          printf("ppp_link_status_cb: PPPERR_DEVICE\n");
          break;

      case PPPERR_ALLOC:             /* Unable to allocate resources. */
          printf("ppp_link_status_cb: PPPERR_ALLOC\n");
          break;

      case PPPERR_USER:              /* User interrupt. */
          printf("ppp_link_status_cb: PPPERR_USER\n");
          break;

      case PPPERR_CONNECT:           /* Connection lost. */
          printf("ppp_link_status_cb: PPPERR_CONNECT\n");
          break;

      case PPPERR_AUTHFAIL:          /* Failed authentication challenge. */
          printf("ppp_link_status_cb: PPPERR_AUTHFAIL\n");
          break;

      case PPPERR_PROTOCOL:          /* Failed to meet protocol. */
          printf("ppp_link_status_cb: PPPERR_PROTOCOL\n");
          break;

      case PPPERR_PEERDEAD:          /* Connection timeout. */
          printf("ppp_link_status_cb: PPPERR_PEERDEAD\n");
          break;

      case PPPERR_IDLETIMEOUT:       /* Idle Timeout. */
          printf("ppp_link_status_cb: PPPERR_IDLETIMEOUT\n");
          break;

      case PPPERR_CONNECTTIME:       /* PPPERR_CONNECTTIME. */
          printf("ppp_link_status_cb: PPPERR_CONNECTTIME\n");
          break;

      case PPPERR_LOOPBACK:          /* Connection timeout. */
          printf("ppp_link_status_cb: PPPERR_LOOPBACK\n");
          break;
      default:
          printf("ppp_link_status_cb: unknown errCode %d\n", err_code);
          break;
    }
}

/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  IP_ADDRESS[0] = 192;
  IP_ADDRESS[1] = 168;
  IP_ADDRESS[2] = 100;
  IP_ADDRESS[3] = 2;
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 0;
  GATEWAY_ADDRESS[1] = 0;
  GATEWAY_ADDRESS[2] = 0;
  GATEWAY_ADDRESS[3] = 0;

/* USER CODE BEGIN IP_ADDRESSES */
/* USER CODE END IP_ADDRESSES */

  /* Initialize the LwIP stack with RTOS */
  tcpip_init( NULL, NULL );

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) with RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  /* We must always bring the network interface up connection or not... */
  netif_set_up(&gnetif);

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);

  /* Create the Ethernet link handler thread */
/* USER CODE BEGIN H7_OS_THREAD_DEF_CREATE_CMSIS_RTOS_V1 */
  osThreadDef(EthLink, ethernet_link_thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE *2);
  osThreadCreate (osThread(EthLink), &gnetif);
/* USER CODE END H7_OS_THREAD_DEF_CREATE_CMSIS_RTOS_V1 */

/* USER CODE BEGIN 3 */
  pppConnect();
/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
static void ethernet_link_status_updated(struct netif *netif)
{
  if (netif_is_up(netif))
  {
/* USER CODE BEGIN 5 */
/* USER CODE END 5 */
  }
  else /* netif is down */
  {
/* USER CODE BEGIN 6 */
/* USER CODE END 6 */
  }
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */

  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

