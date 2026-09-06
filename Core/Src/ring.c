/**
  ******************************************************************************
  * @file    ring.c
  * @brief   UART RX ring buffer (fed by USART3 ISR, consumed by main loop).
  *          (v2.1: extracted from usbd_cdc_interface.c during module split.)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ring.h"

/* Private variables ---------------------------------------------------------*/
/* RX ring buffer (fed by UART_RxByte from USART3 ISR) */
volatile uint16_t rx_head = 0U;   /* next write position (ISR)   */
volatile uint16_t rx_tail = 0U;   /* next read position (main)   */
uint8_t rx_ring[CDC_RX_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Called from the USART3 ISR when a byte arrives. Pushes into ring.
  *         Ring-full policy: drop silently (command rate << buffer capacity).
  * @param  byte  received byte
  */
void UART_RxByte(uint8_t byte)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % CDC_RX_BUFFER_SIZE);
  if (next != rx_tail)                   /* not full */
  {
    rx_ring[rx_head] = byte;
    rx_head = next;
  }
}
