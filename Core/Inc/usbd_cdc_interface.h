/**
  ******************************************************************************
  * @file    usbd_cdc_interface.h
  * @brief   Command processor interface (USART3 + CH340 serial bridge).
  *          (v2.1: USB CDC removed — file name kept for Keil compatibility.)
  ******************************************************************************
  */

#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Buffer sizes */
#define CDC_RX_BUFFER_SIZE   512U

/* Exported functions --------------------------------------------------------*/
/* Send a string back to the PC over USART3 (blocking, register-level). */
int8_t CDC_SendString(const char *str);

/* Called from USART3 ISR: push received byte into ring buffer. */
void UART_RxByte(uint8_t byte);

/* Lazy-start no-op: RXNEIE already enabled by MX_USART3_UART_Init. */
void UART_StartRx(void);

/* Called from main loop: process any received command line. */
void CDC_ProcessRx(void);

/* Called from main loop: check no-command timeout (auto-hold). */
void CDC_TimeoutCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */
