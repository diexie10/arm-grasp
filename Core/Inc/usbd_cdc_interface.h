/**
  ******************************************************************************
  * @file    usbd_cdc_interface.h
  * @brief   Header for usbd_cdc_interface.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define CDC_RX_BUFFER_SIZE   512U
#define CDC_TX_BUFFER_SIZE   512U

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* Send a string back to the PC over the USART3 serial bridge (blocking).
 * Returns 0 on success, -1 if the TX buffer is busy. */
int8_t CDC_SendString(const char *str);
void UART_RxByte(uint8_t byte);

/* Called from main loop: process any received command line. */
void CDC_ProcessRx(void);

/* Called from main loop: check no-command timeout (auto-hold). */
void CDC_TimeoutCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */
