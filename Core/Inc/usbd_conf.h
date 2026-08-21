/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @brief   USB Device configuration header
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_pcd.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define USBD_MAX_NUM_INTERFACES                      1U
#define USBD_MAX_NUM_CONFIGURATION                   1U
#define USBD_MAX_CONFIGURATION_SIZE                  256U
#define USBD_MAX_SUPPORTED_CLASS                     1U
#define USBD_CDC_INTERFACE_NUM                       0U
#define USBD_CDC_CMD_EP                              0x82U
#define USBD_CDC_DATA_IN_EP                          0x81U
#define USBD_CDC_DATA_OUT_EP                         0x01U

#define USBD_SELF_POWERED                            0U

/* Exported macro ------------------------------------------------------------*/
/* Memory management macros */
#define USBD_malloc               malloc
#define USBD_free                 free
#define USBD_memset               memset
#define USBD_memcpy               memcpy

/* DEBUG macros */
#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)    printf(__VA_ARGS__); \
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif /* USBD_DEBUG_LEVEL */

#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...)    printf("USER: ERROR -> "); \
                            printf(__VA_ARGS__); \
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif /* USBD_DEBUG_LEVEL */

#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...)    printf("USER: DBG  -> "); \
                            printf(__VA_ARGS__); \
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif /* USBD_DEBUG_LEVEL */

/* Exported functions --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hPCD;

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
