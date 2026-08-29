/**
  ******************************************************************************
  * @file    usbd_desc.h
  * @brief   Header for usbd_desc.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_DESC_H
#define __USBD_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define         DEVICE_ID1          (0x1FFFF7E8U)
#define         DEVICE_ID2          (0x1FFFF7ECU)
#define         DEVICE_ID3          (0x1FFFF7F0U)

#define  USB_SIZ_STRING_SERIAL     0x0EU  /* 2 + 6 chars (Get_SerialNum fills 6) */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
extern USBD_DescriptorsTypeDef FS_Desc;

#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC_H */
