/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @brief   USB Device descriptors
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"
#include "usbd_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USBD_VID                      0x1209U
#define USBD_PID                      0x0001U
#define USBD_LANGID_STRING            0x409U
#define USBD_MANUFACTURER_STRING      "arm-grasp"
#define USBD_PRODUCT_STRING           "Arm Grasp Virtual COM"
#define USBD_CONFIGURATION_STRING     "CDC Config"
#define USBD_INTERFACE_STRING         "CDC Interface"

#define USB_SIZ_STRING_MANUFACTURER   20U   /* "arm-grasp" = 2 + 9*2 */
#define USB_SIZ_STRING_PRODUCT        44U   /* "Arm Grasp Virtual COM" = 2 + 21*2 */
#define USB_SIZ_STRING_CONFIGURATION  22U   /* "CDC Config" = 2 + 10*2 */
#define USB_SIZ_STRING_INTERFACE      28U   /* "CDC Interface" = 2 + 13*2 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__ALIGN_BEGIN static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                        /* bLength */
  USB_DESC_TYPE_DEVICE,        /* bDescriptorType */
  0x00,                        /* bcdUSB */
  0x02,
  0x02,                        /* bDeviceClass */
  0x00,                        /* bDeviceSubClass */
  0x00,                        /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,            /* bMaxPacketSize */
  LOBYTE(USBD_VID),            /* idVendor */
  HIBYTE(USBD_VID),
  LOBYTE(USBD_PID),            /* idProduct */
  HIBYTE(USBD_PID),
  0x00,                        /* bcdDevice */
  0x02,
  USBD_IDX_MFC_STR,            /* iManufacturer */
  USBD_IDX_PRODUCT_STR,        /* iProduct */
  USBD_IDX_SERIAL_STR,         /* iSerialNumber */
  USBD_MAX_NUM_CONFIGURATION   /* bNumConfigurations */
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END =
{
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

__ALIGN_BEGIN static uint8_t USBD_StringManufacturer[USB_SIZ_STRING_MANUFACTURER] __ALIGN_END =
{
  USB_SIZ_STRING_MANUFACTURER,
  USB_DESC_TYPE_STRING,
  'a', 0, 'r', 0, 'm', 0, '-', 0, 'g', 0, 'r', 0, 'a', 0, 's', 0, 'p', 0
};

__ALIGN_BEGIN static uint8_t USBD_StringProduct[USB_SIZ_STRING_PRODUCT] __ALIGN_END =
{
  USB_SIZ_STRING_PRODUCT,
  USB_DESC_TYPE_STRING,
  'A', 0, 'r', 0, 'm', 0, ' ', 0, 'G', 0, 'r', 0, 'a', 0, 's', 0, 'p', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0
};

static uint8_t USBD_StringConfig[USB_SIZ_STRING_CONFIGURATION] =
{
  USB_SIZ_STRING_CONFIGURATION,
  USB_DESC_TYPE_STRING,
  'C', 0, 'D', 0, 'C', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0
};

static uint8_t USBD_StringInterface[USB_SIZ_STRING_INTERFACE] =
{
  USB_SIZ_STRING_INTERFACE,
  USB_DESC_TYPE_STRING,
  'C', 0, 'D', 0, 'C', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

/* Private function prototypes -----------------------------------------------*/
static void Get_SerialNum(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get the device serial number from the unique ID registers.
  * @retval None
  */
static void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *)DEVICE_ID1;
  deviceserial1 = *(uint32_t *)DEVICE_ID2;
  deviceserial2 = *(uint32_t *)DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0U)
  {
    USBD_StringSerial[2] = (uint8_t)(deviceserial0 >> 24);
    USBD_StringSerial[3] = (uint8_t)(deviceserial0 >> 16);
    USBD_StringSerial[4] = (uint8_t)(deviceserial0 >> 8);
    USBD_StringSerial[5] = (uint8_t)(deviceserial0);
    USBD_StringSerial[6] = (uint8_t)(deviceserial2 >> 24);
    USBD_StringSerial[7] = (uint8_t)(deviceserial2 >> 16);
    USBD_StringSerial[8] = (uint8_t)(deviceserial2 >> 8);
    USBD_StringSerial[9] = (uint8_t)(deviceserial2);
    USBD_StringSerial[10] = (uint8_t)(deviceserial1 >> 24);
    USBD_StringSerial[11] = (uint8_t)(deviceserial1 >> 16);
    USBD_StringSerial[12] = (uint8_t)(deviceserial1 >> 8);
    USBD_StringSerial[13] = (uint8_t)(deviceserial1);
  }
}

/**
  * @brief  Return the device descriptor.
  * @param  speed: current device speed
  * @param  length: pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_DeviceDesc);
  return (uint8_t *)USBD_DeviceDesc;
}

/**
  * @brief  Return the LangID string descriptor.
  * @param  speed: current device speed
  * @param  length: pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return (uint8_t *)USBD_LangIDDesc;
}

/**
  * @brief  Return the manufacturer string descriptor.
  * @param  speed: current device speed
  * @param  length: pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_MANUFACTURER;
  return (uint8_t *)USBD_StringManufacturer;
}

/**
  * @brief  Return the product string descriptor.
  * @param  speed: current device speed
  * @param  length: pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_PRODUCT;
  return (uint8_t *)USBD_StringProduct;
}

/**
  * @brief  Return the serial number string descriptor.
  * @param  speed: current device speed
  * @param  length: pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;
  Get_SerialNum();
  return (uint8_t *)USBD_StringSerial;
}

/**
  * @brief  Return the configuration string descriptor.
  * @param  speed: current device speed
  * @param  length: pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_CONFIGURATION;
  return (uint8_t *)USBD_StringConfig;
}

/**
  * @brief  Return the interface string descriptor.
  * @param  speed: current device speed
  * @param  length: pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_INTERFACE;
  return (uint8_t *)USBD_StringInterface;
}

/* Exported variables --------------------------------------------------------*/
USBD_DescriptorsTypeDef FS_Desc =
{
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor
};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
