/**
  ******************************************************************************
  * @file    usbd_conf.c
  * @brief   USB Device configuration and low-level callbacks
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hPCD;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

/**
  * @brief  Initializes the PCD.
  * @param  pdev: device instance
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  /* Set LL driver parameters */
  hPCD.Instance = USB;
  hPCD.Init.dev_endpoints = 8U;
  hPCD.Init.speed = PCD_SPEED_FULL;
  hPCD.Init.ep0_mps = PCD_EP0MPS_64;
  hPCD.Init.phy_itface = PCD_PHY_EMBEDDED;
  hPCD.Init.Sof_enable = DISABLE;
  hPCD.Init.low_power_enable = DISABLE;
  hPCD.Init.lpm_enable = DISABLE;
  hPCD.Init.battery_charging_enable = DISABLE;

  /* Link the driver to the stack */
  hPCD.pData = pdev;
  pdev->pData = &hPCD;

  /* Initialize LL Driver */
  HAL_PCD_Init(&hPCD);

  HAL_PCDEx_PMAConfig(&hPCD, 0x00U, PCD_SNG_BUF, 0x18U);
  HAL_PCDEx_PMAConfig(&hPCD, 0x80U, PCD_SNG_BUF, 0x58U);
  HAL_PCDEx_PMAConfig(&hPCD, 0x01U, PCD_SNG_BUF, 0x98U);
  HAL_PCDEx_PMAConfig(&hPCD, 0x81U, PCD_SNG_BUF, 0xD8U);
  HAL_PCDEx_PMAConfig(&hPCD, 0x82U, PCD_SNG_BUF, 0x118U);

  return USBD_OK;
}

/**
  * @brief  De-Initializes the PCD.
  * @param  pdev: device instance
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  pdev->pData = NULL;
  return USBD_OK;
}

/**
  * @brief  Starts the PCD.
  * @param  pdev: device instance
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Stops the PCD.
  * @param  pdev: device instance
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK;
}

/**
  * @brief  Opens an endpoint.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @param  ep_type: endpoint type (USBD_EP_TYPE_*)
  * @param  ep_mps: endpoint max packet size
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps)
{
  HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  return USBD_OK;
}

/**
  * @brief  Closes an endpoint.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Flushes an endpoint (no FIFO on F1, nothing to do).
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  UNUSED(pdev);
  UNUSED(ep_addr);
  return USBD_OK;
}

/**
  * @brief  Stalls an endpoint.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Clears the stall condition on an endpoint.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Sets the USB device address.
  * @param  pdev: device instance
  * @param  dev_addr: device address
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK;
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @param  pbuf: data buffer
  * @param  size: data size
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @param  pbuf: data buffer
  * @param  size: data size
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                          uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
  * @brief  Returns the stall status of an endpoint.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @retval 1 if stalled, 0 otherwise
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  uint8_t epnum = ep_addr & EP_ADDR_MSK;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    return hpcd->IN_ep[epnum].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[epnum].is_stall;
  }
}

/**
  * @brief  Returns the received data size.
  * @param  pdev: device instance
  * @param  ep_addr: endpoint address
  * @retval received data size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/**
  * @brief  Delays the USB stack.
  * @param  Delay: delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/*******************************************************************************
                       HAL PCD Callbacks (HAL -> PCD)
*******************************************************************************/

/**
  * @brief  PCD MSP Initialization.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hpcd->Instance == USB)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USB D- (PA11) and D+ (PA12) */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USB interrupt */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

/**
  * @brief  PCD MSP De-Initialization.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Instance == USB)
  {
    __HAL_RCC_USB_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

/**
  * @brief  Setup stage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
  * @brief  Data OUT stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, NULL);
}

/**
  * @brief  Data IN stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, NULL);
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  /* STM32F103 USB FS IP is a full-speed device only (PCD_SPEED_FULL).
   * The USB library keeps dev_speed at its initial value
   * (USBD_SPEED_HIGH = 0) unless set here; with dev_speed==HIGH,
   * GET_DESCRIPTOR(Configuration) returns the HS descriptor (endpoint
   * wMaxPacketSize = 512), which is illegal on a full-speed device and
   * makes Windows reject enumeration. Always report full speed. */
  (void)USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);

  USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

/**
  * @brief  Suspend callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

/**
  * @brief  Resume callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

/**
  * @brief  ISO OUT incomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

/**
  * @brief  ISO IN incomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

/**
  * @brief  Connect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  *
  * @note   STM32F103 USB FS IP HAS NO PHYSICAL DISCONNECT INTERRUPT.
  *         This callback is NEVER invoked on this MCU - do not rely on it
  *         for "USB unplugged" detection (e.g. stopping servo PWM).
  *         Actual unplug path on F1: 3ms without SOF -> SUSP interrupt ->
  *         HAL_PCD_SuspendCallback. Servo PWM stop on unplug is therefore
  *         handled by the PC-side 10s command timeout, NOT by this callback.
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
