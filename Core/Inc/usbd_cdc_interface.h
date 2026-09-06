/**
  ******************************************************************************
  * @file    usbd_cdc_interface.h
  * @brief   Top-level include for command processor, servo, and ring modules.
  *          (v2.1: USB CDC removed — project uses USART3 + CH340 serial bridge.
  *           File name kept for Keil project compatibility.)
  *          After module split: this header aggregates ring.h, servo.h, cmd.h.
  ******************************************************************************
  */

#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include all sub-module headers so existing main.c/main.h include
 * "usbd_cdc_interface.h" still sees every exported symbol. */
#include "ring.h"
#include "servo.h"
#include "cmd.h"

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */
