/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Command processor interface (parse/execute serial commands).
  *          (v2.1: extracted from usbd_cdc_interface.c during module split.)
  ******************************************************************************
  */

#ifndef __CMD_H
#define __CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Safety state flags (written by E/H/G commands, read by servo module) */
extern volatile uint8_t  estop_active;    /* E command: PWM stopped      */
extern volatile uint8_t  timeout_active;  /* no command for 15s (see CMD_TIMEOUT_MS) */
extern volatile uint32_t last_cmd_tick;   /* HAL tick of last command    */

/* Exported functions --------------------------------------------------------*/

/* Send a string back to the PC over USART3 (blocking, register-level). */
int8_t CDC_SendString(const char *str);

/* Called from main loop: process any received command line. */
void CDC_ProcessRx(void);

/* Called from main loop: check no-command timeout (auto-hold). */
void CDC_TimeoutCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H */
