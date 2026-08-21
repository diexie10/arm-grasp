/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_interface.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* Servo PWM: 50Hz (20ms), pulse 0.5ms..2.5ms -> 0..270 deg
 * (servos are 270-degree type, verified by user 2026-08-20)
 * Timer clock 1MHz (PSC=71), ARR=19999 -> 20ms period
 * Compare value = 500 + angle * 2000 / 270
 */
#define SERVO_MIN_PULSE    500U   /* 0.5ms -> 0 deg   */
#define SERVO_MAX_PULSE    2500U  /* 2.5ms -> 270 deg */
#define SERVO_MAX_ANGLE    270U   /* 270-degree servos */
/* Neutral (home) pulse derived from the SAME mapping Servo_SetAngle uses,
 * so the timer reset value and command PWM share one definition of "90 deg".
 * = 500 + 90*2000/270 = 1166 us (= 89.9 deg physical on a 270-deg servo).
 * NEVER hardcode 1500 here: on a 270-deg servo that is 135 deg, not home. */
#define SERVO_HOME_PULSE   (SERVO_MIN_PULSE + (90U * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / SERVO_MAX_ANGLE)

/* IR sensor GPIOs (active low: 0 = beam broken / object present) */
#define IR1_GPIO_PORT      GPIOB
#define IR1_GPIO_PIN       GPIO_PIN_12
#define IR2_GPIO_PORT      GPIOB
#define IR2_GPIO_PIN       GPIO_PIN_13

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
