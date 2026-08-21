/**
  ******************************************************************************
  * @file    usbd_cdc_interface.c
  * @brief   USB CDC interface: ring buffer, TX, and command processing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_interface.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* No-command timeout: hold position after this many ms without any command. */
#define CMD_TIMEOUT_MS   10000U

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t UserRxBufferFS[CDC_RX_BUFFER_SIZE];

/* RX ring buffer */
static volatile uint16_t rx_head = 0U;   /* next write position (ISR)   */
static volatile uint16_t rx_tail = 0U;   /* next read position (main)   */
static uint8_t rx_ring[CDC_RX_BUFFER_SIZE];

/* Command line buffer (main loop) */
static char cmd_line[64];
static uint16_t cmd_len = 0U;

/* Servo angle storage (degrees, clamped to joint limits).
 * Written by the main loop only (Servo_SetAngle via CDC_ProcessRx).
 * 16-bit aligned accesses are atomic on Cortex-M3. */
static volatile uint16_t servo_angle[6] = {90U, 90U, 90U, 90U, 90U, 90U};

/* Per-channel PWM enable flags. A channel is started LAZILY by its first
 * Servo_SetAngle call, so power-up and timeout recovery draw zero current
 * until the PC explicitly commands that servo - this is what makes the
 * PC-side soft start (M1..M6, 200 ms apart) actually sequence the inrush.
 * Cleared by Servo_DisableAll (E-stop / USB loss / timeout). */
static uint8_t servo_enabled[6] = {0U, 0U, 0U, 0U, 0U, 0U};

/* Joint angle limits (servo degrees, 0-270 = 0.5-2.5ms PWM on 270-deg servos).
 * Values from design doc, TBD after real measurement of the mechanical
 * stops (see docs/诚实声明与技术债务清单.md). These are SERVO angles;
 * the PC maps joint angles via JOINT_OFFSET in config.py:
 *   J1[0,180] J2[30,180] J3[10,150] J4[0,180] J5[45,135] J6[30,120]
 * (limits kept conservative at 0-180 until stage-6 measurement) */
static const uint16_t joint_min[6] = {0U, 30U, 10U, 0U, 45U, 30U};
static const uint16_t joint_max[6] = {180U, 180U, 150U, 180U, 135U, 120U};

/* Safety state flags */
static volatile uint8_t estop_active    = 0U;  /* E command: PWM stopped      */
static volatile uint8_t timeout_active   = 0U; /* no command for 10s          */
static volatile uint32_t last_cmd_tick   = 0U; /* HAL tick of last command    */

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

static void Servo_SetAngle(uint8_t ch, uint16_t angle);
static void Servo_SetAll(uint16_t angle);
static void Servo_DisableAll(void);
static void Servo_EnableAll(void);
static void Cmd_Execute(const char *line);

/* Exported variables --------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC interface init.
  * @retval status
  */
static int8_t CDC_Init_FS(void)
{
  /* USB CDC is DEPRECATED on this board (it enumerates but Windows never
   * registers the device; commands and replies use the USART3 bridge).
   * Do NOT enable servos here: enabling is owned exclusively by explicit
   * USART3 commands through the lazy per-channel start in Servo_SetAngle.
   * A hidden enable path here would bypass the PC-side soft start. */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return 0;
}

/**
  * @brief  CDC interface de-init.
  * @retval status
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USB is retired to a pure-enumeration stub: the command/reply path is
   * the USART3 bridge, so USB bus events (reset / suspend / unplug) must
   * NOT stop the servos or flush the command ring - a host reboot must not
   * make a grasped object fall. Servo power is governed solely by E-stop,
   * the 10 s command timeout, and per-command lazy starts. */
  return 0;
}

/**
  * @brief  CDC control (line coding etc.).
  * @retval status
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd)
  {
    case CDC_SET_LINE_CODING:
      break;
    case CDC_GET_LINE_CODING:
      break;
    case CDC_SET_CONTROL_LINE_STATE:
      break;
    default:
      break;
  }
  UNUSED(pbuf);
  UNUSED(length);
  return 0;
}

/**
  * @brief  CDC receive callback: push data into the ring buffer.
  * @retval status
  */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  /* USB CDC RX is retired: the USART3 bridge (UART_RxByte) is the ONLY
   * command input. Bytes arriving over USB are discarded - feeding them
   * into rx_ring would make it a multi-writer ring shared by two ISR
   * contexts for no benefit. */
  UNUSED(Buf);
  UNUSED(Len);
  /* Re-arm the receive so the host's writer does not stall. */
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return 0;
}

/**
  * @brief  CDC transmit complete callback.
  * @retval status
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  return 0;
}

/**
  * @brief  Push one USART3 received byte into the shared command ring.
  *         Called from USART3_IRQHandler (ISR context).
  * @param  byte: received byte
  * @retval None
  */
void UART_RxByte(uint8_t byte)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % CDC_RX_BUFFER_SIZE);
  if (next != rx_tail)
  {
    rx_ring[rx_head] = byte;
    rx_head = next;
  }
  /* else: ring full, byte dropped (same policy as the USB CDC path). */
}

/**
  * @brief  Send a string back to the PC.
  *         Replies go over the USART3 serial bridge (register-level,
  *         blocking TX) since USB CDC is not usable on this board.
  * @retval 0 on success
  */
int8_t CDC_SendString(const char *str)
{
  uint16_t len = 0U;
  uint16_t i;
  while (str[len] != '\0')
  {
    len++;
  }
  if (len == 0U)
  {
    return 0;
  }
  if (len > CDC_TX_BUFFER_SIZE)
  {
    len = CDC_TX_BUFFER_SIZE;
  }
  /* Blocking transmit; called from main-loop context only (Cmd_Execute),
   * never from an ISR. Short replies at 115200 take ~1-5 ms, well within
   * the 2 s watchdog budget. */
  for (i = 0U; i < len; i++)
  {
    while ((USART3->SR & USART_SR_TXE) == 0U)
    {
    }
    USART3->DR = (uint16_t)str[i];
  }
  while ((USART3->SR & USART_SR_TC) == 0U)
  {
  }
  return 0;
}

/**
  * @brief  Set one servo angle, clamped to the joint limits.
  */
static void Servo_SetAngle(uint8_t ch, uint16_t angle)
{
  uint32_t cmp;
  if (ch >= 6U)
  {
    return;
  }
  /* Clamp to joint limit (safety layer L2). */
  if (angle < joint_min[ch])
  {
    angle = joint_min[ch];
  }
  if (angle > joint_max[ch])
  {
    angle = joint_max[ch];
  }
  servo_angle[ch] = angle;
  cmp = SERVO_MIN_PULSE + ((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / SERVO_MAX_ANGLE;

  switch (ch)
  {
    /* Compare is written BEFORE the lazy start so the very first pulse the
     * servo ever sees already carries the commanded width (never the timer
     * reset default). The start is gated on estop_active: an M command
     * during an E-stop must only update the stored angle, never re-power
     * the channel (the E command stops PWM; 'H' is the explicit recovery). */
    case 0U:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
      if ((servo_enabled[0] == 0U) && (estop_active == 0U))
      {
        servo_enabled[0] = 1U;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      }
      break;
    case 1U:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp);
      if ((servo_enabled[1] == 0U) && (estop_active == 0U))
      {
        servo_enabled[1] = 1U;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      }
      break;
    case 2U:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmp);
      if ((servo_enabled[2] == 0U) && (estop_active == 0U))
      {
        servo_enabled[2] = 1U;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      }
      break;
    case 3U:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, cmp);
      if ((servo_enabled[3] == 0U) && (estop_active == 0U))
      {
        servo_enabled[3] = 1U;
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      }
      break;
    case 4U:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, cmp);
      if ((servo_enabled[4] == 0U) && (estop_active == 0U))
      {
        servo_enabled[4] = 1U;
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
      }
      break;
    case 5U:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, cmp);
      if ((servo_enabled[5] == 0U) && (estop_active == 0U))
      {
        servo_enabled[5] = 1U;
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
      }
      break;
    default: break;
  }
}

/**
  * @brief  Set all servos to the same angle.
  */
static void Servo_SetAll(uint16_t angle)
{
  uint8_t i;
  for (i = 0U; i < 6U; i++)
  {
    Servo_SetAngle(i, angle);
  }
}

/**
  * @brief  Stop PWM on all 6 channels. Servos lose the signal and go limp
  *         (no torque). Used on E-stop and USB disconnect.
  */
static void Servo_DisableAll(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  /* Clear the lazy-start flags so the next command per channel genuinely
   * re-powers it through Servo_SetAngle (never a bulk blind start). */
  servo_enabled[0] = 0U;
  servo_enabled[1] = 0U;
  servo_enabled[2] = 0U;
  servo_enabled[3] = 0U;
  servo_enabled[4] = 0U;
  servo_enabled[5] = 0U;
}

/**
  * @brief  Restore PWM on all 6 channels at the last commanded angles.
  *         Each Servo_SetAngle lazily starts its channel on first use, so
  *         re-issuing the stored angles is sufficient - channels that are
  *         already running are only updated, stopped ones are re-powered.
  *
  * @note   Used ONLY by 'H' (explicit user recovery). All six channels come
  *         up within microseconds of each other; that is acceptable here
  *         because recovery happens at held positions (hold current, no
  *         acceleration inrush) and the PTC bounds any fault. Boot and the
  *         timeout path deliberately do NOT call this: they rely on the
  *         per-command lazy start so the PC soft start stays effective.
  */
static void Servo_EnableAll(void)
{
  uint8_t i;
  for (i = 0U; i < 6U; i++)
  {
    Servo_SetAngle(i, servo_angle[i]);
  }
}

/**
  * @brief  Execute one command line.
  *         Supported commands:
  *           M<n> <angle>   move servo n (1..6) to angle (clamped to joint limits)
  *           MALL <angle>   move all servos to angle
  *           I              query IR sensors -> "IR1=x IR2=y"
  *           S              query servo angles -> "S1=.. S2=.. ..."
  *           H              home: all servos to 90 deg (also clears E-stop)
  *           E              emergency stop: stop all PWM (servos go limp)
  */
static void Cmd_Execute(const char *line)
{
  char reply[64];
  uint16_t a, b;
  int ia, ib;

  if ((line[0] == 'M') || (line[0] == 'm'))
  {
    if (estop_active != 0U)
    {
      /* E-stop active: refuse motion commands until 'H'. */
      CDC_SendString("ERR\r\n");
      return;
    }
    if ((line[1] == 'A') || (line[1] == 'a'))
    {
      /* MALL <angle>. Each joint clamps to its own limit, so the echoed
       * value is the requested one; query 'S' for actual positions.
       * NOTE: unlike M<n>, MALL echoes the REQUESTED angle, not the clamped
       * one - the PC must not treat this echo as actual joint state
       * (arm_serial.py only parses "OK M<n> <angle>" echoes; trajectory.py
       * avoids MALL entirely for this reason). */
      if (sscanf(line + 4, "%d", &ia) == 1)
      {
        if (ia < 0)
        {
          CDC_SendString("ERR\r\n");
          return;
        }
        a = (uint16_t)ia;
        Servo_SetAll(a);
        sprintf(reply, "OK MALL %u\r\n", a);
        CDC_SendString(reply);
      }
      else
      {
        CDC_SendString("ERR\r\n");
      }
    }
    else
    {
      /* M<n> <angle> */
      if (sscanf(line + 1, "%d %d", &ia, &ib) == 2)
      {
        if ((ia >= 1) && (ia <= 6) && (ib >= 0))
        {
          a = (uint16_t)ia;
          b = (uint16_t)ib;
          Servo_SetAngle((uint8_t)(a - 1U), b);
          /* Echo the clamped angle so the PC knows the real position. */
          sprintf(reply, "OK M%u %u\r\n", a, (uint16_t)servo_angle[a - 1U]);
          CDC_SendString(reply);
        }
        else
        {
          CDC_SendString("ERR\r\n");
        }
      }
      else
      {
        CDC_SendString("ERR\r\n");
      }
    }
  }
  else if ((line[0] == 'I') || (line[0] == 'i'))
  {
    sprintf(reply, "IR1=%u IR2=%u\r\n",
            (uint16_t)HAL_GPIO_ReadPin(IR1_GPIO_PORT, IR1_GPIO_PIN),
            (uint16_t)HAL_GPIO_ReadPin(IR2_GPIO_PORT, IR2_GPIO_PIN));
    CDC_SendString(reply);
  }
  else if ((line[0] == 'S') || (line[0] == 's'))
  {
    sprintf(reply, "S1=%u S2=%u S3=%u S4=%u S5=%u S6=%u\r\n",
            (uint16_t)servo_angle[0], (uint16_t)servo_angle[1],
            (uint16_t)servo_angle[2], (uint16_t)servo_angle[3],
            (uint16_t)servo_angle[4], (uint16_t)servo_angle[5]);
    CDC_SendString(reply);
  }
  else if ((line[0] == 'H') || (line[0] == 'h'))
  {
    /* Home: clear E-stop, re-enable PWM, move all servos to 90 deg. */
    estop_active = 0U;
    Servo_EnableAll();
    Servo_SetAll(90U);
    CDC_SendString("OK H\r\n");
  }
  else if ((line[0] == 'E') || (line[0] == 'e'))
  {
    /* Emergency stop: stop all PWM immediately (servos go limp). */
    estop_active = 1U;
    Servo_DisableAll();
    CDC_SendString("OK E\r\n");
  }
  else
  {
    CDC_SendString("ERR\r\n");
  }
}

/**
  * @brief  Process received bytes: assemble lines, execute on '\n'.
  *         Called from the main loop.
  */
void CDC_ProcessRx(void)
{
  while (rx_tail != rx_head)
  {
    char c = (char)rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % CDC_RX_BUFFER_SIZE);

    if ((c == '\n') || (c == '\r'))
    {
      if (cmd_len > 0U)
      {
        cmd_line[cmd_len] = '\0';
        /* Any complete line means the PC is alive: reset timeout and, if the
         * timeout had stopped the PWM, re-enable the servos at the last
         * commanded angles before executing the new command. */
        last_cmd_tick = HAL_GetTick();
        if (timeout_active != 0U)
        {
          /* Clear the timeout flag ONLY. Do NOT bulk re-enable here: the
           * executing command's Servo_SetAngle lazily restarts exactly the
           * channel(s) it moves, so the soft-start property survives a PC
           * silence timeout. Query commands (S/I) intentionally leave the
           * arm limp until a real motion command arrives. */
          timeout_active = 0U;
        }
        Cmd_Execute(cmd_line);
        cmd_len = 0U;
      }
    }
    else if (cmd_len < (sizeof(cmd_line) - 1U))
    {
      cmd_line[cmd_len++] = c;
    }
    else
    {
      /* line too long: drop it */
      cmd_len = 0U;
    }
  }
}

/**
  * @brief  Check the no-command timeout. Called from the main loop.
  *         When no command arrives for CMD_TIMEOUT_MS, stop all PWM so the
  *         arm goes limp (safe: the PC has gone silent, e.g. crashed or the
  *         COM port was closed). The next command line re-enables the servos.
  */
void CDC_TimeoutCheck(void)
{
  if ((timeout_active == 0U) &&
      (HAL_GetTick() - last_cmd_tick > CMD_TIMEOUT_MS))
  {
    timeout_active = 1U;
    Servo_DisableAll();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
