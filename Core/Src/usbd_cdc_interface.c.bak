/**
  ******************************************************************************
  * @file    usbd_cdc_interface.c
  * @brief   Command processor: UART RX ring, servo PWM, safety layer.
  *          (v2.1: USB CDC removed — project uses USART3 + CH340 serial bridge.
  *           File name kept for Keil project compatibility.)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "usbd_cdc_interface.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* No-command timeout: hold position after this many ms without any command. */
#define CMD_TIMEOUT_MS   10000U

/* Private variables ---------------------------------------------------------*/
/* RX ring buffer (fed by UART_RxByte from USART3 ISR) */
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
 * PC-side soft start (M1..M6, 200 ms apart) genuinely sequence the inrush.
 * Cleared by Servo_DisableAll (E-stop / timeout). */
static uint8_t servo_enabled[6] = {0U, 0U, 0U, 0U, 0U, 0U};

/* Joint angle limits (servo degrees, 0-270 = 0.5-2.5ms PWM on 270-deg servos).
 * Conservative 0-180 band until stage-6 measurement (see docs). */
static const uint16_t joint_min[6] = {0U, 30U, 10U, 0U, 45U, 30U};
static const uint16_t joint_max[6] = {180U, 180U, 150U, 180U, 135U, 120U};

/* Safety state flags */
static volatile uint8_t estop_active    = 0U;  /* E command: PWM stopped      */
static volatile uint8_t timeout_active   = 0U; /* no command for 10s          */
static volatile uint32_t last_cmd_tick   = 0U; /* HAL tick of last command    */

/* Private function prototypes -----------------------------------------------*/
static void Servo_SetAngle(uint8_t ch, uint16_t angle);
static void Servo_SetAll(uint16_t angle);
static void Servo_DisableAll(void);
static uint8_t HexNibble(char c);
static uint8_t Cmd_CheckChecksum(char *line);
static void Cmd_Execute(const char *line);

/* Exported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/**
  * @brief  Called from the USART3 ISR when a byte arrives. Pushes into ring.
  *         Ring-full policy: drop silently (command rate << buffer capacity).
  * @param  byte  received byte
  */
void UART_RxByte(uint8_t byte)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % CDC_RX_BUFFER_SIZE);
  if (next != rx_tail)                   /* not full */
  {
    rx_ring[rx_head] = byte;
    rx_head = next;
  }
}

/**
  * @brief  Lazy-start no-op: RXNEIE is already enabled by MX_USART3_UART_Init.
  */
void UART_StartRx(void)
{
}

/**
  * @brief  Blocking transmit over USART3 (register-level polling).
  *         Called from main-loop context only (Cmd_Execute), never ISR.
  * @param  str  NUL-terminated string (includes \r\n)
  */
int8_t CDC_SendString(const char *str)
{
  while (*str != '\0')
  {
    while ((USART3->SR & USART_SR_TXE) == 0U) { /* wait TX empty */ }
    USART3->DR = (uint16_t)(*str++);
  }
  while ((USART3->SR & USART_SR_TC) == 0U) { /* wait transmission complete */ }
  return 0;
}

/**
  * @brief  Set servo angle (clamped) and lazily start its PWM channel.
  * @param  ch     channel index 0..5
  * @param  angle  requested servo angle (degrees)
  */
static void Servo_SetAngle(uint8_t ch, uint16_t angle)
{
  uint16_t cmp;
  TIM_HandleTypeDef *tim;
  uint32_t chan;

  if (ch >= 6U) return;

  /* Clamp to joint limits */
  if (angle < joint_min[ch]) { angle = joint_min[ch]; }
  if (angle > joint_max[ch]) { angle = joint_max[ch]; }

  /* Angle -> pulse width -> compare value (270-deg servos) */
  cmp = SERVO_MIN_PULSE +
        ((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / SERVO_MAX_ANGLE;

  /* Select timer/channel */
  switch (ch)
  {
    case 0: tim = &htim1; chan = TIM_CHANNEL_1; break;
    case 1: tim = &htim1; chan = TIM_CHANNEL_2; break;
    case 2: tim = &htim1; chan = TIM_CHANNEL_3; break;
    case 3: tim = &htim2; chan = TIM_CHANNEL_1; break;
    case 4: tim = &htim2; chan = TIM_CHANNEL_2; break;
    default: tim = &htim2; chan = TIM_CHANNEL_3; break;
  }

  __HAL_TIM_SET_COMPARE(tim, chan, cmp);
  servo_angle[ch] = angle;

  /* Lazy start: only enable PWM on first command for this channel. */
  if ((servo_enabled[ch] == 0U) && (estop_active == 0U))
  {
    servo_enabled[ch] = 1U;
    HAL_TIM_PWM_Start(tim, chan);
  }
}

/**
  * @brief  Set all six servo angles (each clamped independently).
  * @param  angle  requested angle
  * @note   Echoes the REQUESTED angle; query 'S' for actual positions.
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
  * @brief  Stop all PWM outputs (servos go limp / unpowered).
  */
static void Servo_DisableAll(void)
{
  uint8_t i;
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  for (i = 0U; i < 6U; i++) { servo_enabled[i] = 0U; }
}

/**
  * @brief  Hex digit to nibble. Returns 0xFF for non-hex characters.
  */
static uint8_t HexNibble(char c)
{
  if ((c >= '0') && (c <= '9')) { return (uint8_t)(c - '0'); }
  if ((c >= 'A') && (c <= 'F')) { return (uint8_t)(c - 'A' + 10); }
  if ((c >= 'a') && (c <= 'f')) { return (uint8_t)(c - 'a' + 10); }
  return 0xFFU;
}

/**
  * @brief  Verify optional "*XX" XOR checksum suffix on a command line.
  * @param  line  command line (modified in place: truncated at '*')
  * @retval 1 = valid (or absent), 0 = bad checksum
  */
static uint8_t Cmd_CheckChecksum(char *line)
{
  char *star = NULL;
  const char *p;
  uint8_t acc = 0U;
  uint8_t hi, lo;

  for (p = line; (*p != '\0') && (p < (line + sizeof(cmd_line))); p++)
  {
    if (*p == '*') { star = (char *)p; break; }
  }
  if (star == NULL) { return 1U; }
  if ((star[1] == '\0') || (star[2] == '\0')) { return 0U; }
  for (p = line; p < star; p++) { acc ^= (uint8_t)*p; }
  hi = HexNibble(star[1]);
  lo = HexNibble(star[2]);
  if ((hi > 15U) || (lo > 15U)) { return 0U; }
  if (acc != (uint8_t)((uint8_t)(hi << 4) | lo)) { return 0U; }
  *star = '\0';
  return 1U;
}

/**
  * @brief  Execute one command line.
  *         M<n> <angle> / MALL <angle> / I / S / H / E
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
      CDC_SendString("ERR\r\n");
      return;
    }
    if ((line[1] == 'A') || (line[1] == 'a'))
    {
      if (sscanf(line + 4, "%d", &ia) == 1)
      {
        if (ia < 0) { CDC_SendString("ERR\r\n"); return; }
        a = (uint16_t)ia;
        Servo_SetAll(a);
        sprintf(reply, "OK MALL %u\r\n", a);
        CDC_SendString(reply);
      }
      else { CDC_SendString("ERR\r\n"); }
    }
    else
    {
      if (sscanf(line + 1, "%d %d", &ia, &ib) == 2)
      {
        if ((ia >= 1) && (ia <= 6) && (ib >= 0))
        {
          a = (uint16_t)ia;
          b = (uint16_t)ib;
          Servo_SetAngle((uint8_t)(a - 1U), b);
          sprintf(reply, "OK M%u %u\r\n", a, (uint16_t)servo_angle[a - 1U]);
          CDC_SendString(reply);
        }
        else { CDC_SendString("ERR\r\n"); }
      }
      else { CDC_SendString("ERR\r\n"); }
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
    static const uint16_t home_servo[6] = {90U, 135U, 60U, 75U, 90U, 90U};
    uint8_t hi;
    estop_active = 0U;
    for (hi = 0U; hi < 6U; hi++)
    {
      Servo_SetAngle(hi, home_servo[hi]);
    }
    CDC_SendString("OK H\r\n");
  }
  else if ((line[0] == 'E') || (line[0] == 'e'))
  {
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
        if (Cmd_CheckChecksum(cmd_line) == 0U)
        {
          /* Corrupted command (EMI): refuse loudly, do NOT execute.
           * Do NOT refresh last_cmd_tick either. */
          CDC_SendString("ERR CKS\r\n");
        }
        else
        {
          last_cmd_tick = HAL_GetTick();
          if (timeout_active != 0U) { timeout_active = 0U; }
          Cmd_Execute(cmd_line);
        }
        cmd_len = 0U;
      }
    }
    else if (cmd_len < (sizeof(cmd_line) - 1U))
    {
      cmd_line[cmd_len++] = c;
    }
    else
    {
      cmd_len = 0U;
    }
  }
}

/**
  * @brief  Check the no-command timeout. Called from the main loop.
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
