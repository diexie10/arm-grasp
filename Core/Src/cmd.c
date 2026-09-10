/**
  ******************************************************************************
  * @file    cmd.c
  * @brief   Command processor: parse/execute serial commands, safety layer.
  *          (v2.1: USB CDC removed — project uses USART3 + CH340 serial bridge.
  *           Renamed from usbd_cdc_interface.c during module split for clarity.)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdarg.h>
#include "ring.h"
#include "servo.h"
#include "cmd.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* No-command timeout: hold position after this many ms without any command. */
#define CMD_TIMEOUT_MS   15000U  /* 15s: allow for camera warmup + model load */

/* Standard reply strings */
#define RPL_ERR "ERR\r\n"

/* Private variables ---------------------------------------------------------*/
/* Command line buffer (main loop) */
static char cmd_line[64];
static uint16_t cmd_len = 0U;

/* Safety state flags */
volatile uint8_t estop_active    = 0U;  /* E command: PWM stopped      */
volatile uint8_t timeout_active   = 0U; /* no command for 15s (see CMD_TIMEOUT_MS) */
volatile uint32_t last_cmd_tick   = 0U; /* HAL tick of last command    */

/* Home (hover) servo angles — single source shared by H (go-home) and L
 * (limit handshake). Must stay in sync with PC-side config.HOME_SERVO;
 * drift is caught by the L handshake + pc/tests/test_firmware_contract.py. */
static const uint16_t home_servo[6] = {101U, 122U, 167U, 97U, 90U, 90U}; /* 悬停位: 大臂122 小臂167 */

/* Private function prototypes -----------------------------------------------*/
static void CDC_Reply(const char *fmt, ...);
static uint8_t HexNibble(char c);
static uint8_t Cmd_CheckChecksum(char *line);
static void Cmd_Execute(const char *line);

/* Private functions ---------------------------------------------------------*/

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
  * @brief  Format and send a reply string to the PC over USART3.
  * @param  fmt  printf-style format string (max 95 chars output)
  */
static void CDC_Reply(const char *fmt, ...)
{
  char reply[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(reply, sizeof(reply), fmt, ap);
  va_end(ap);
  CDC_SendString(reply);
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
  uint16_t a;
  int ia, ib;

  if ((line[0] == 'M') || (line[0] == 'm'))
  {
    if (estop_active != 0U)
    {
      CDC_SendString(RPL_ERR);
      return;
    }
    if ((line[1] == 'A') || (line[1] == 'a'))
    {
      if (sscanf(line + 4, "%d", &ia) == 1)
      {
        if (ia < 0) { CDC_SendString(RPL_ERR); return; }
        a = (uint16_t)ia;
        Servo_SetAll(a);
        /* Note: echoes REQUESTED angle, not per-servo clamped values.
         * MALL is intentionally kept simple; use individual M commands
         * when per-joint echo accuracy matters (回显铁律). */
        CDC_Reply("OK MALL %u\r\n", a);
      }
      else { CDC_SendString(RPL_ERR); }
    }
    else
    {
      if (sscanf(line + 1, "%d %d", &ia, &ib) == 2)
      {
        if ((ia >= 1) && (ia <= 6) && (ib >= 0))
        {
          uint8_t c = (uint8_t)(ia - 1U);
          uint16_t t = ClampJoint(c, ib);
          /* Immediate single-axis override, four-piece (ADR-3 §4):
           * target+current snap prevents reversing toward a stale G target,
           * vel reset prevents a residual-velocity spike, settled restarts
           * the DONE window. Other axes' trapezoids are untouched. */
          axis[c].target  = t;
          axis[c].subgoal = t;     /* M jumps whole-hog: no stepping */
          axis[c].dwell   = 0U;
          axis[c].current = (float)t;
          axis[c].vel     = 0.0f;
          axis[c].settled = 0U;
          Servo_SetAngle(c, t);
          CDC_Reply("OK M%u %u\r\n", (uint16_t)ia, t);
        }
        else { CDC_SendString(RPL_ERR); }
      }
      else { CDC_SendString(RPL_ERR); }
    }
  }
  else if ((line[0] == 'I') || (line[0] == 'i'))
  {
    CDC_Reply("IR1=%u IR2=%u\r\n",
              (uint16_t)HAL_GPIO_ReadPin(IR1_GPIO_PORT, IR1_GPIO_PIN),
              (uint16_t)HAL_GPIO_ReadPin(IR2_GPIO_PORT, IR2_GPIO_PIN));
  }
  else if ((line[0] == 'S') || (line[0] == 's'))
  {
    /* Reports the ramped CURRENT position (rounded), not the raw target:
     * during motion this shows true commanded-so-far values. */
    CDC_Reply("S1=%u S2=%u S3=%u S4=%u S5=%u S6=%u\r\n",
              (uint16_t)(axis[0].current + 0.5f), (uint16_t)(axis[1].current + 0.5f),
              (uint16_t)(axis[2].current + 0.5f), (uint16_t)(axis[3].current + 0.5f),
              (uint16_t)(axis[4].current + 0.5f), (uint16_t)(axis[5].current + 0.5f));
  }
  else if ((line[0] == 'G') || (line[0] == 'g'))
  {
    int gv[6];
    uint8_t gi;
    uint8_t bad = 0U;
    if (estop_active != 0U) { CDC_SendString(RPL_ERR); return; }
    if (sscanf(line + 1, "%d %d %d %d %d %d",
               &gv[0], &gv[1], &gv[2], &gv[3], &gv[4], &gv[5]) == 6)
    {
      for (gi = 0U; gi < 6U; gi++)
      {
        if (gv[gi] < 0) { bad = 1U; }
      }
      if (bad == 0U)
      {
        for (gi = 0U; gi < 6U; gi++)
        {
          Axis_SetTarget(gi, ClampJoint(gi, gv[gi]), 1U);
        }
        motion_done = 0U;
        Servo_NoteMotionStart();  /* after all Axis_SetTarget calls */
        CDC_Reply("OK G %u %u %u %u %u %u\r\n",
                  axis[0].target, axis[1].target, axis[2].target,
                  axis[3].target, axis[4].target, axis[5].target);
      }
      else { CDC_SendString(RPL_ERR); }
    }
    else { CDC_SendString(RPL_ERR); }
  }
  else if ((line[0] == 'Q') || (line[0] == 'q'))
  {
    if (motion_done != 0U) { CDC_SendString("DONE\r\n"); }
    else                   { CDC_SendString("BUSY\r\n"); }
  }
  else if ((line[0] == 'H') || (line[0] == 'h'))
  {
    /* D1: home_servo[] is defined at file scope (shared with the L handshake).
     * It corresponds to PC-side config.HOME_SERVO / JOINT_HOME + JOINT_OFFSET.
     * Changing HOME requires sync on BOTH sides. */
    uint8_t hi;
    uint8_t was_estop = estop_active;
    estop_active = 0U;
    for (hi = 0U; hi < 6U; hi++)
    {
      Axis_SetTarget(hi, home_servo[hi], 0U);
      if (was_estop != 0U)
      {
        /* Post-E the physical position is UNKNOWN (open-loop servos lost
         * position when PWM stopped). Never glide from a stale current.
         * Reset bookkeeping to HOME and leave PWM OFF: the PC-side
         * soft_start re-enables axes one by one (200 ms apart). */
        axis[hi].current = (float)home_servo[hi];
      }
      else
      {
        /* Normal H: current is trustworthy -> glide home via trapezoid. */
        motion_done = 0U;
      }
    }
    if (was_estop == 0U) { Servo_NoteMotionStart(); }
    CDC_SendString("OK H\r\n");
  }
  else if ((line[0] == 'E') || (line[0] == 'e'))
  {
    uint8_t ei;
    estop_active = 1U;
    Servo_DisableAll();
    for (ei = 0U; ei < 6U; ei++)
    {
      axis[ei].vel     = 0.0f;   /* freeze motion state */
      axis[ei].stagger = 0U;
    }
    motion_done = 1U;            /* nothing is moving; timeout protection resumes */
    CDC_Reply("OK E\r\n");
  }
  else if ((line[0] == 'L') || (line[0] == 'l'))
  {
    /* Runtime handshake: report the COMPILED limit/home tables so the host
     * verifies its config at connect (zero cross-boundary drift; see
     * docs/architecture/6-工程规范与最佳实践.md). Format:
     *   OK L MIN m0..m5 MAX x0..x5 HOME h0..h5
     * joint_min/joint_max are servo-domain clamps (servo.c). */
    CDC_Reply("OK L MIN %u %u %u %u %u %u MAX %u %u %u %u %u %u "
              "HOME %u %u %u %u %u %u\r\n",
              joint_min[0], joint_min[1], joint_min[2],
              joint_min[3], joint_min[4], joint_min[5],
              joint_max[0], joint_max[1], joint_max[2],
              joint_max[3], joint_max[4], joint_max[5],
              home_servo[0], home_servo[1], home_servo[2],
              home_servo[3], home_servo[4], home_servo[5]);
  }
  else
  {
    CDC_SendString(RPL_ERR);
  }
}

/* Exported functions --------------------------------------------------------*/

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
  if (timeout_active != 0U) { return; }

  /* While a commanded motion runs, the idle-timeout is suppressed: normally
   * Q polling refreshes last_cmd_tick anyway, and suppression covers the
   * PC-died-mid-motion case so the arm completes its planned trajectory
   * instead of collapsing mid-swing. MOTION_HARD_CAP_MS (enforced in
   * Servo_RampStep) guarantees this suppression cannot last forever. */
  if (motion_done == 0U) { return; }

  if ((HAL_GetTick() - last_cmd_tick) > CMD_TIMEOUT_MS)
  {
    timeout_active = 1U;
    Servo_DisableAll();
  }
}
