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
#define CMD_TIMEOUT_MS   15000U  /* 15s: allow for camera warmup + model load */

/* Private variables ---------------------------------------------------------*/
/* RX ring buffer (fed by UART_RxByte from USART3 ISR) */
static volatile uint16_t rx_head = 0U;   /* next write position (ISR)   */
static volatile uint16_t rx_tail = 0U;   /* next read position (main)   */
static uint8_t rx_ring[CDC_RX_BUFFER_SIZE];

/* Command line buffer (main loop) */
static char cmd_line[64];
static uint16_t cmd_len = 0U;

/* Per-channel PWM enable flags. A channel is started LAZILY by its first
 * Servo_SetAngle call, so power-up and timeout recovery draw zero current
 * until the PC explicitly commands that servo - this is what makes the
 * PC-side soft start (M1..M6, 200 ms apart) genuinely sequence the inrush.
 * Cleared by Servo_DisableAll (E-stop / timeout). */
static uint8_t servo_enabled[6] = {0U, 0U, 0U, 0U, 0U, 0U};

/* Timer handles (defined in main.c). */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* Channel → timer/channel mapping (single source of truth). */
typedef struct { TIM_HandleTypeDef *tim; uint32_t chan; } ServoChannel;
static const ServoChannel servo_map[6] = {
  {&htim1, TIM_CHANNEL_1},  /* ch0 = J1 = PA8 */
  {&htim1, TIM_CHANNEL_2},  /* ch1 = J2 = PA9 */
  {&htim1, TIM_CHANNEL_3},  /* ch2 = J3 = PA10 */
  {&htim2, TIM_CHANNEL_1},  /* ch3 = J4 = PA0 */
  {&htim2, TIM_CHANNEL_2},  /* ch4 = J5 = PA1 */
  {&htim2, TIM_CHANNEL_3},  /* ch5 = J6 = PA2 */
};

/* Universal motion executor (ADR-3). G/M/H commands manipulate per-axis
 * state; Servo_RampStep drives trapezoidal motion toward targets.
 * Main-loop-only access (ISR never touches), so no volatile needed.
 * Constants carry multiply-back verification - unit-conversion incidents
 * happened twice in review, so every derived value shows its check. */
typedef struct {
  uint16_t target;   /* goal angle, servo domain, clamped to joint limits */
  float    current;  /* current angle, deg. Float is mandatory: integer vel
                        made the trapezoid collapse into velocity steps */
  float    vel;      /* signed velocity, deg/tick */
  uint8_t  settled;  /* consecutive at-target ticks (saturating at 255) */
  uint8_t  stagger;  /* start-delay ticks; G staggers axis i by i*STAGGER_TICKS */
  uint16_t subgoal;  /* waypoint the ramp actually chases; == target except
                        step axes (S1/S3) advance it in adaptive bites */
  uint8_t  dwell;    /* S3: ticks parked at a bite edge before the next */
} ServoAxis;
static ServoAxis axis[6] = {
  {90U, 90.0f, 0.0f, 0U, 0U, 90U, 0U}, {90U, 90.0f, 0.0f, 0U, 0U, 90U, 0U},
  {90U, 90.0f, 0.0f, 0U, 0U, 90U, 0U}, {90U, 90.0f, 0.0f, 0U, 0U, 90U, 0U},
  {90U, 90.0f, 0.0f, 0U, 0U, 90U, 0U}, {90U, 90.0f, 0.0f, 0U, 0U, 90U, 0U},
};

#define RAMP_TICK_MS       20U    /* x1 = 50 Hz tick, matches servo refresh */
/* Per-axis cruise/crawl caps (deg/tick; x50 = deg/s). One global MAX_VEL=30
 * deg/s made S3 (elbow, loaded with forearm+wrist) hunt visibly: the command
 * stream outran its loaded slew (~25-35 deg/s), the servo overshot and
 * corrected every frame. User verdict 2026-08-26: even 20 deg/s cruise shook;
 * precision > speed. S3 now UNIFORM 7.5 deg/s (max==min kills the trapezoid,
 * soft-start 0->0.15 over ~0.5 s then constant - already proven smooth as
 * the other axes' crawl tail). */
static const float max_vel[6] = {0.6f, 0.6f, 0.15f, 0.6f, 0.6f, 0.6f}; /* x50 = 30/30/7.5/30/30/30 deg/s */
static const float min_vel[6] = {0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f}; /* crawl floor; 0.15 deg/tick = 1.67 us/tick pulse, above deadband */
#define ACC_PER_TICK     0.006f   /* x2500 = 15 deg/s^2 (50% of max vel)     */
#define REACH_TOL         0.01f   /* arrival threshold, deg                  */
#define SETTLE_TICKS         3U   /* consecutive at-target ticks before DONE */
#define STAGGER_TICKS        3U   /* axis i starts i*3 ticks later = 60 ms   */
#define MOTION_HARD_CAP_MS 15000U /* force-complete window after G; guarantees
                                   the idle-timeout suppression cannot last */
/* Step-and-dwell profile (user request 2026-08-26): continuous tracking kept
 * the loaded joints hunting even at 7.5 deg/s cruise. Discrete bites instead:
 * glide, park STEP_DWELL_TICKS, repeat - each setpoint lets the servo settle
 * dead before the next. Precision >> speed. Adaptive bite width: 15 deg
 * mid-travel, 10 deg approach, 5 deg final (user-tuned).
 * Applies to: S1 base + S3 elbow (both loaded, both hunted). */
static const uint8_t axis_steps[6] = {1U, 0U, 1U, 0U, 0U, 0U};
#define STEP_NEAR_DEG      5.0f
#define STEP_MID_DEG       10.0f
#define STEP_FAR_DEG       15.0f
#define BITE_FAR_THR       20.0f  /* remaining above -> FAR bite */
#define BITE_MID_THR       10.0f  /* remaining above -> MID bite */
#define STEP_DWELL_TICKS   33U   /* x20ms = 0.66 s park between bites (user: 2/3 of the original 1 s) */

/* Bite width by remaining distance. Distances not rates - no multiply-back. */
static float StepBiteSize(float remain_abs)
{
  if (remain_abs > BITE_FAR_THR) { return STEP_FAR_DEG; }
  if (remain_abs > BITE_MID_THR) { return STEP_MID_DEG; }
  return STEP_NEAR_DEG;
}

static uint8_t  motion_done     = 1U;  /* Q query reply state; only G clears */
static uint32_t move_start_tick = 0U;  /* HAL tick when G was accepted       */
static uint32_t ramp_tick       = 0U;  /* RampStep pacing                    */

/* Joint angle limits (servo degrees, 0-270 = 0.5-2.5ms PWM on 270-deg servos).
 * Conservative 0-180 band until stage-6 measurement (see docs). */
static const uint16_t joint_min[6] = {11U, 3U, 5U, 7U, 0U, 0U};
static const uint16_t joint_max[6] = {191U, 183U, 167U, 187U, 270U, 270U};

/* Horn-hole compensation (per-axis mechanical trim). Servo horn drilling is
 * imprecise: the link sits true only at a physical angle offset from logical.
 * physical = logical + trim; applied ONLY at pulse conversion, so limits,
 * HOME table and echoes all stay in logical domain. Fill per axis on
 * assembly-day measurement. */
static const int8_t servo_trim[6] = {-11, -3, -5, -7, 0, 0};

/* Overtravel margin beyond the logical 0..SERVO_MAX_ANGLE band: the servo's
 * mechanical travel extends past the conservative band edge (user-verified),
 * so the trim may push slightly past it. NOTE: this requires the pulse math
 * below to stay fully signed - an unsigned cast wraps negative phys into
 * CCR > ARR (constant-high output = signal loss = servo holds). */
#define TRIM_OVERTRAVEL_DEG  30

/* Safety state flags */
static volatile uint8_t estop_active    = 0U;  /* E command: PWM stopped      */
static volatile uint8_t timeout_active   = 0U; /* no command for 10s          */
static volatile uint32_t last_cmd_tick   = 0U; /* HAL tick of last command    */

/* Private function prototypes -----------------------------------------------*/
static void Servo_SetAngle(uint8_t ch, uint16_t angle);
static void Servo_SetAngleF(uint8_t ch, float angle_deg);
static void Servo_SetAll(uint16_t angle);
static void Servo_DisableAll(void);
static uint8_t HexNibble(char c);
static uint8_t Cmd_CheckChecksum(char *line);
static void Cmd_Execute(const char *line);

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
  int32_t phys;

  if (ch >= 6U) return;

  /* Angle -> pulse width -> compare value. Callers own clamping and the
   * axis[] bookkeeping; this is pure PWM output + lazy start.
   * Horn trim applied here (physical = logical + trim), allowed TRIM_OVERTRAVEL_DEG
   * past the logical band edges (servo travel verified wider than the band).
   * ALL math signed: an unsigned cast of negative phys wraps into CCR > ARR
   * (constant-high output = signal loss = servo holds position). */
  phys = (int32_t)angle + (int32_t)servo_trim[ch];
  if (phys < -TRIM_OVERTRAVEL_DEG)                  { phys = -TRIM_OVERTRAVEL_DEG; }
  if (phys > SERVO_MAX_ANGLE + TRIM_OVERTRAVEL_DEG) { phys = SERVO_MAX_ANGLE + TRIM_OVERTRAVEL_DEG; }
  cmp = (uint16_t)(SERVO_MIN_PULSE +
        (phys * ((int32_t)SERVO_MAX_PULSE - (int32_t)SERVO_MIN_PULSE)) / SERVO_MAX_ANGLE);

  __HAL_TIM_SET_COMPARE(servo_map[ch].tim, servo_map[ch].chan, cmp);

  /* Lazy start: only enable PWM on first command for this channel. */
  if ((servo_enabled[ch] == 0U) && (estop_active == 0U))
  {
    servo_enabled[ch] = 1U;
    HAL_TIM_PWM_Start(servo_map[ch].tim, servo_map[ch].chan);
  }
}

/**
  * @brief  Float-domain PWM write for the ramp: no integer-degree rounding.
  *         The ramp advances <1 deg/tick near arrival; rounding to whole
  *         degrees froze the pulse for seconds at a time (quantization stall).
  * @param  ch         channel index 0..5
  * @param  angle_deg  fractional servo angle (degrees)
  */
static void Servo_SetAngleF(uint8_t ch, float angle_deg)
{
  float phys;
  uint16_t cmp;

  if (ch >= 6U) return;

  /* Same trim + overtravel clamp as Servo_SetAngle, in float. ALL signed:
   * negative phys must survive to the cmp formula (see trap #27). */
  phys = angle_deg + (float)servo_trim[ch];
  if (phys < -(float)TRIM_OVERTRAVEL_DEG)                  { phys = -(float)TRIM_OVERTRAVEL_DEG; }
  if (phys > (float)SERVO_MAX_ANGLE + (float)TRIM_OVERTRAVEL_DEG) { phys = (float)SERVO_MAX_ANGLE + (float)TRIM_OVERTRAVEL_DEG; }
  cmp = (uint16_t)((float)SERVO_MIN_PULSE +
        phys * ((float)SERVO_MAX_PULSE - (float)SERVO_MIN_PULSE) / (float)SERVO_MAX_ANGLE);

  __HAL_TIM_SET_COMPARE(servo_map[ch].tim, servo_map[ch].chan, cmp);

  if ((servo_enabled[ch] == 0U) && (estop_active == 0U))
  {
    servo_enabled[ch] = 1U;
    HAL_TIM_PWM_Start(servo_map[ch].tim, servo_map[ch].chan);
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
    uint16_t t = angle;
    if (t < joint_min[i]) { t = joint_min[i]; }
    if (t > joint_max[i]) { t = joint_max[i]; }
    /* Same immediate-override semantics as single M (four-piece). */
    axis[i].target  = t;
    axis[i].current = (float)t;
    axis[i].vel     = 0.0f;
    axis[i].settled = 0U;
    Servo_SetAngle(i, t);
  }
}

/**
  * @brief  Stop all PWM outputs (servos go limp / unpowered).
  */
static void Servo_DisableAll(void)
{
  uint8_t i;
  for (i = 0U; i < 6U; i++)
  {
    HAL_TIM_PWM_Stop(servo_map[i].tim, servo_map[i].chan);
    servo_enabled[i] = 0U;
  }
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
  uint16_t a;
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
        /* Note: echoes REQUESTED angle, not per-servo clamped values.
         * MALL is intentionally kept simple; use individual M commands
         * when per-joint echo accuracy matters (回显铁律). */
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
          uint8_t c = (uint8_t)(ia - 1U);
          uint16_t t = (uint16_t)ib;
          if (t < joint_min[c]) { t = joint_min[c]; }
          if (t > joint_max[c]) { t = joint_max[c]; }
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
          sprintf(reply, "OK M%u %u\r\n", (uint16_t)ia, t);
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
    /* Reports the ramped CURRENT position (rounded), not the raw target:
     * during motion this shows true commanded-so-far values. */
    sprintf(reply, "S1=%u S2=%u S3=%u S4=%u S5=%u S6=%u\r\n",
            (uint16_t)(axis[0].current + 0.5f), (uint16_t)(axis[1].current + 0.5f),
            (uint16_t)(axis[2].current + 0.5f), (uint16_t)(axis[3].current + 0.5f),
            (uint16_t)(axis[4].current + 0.5f), (uint16_t)(axis[5].current + 0.5f));
    CDC_SendString(reply);
  }
  else if ((line[0] == 'G') || (line[0] == 'g'))
  {
    int gv[6];
    uint8_t gi;
    uint8_t bad = 0U;
    if (estop_active != 0U) { CDC_SendString("ERR\r\n"); return; }
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
          uint16_t t = (uint16_t)gv[gi];
          if (t < joint_min[gi]) { t = joint_min[gi]; }
          if (t > joint_max[gi]) { t = joint_max[gi]; }
          axis[gi].target  = t;
          if (axis_steps[gi] != 0U)
          {
            /* Stepping mode: first bite = one adaptive step from CURRENT
             * toward t; RampStep dwells+advances the rest of the way. */
            float r = (float)t - axis[gi].current;
            float ra = (r > 0.0f) ? r : -r;
            float b  = StepBiteSize(ra);
            if (r > b)       { axis[gi].subgoal = (uint16_t)(axis[gi].current + b); }
            else if (r < -b) { axis[gi].subgoal = (uint16_t)(axis[gi].current - b); }
            else             { axis[gi].subgoal = t; }
          }
          else { axis[gi].subgoal = t; }
          axis[gi].dwell   = 0U;
          axis[gi].vel     = 0.0f;   /* G arrives between waypoints (PC waits
                                        for DONE); zeroing vel is predictable */
          axis[gi].settled = 0U;
          /* Stagger only axes that actually move: an unmoved axis holding a
           * stagger counter would delay DONE by up to i*STAGGER_TICKS. */
          axis[gi].stagger =
            ((t != (uint16_t)(axis[gi].current + 0.5f)))
              ? (uint8_t)(gi * STAGGER_TICKS) : 0U;
        }
        motion_done     = 0U;
        move_start_tick = HAL_GetTick();
        sprintf(reply, "OK G %u %u %u %u %u %u\r\n",
                axis[0].target, axis[1].target, axis[2].target,
                axis[3].target, axis[4].target, axis[5].target);
        CDC_SendString(reply);
      }
      else { CDC_SendString("ERR\r\n"); }
    }
    else { CDC_SendString("ERR\r\n"); }
  }
  else if ((line[0] == 'Q') || (line[0] == 'q'))
  {
    if (motion_done != 0U) { CDC_SendString("DONE\r\n"); }
    else                   { CDC_SendString("BUSY\r\n"); }
  }
  else if ((line[0] == 'H') || (line[0] == 'h'))
  {
    static const uint16_t home_servo[6] = {101U, 122U, 167U, 97U, 90U, 90U}; /* 悬停位: 大臂122 小臂167 */
    uint8_t hi;
    uint8_t was_estop = estop_active;
    estop_active = 0U;
    for (hi = 0U; hi < 6U; hi++)
    {
      axis[hi].target  = home_servo[hi];
      if (axis_steps[hi] != 0U)
      {
        /* Same adaptive stepping as G: first bite from current toward HOME */
        float r  = (float)home_servo[hi] - axis[hi].current;
        float ra = (r > 0.0f) ? r : -r;
        float b  = StepBiteSize(ra);
        if (r > b)       { axis[hi].subgoal = (uint16_t)(axis[hi].current + b); }
        else if (r < -b) { axis[hi].subgoal = (uint16_t)(axis[hi].current - b); }
        else             { axis[hi].subgoal = home_servo[hi]; }
      }
      else { axis[hi].subgoal = home_servo[hi]; }
      axis[hi].dwell   = 0U;
      axis[hi].vel     = 0.0f;
      axis[hi].settled = 0U;
      axis[hi].stagger = 0U;
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
    if (was_estop == 0U) { move_start_tick = HAL_GetTick(); }
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

/**
  * @brief  Trapezoidal motion executor (ADR-3 §3). Called from main loop.
  *         Per axis: accelerate to MAX_VEL, cruise, decelerate along
  *         v^2/(2a) distance, settle SETTLE_TICKS at target.
  *         Non-blocking; paced by HAL_GetTick every RAMP_TICK_MS.
  */
void Servo_RampStep(void)
{
  uint8_t i;
  uint8_t all_settled = 1U;

  if (estop_active != 0U) { return; }
  if ((HAL_GetTick() - ramp_tick) < RAMP_TICK_MS) { return; }
  ramp_tick = HAL_GetTick();

  for (i = 0U; i < 6U; i++)
  {
    ServoAxis *ax = &axis[i];
    float dist;
    float decel_dist;
    float step;

    if (ax->stagger > 0U)
    {
      ax->stagger--;
      all_settled = 0U;          /* motion pending: not settled yet */
      continue;
    }

    dist = (float)ax->subgoal - ax->current;
    if (dist < 0.0f) { dist = -dist; }

    if (dist < REACH_TOL)
    {
      float remain = (float)ax->target - (float)ax->subgoal;
      if ((remain > REACH_TOL) || (remain < -REACH_TOL))
      {
        /* parked short of the final target */
        if (axis_steps[i] != 0U)
        {
          /* bite edge: dwell out, then advance one step toward target */
          ax->vel = 0.0f;
          if (ax->dwell < STEP_DWELL_TICKS) { ax->dwell++; all_settled = 0U; continue; }
          ax->dwell = 0U;
          {
            float r_abs = (remain > 0.0f) ? remain : -remain;
            float bite  = StepBiteSize(r_abs);
            if (remain > 0.0f) { ax->subgoal += (r_abs > bite) ? (uint16_t)bite : (uint16_t)(r_abs + 0.5f); }
            else               { ax->subgoal -= (r_abs > bite) ? (uint16_t)bite : (uint16_t)(r_abs + 0.5f); }
          }
          all_settled = 0U;
          continue;
        }
        /* non-S3 stale subgoal: heal (should not happen) */
        ax->subgoal = ax->target;
        all_settled = 0U;
        continue;
      }
      ax->vel = 0.0f;
      if (ax->settled < 255U) { ax->settled++; }
      if (ax->settled < SETTLE_TICKS) { all_settled = 0U; }
      continue;
    }

    all_settled = 0U;

    /* Adaptive braking distance v^2/(2a), recomputed every tick - a fixed
     * decel window was proven non-convergent in review. */
    decel_dist = (ax->vel * ax->vel) / (2.0f * ACC_PER_TICK);
    if (dist > decel_dist)
    {
      if (ax->vel < max_vel[i]) { ax->vel += ACC_PER_TICK; }   /* accel / cruise */
    }
    else
    {
      ax->vel -= ACC_PER_TICK;                              /* decelerate */
      if (ax->vel < min_vel[i]) { ax->vel = min_vel[i]; }   /* crawl floor above deadband */
    }

    step = (ax->vel < dist) ? ax->vel : dist;               /* no overshoot */
    if ((float)ax->subgoal >= ax->current) { ax->current += step; }
    else                                   { ax->current -= step; }
    Servo_SetAngleF(i, ax->current);                        /* float domain: no degree-quantization stall */
  }

  motion_done = all_settled;

  /* Hard cap: if motion somehow never completes (stale target unreachable),
   * force-complete so idle-timeout protection resumes. */
  if ((motion_done == 0U) &&
      ((HAL_GetTick() - move_start_tick) > MOTION_HARD_CAP_MS))
  {
    for (i = 0U; i < 6U; i++)
    {
      axis[i].vel     = 0.0f;
      axis[i].settled = SETTLE_TICKS;
      axis[i].subgoal = axis[i].target;
      axis[i].dwell   = 0U;
    }
    motion_done = 1U;
  }
}
