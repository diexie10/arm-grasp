/**
  ******************************************************************************
  * @file    servo.c
  * @brief   Servo control: PWM output, axis state, trapezoidal ramp.
  *          (v2.1: extracted from usbd_cdc_interface.c during module split.)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "servo.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Overtravel margin beyond the logical 0..SERVO_MAX_ANGLE band: the servo's
 * mechanical travel extends past the conservative band edge (user-verified),
 * so the trim may push slightly past it. NOTE: this requires the pulse math
 * below to stay fully signed - an unsigned cast wraps negative phys into
 * CCR > ARR (constant-high output = signal loss = servo holds). */
#define TRIM_OVERTRAVEL_DEG  30

/* Ramp timing and motion constants */
#define RAMP_TICK_MS       20U    /* x1 = 50 Hz tick, matches servo refresh */
#define ACC_PER_TICK     0.006f   /* x2500 = 15 deg/s^2 (50% of max vel)     */
#define REACH_TOL         0.01f   /* arrival threshold, deg                  */
#define SETTLE_TICKS         3U   /* consecutive at-target ticks before DONE */
#define STAGGER_TICKS        3U   /* axis i starts i*3 ticks later = 60 ms   */
#define MOTION_HARD_CAP_MS 15000U /* force-complete window after G; guarantees
                                   the idle-timeout suppression cannot last */

/* Per-axis cruise/crawl caps (deg/tick; x50 = deg/s). One global MAX_VEL=30
 * deg/s made S3 (elbow, loaded with forearm+wrist) hunt visibly: the command
 * stream outran its loaded slew (~25-35 deg/s), the servo overshot and
 * corrected every frame. User verdict 2026-08-26: even 20 deg/s cruise shook;
 * precision > speed. S3 now UNIFORM 7.5 deg/s (max==min kills the trapezoid,
 * soft-start 0->0.15 over ~0.5 s then constant - already proven smooth as
 * the other axes' crawl tail). */
static const float max_vel[6] = {0.6f, 0.6f, 0.15f, 0.6f, 0.6f, 0.6f}; /* x50 = 30/30/7.5/30/30/30 deg/s */
static const float min_vel[6] = {0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f}; /* crawl floor; 0.15 deg/tick = 1.67 us/tick pulse, above deadband */

/* Step-and-dwell profile (user request 2026-08-26): continuous tracking kept
 * the loaded joints hunting even at 7.5 deg/s cruise. Discrete bites instead:
 * glide, park STEP_DWELL_TICKS, repeat - each setpoint lets the servo settle
 * dead before the next. Precision >> speed. Adaptive bite width: 15 deg
 * mid-travel, 10 deg approach, 5 deg final (user-tuned).
 * Applies to: S1 base + S3 elbow (both loaded, both hunted). */
#define STEP_NEAR_DEG      5.0f
#define STEP_MID_DEG       10.0f
#define STEP_FAR_DEG       15.0f
#define BITE_FAR_THR       20.0f  /* remaining above -> FAR bite */
#define BITE_MID_THR       10.0f  /* remaining above -> MID bite */
#define STEP_DWELL_TICKS   33U   /* x20ms = 0.66 s park between bites (user: 2/3 of the original 1 s) */

/* Safety state (written by cmd module, read by servo module) */
extern volatile uint8_t estop_active;

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

/* Private variables ---------------------------------------------------------*/
/* Per-channel PWM enable flags */
uint8_t servo_enabled[6] = {0U, 0U, 0U, 0U, 0U, 0U};

/* Joint angle limits (servo degrees, 0-270 = 0.5-2.5ms PWM on 270-deg servos).
 * Conservative 0-180 band until stage-6 measurement (see docs).
 * D2: manually synchronized with PC-side config.JOINT_MIN / config.JOINT_MAX
 * (protocol boundary cannot auto-sync). */
const uint16_t joint_min[6] = {11U, 3U, 5U, 7U, 0U, 0U};
const uint16_t joint_max[6] = {191U, 183U, 167U, 187U, 270U, 270U};

/* Horn-hole compensation (per-axis mechanical trim). Servo horn drilling is
 * imprecise: the link sits true only at a physical angle offset from logical.
 * physical = logical + trim; applied ONLY at pulse conversion, so limits,
 * HOME table and echoes all stay in logical domain. Fill per axis on
 * assembly-day measurement.
 * A9: servo_trim[i] == -joint_min[i] is a calibration coincidence (trim pulls
 * zero point to lower limit). Semantics differ (horn offset vs travel bound);
 * on re-calibration both tables MUST be rechecked — do NOT merge. */
const int8_t servo_trim[6] = {-11, -3, -5, -7, 0, 0};

/* Per-axis step mode: 1 = step-and-dwell (S1 base + S3 elbow), 0 = continuous */
const uint8_t axis_steps[6] = {1U, 0U, 1U, 0U, 0U, 0U};

/* Motion state */
ServoAxis axis[6] = {
  AXIS_IDLE, AXIS_IDLE, AXIS_IDLE, AXIS_IDLE, AXIS_IDLE, AXIS_IDLE,
};
uint8_t  motion_done     = 1U;  /* Q query reply state; only G clears */
uint32_t move_start_tick = 0U;  /* HAL tick when G was accepted       */
static uint32_t ramp_tick       = 0U;  /* RampStep pacing                    */

/* Private function prototypes -----------------------------------------------*/
static void Servo_WritePhys(uint8_t ch, float phys);
static void Servo_SetAngleF(uint8_t ch, float angle_deg);
static float StepBiteSize(float remain_abs);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Bite width by remaining distance. Distances not rates - no multiply-back.
  */
static float StepBiteSize(float remain_abs)
{
  if (remain_abs > BITE_FAR_THR) { return STEP_FAR_DEG; }
  if (remain_abs > BITE_MID_THR) { return STEP_MID_DEG; }
  return STEP_NEAR_DEG;
}

/**
  * @brief  Core PWM output: trim, overtravel clamp, pulse calc, CCR write,
  *         lazy PWM start. ALL math must stay signed: negative phys → unsigned
  *         CCR wraps to > ARR (constant-high = signal loss = servo holds).
  * @param  ch    channel index 0..5
  * @param  phys  physical angle (deg) = logical + trim
  */
static void Servo_WritePhys(uint8_t ch, float phys)
{
  uint16_t cmp;

  phys += (float)servo_trim[ch];
  if (phys < -(float)TRIM_OVERTRAVEL_DEG)                  { phys = -(float)TRIM_OVERTRAVEL_DEG; }
  if (phys > (float)SERVO_MAX_ANGLE + (float)TRIM_OVERTRAVEL_DEG) { phys = (float)SERVO_MAX_ANGLE + (float)TRIM_OVERTRAVEL_DEG; }
  cmp = (uint16_t)((float)SERVO_MIN_PULSE +
        phys * ((float)SERVO_MAX_PULSE - (float)SERVO_MIN_PULSE) / (float)SERVO_MAX_ANGLE);

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
  if (ch >= 6U) return;
  Servo_WritePhys(ch, angle_deg);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Set servo angle (clamped) and lazily start its PWM channel.
  * @param  ch     channel index 0..5
  * @param  angle  requested servo angle (degrees)
  */
void Servo_SetAngle(uint8_t ch, uint16_t angle)
{
  if (ch >= 6U) return;
  Servo_WritePhys(ch, (float)angle);
}

/**
  * @brief  Clamp joint angle to [joint_min, joint_max]. Signed input preserved.
  * @param  ch  channel index 0..5
  * @param  v   raw requested angle (signed to catch negative from sscanf)
  * @return clamped angle within joint limits
  */
uint16_t ClampJoint(uint8_t ch, int32_t v)
{
  uint16_t t = (uint16_t)v;
  if (t < joint_min[ch]) { t = joint_min[ch]; }
  if (t > joint_max[ch]) { t = joint_max[ch]; }
  return t;
}

/**
  * @brief  Adaptive stepping: set target, compute first bite from current
  *         position, zero velocity, reset settled. G uses apply_stagger=1,
  *         H uses apply_stagger=0 (post-estop unknown position).
  * @param  i              axis index 0..5
  * @param  target         goal angle (servo domain, already clamped)
  * @param  apply_stagger  1 = stagger axis start by i*STAGGER_TICKS
  */
void Axis_SetTarget(uint8_t i, uint16_t target, uint8_t apply_stagger)
{
  axis[i].target = target;
  if (axis_steps[i] != 0U)
  {
    /* Stepping mode: first bite = one adaptive step from CURRENT toward target. */
    float r  = (float)target - axis[i].current;
    float ra = fabsf(r);
    float b  = StepBiteSize(ra);
    if (r > b)       { axis[i].subgoal = (uint16_t)(axis[i].current + b); }
    else if (r < -b) { axis[i].subgoal = (uint16_t)(axis[i].current - b); }
    else             { axis[i].subgoal = target; }
  }
  else { axis[i].subgoal = target; }
  axis[i].dwell   = 0U;
  axis[i].vel     = 0.0f;   /* arrives between waypoints (PC waits for DONE) */
  axis[i].settled = 0U;
  /* Stagger only axes that actually move: an unmoved axis holding a
   * stagger counter would delay DONE by up to i*STAGGER_TICKS. */
  if (apply_stagger != 0U)
  {
    axis[i].stagger =
      ((target != (uint16_t)(axis[i].current + 0.5f)))
        ? (uint8_t)(i * STAGGER_TICKS) : 0U;
  }
  else
  {
    axis[i].stagger = 0U;
  }
}

/**
  * @brief  Set all six servo angles (each clamped independently).
  * @param  angle  requested angle
  * @note   Echoes the REQUESTED angle; query 'S' for actual positions.
  */
void Servo_SetAll(uint16_t angle)
{
  uint8_t i;
  for (i = 0U; i < 6U; i++)
  {
    uint16_t t = ClampJoint(i, (int32_t)angle);
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
void Servo_DisableAll(void)
{
  uint8_t i;
  for (i = 0U; i < 6U; i++)
  {
    HAL_TIM_PWM_Stop(servo_map[i].tim, servo_map[i].chan);
    servo_enabled[i] = 0U;
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
