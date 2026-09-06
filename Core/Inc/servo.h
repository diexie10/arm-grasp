/**
  ******************************************************************************
  * @file    servo.h
  * @brief   Servo control interface: PWM output, axis state, trapezoidal ramp.
  *          (v2.1: extracted from usbd_cdc_interface.c during module split.)
  ******************************************************************************
  */

#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

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
                        step axes (S1/S2/S3) advance it in adaptive bites */
  uint8_t  dwell;    /* S3: ticks parked at a bite edge before the next */
} ServoAxis;

#define AXIS_IDLE {90U, 90.0f, 0.0f, 0U, 0U, 90U, 0U}

/* Joint angle limits (servo degrees, 0-270 = 0.5-2.5ms PWM on 270-deg servos).
 * Conservative 0-180 band until stage-6 measurement (see docs).
 * D2: manually synchronized with PC-side config.JOINT_MIN / config.JOINT_MAX
 * (protocol boundary cannot auto-sync). */
extern const uint16_t joint_min[6];
extern const uint16_t joint_max[6];

/* Horn-hole compensation (per-axis mechanical trim). Servo horn drilling is
 * imprecise: the link sits true only at a physical angle offset from logical.
 * physical = logical + trim; applied ONLY at pulse conversion, so limits,
 * HOME table and echoes all stay in logical domain. Fill per axis on
 * assembly-day measurement.
 * A9: servo_trim[i] == -joint_min[i] is a calibration coincidence (trim pulls
 * zero point to lower limit). Semantics differ (horn offset vs travel bound);
 * on re-calibration both tables MUST be rechecked — do NOT merge. */
extern const int8_t servo_trim[6];

/* Per-channel PWM enable flags. A channel is started LAZILY by its first
 * Servo_SetAngle call, so power-up and timeout recovery draw zero current
 * until the PC explicitly commands that servo - this is what makes the
 * PC-side soft start (M1..M6, 200 ms apart) genuinely sequence the inrush.
 * Cleared by Servo_DisableAll (E-stop / timeout). */
extern uint8_t servo_enabled[6];

/* Per-axis step mode: 1 = step-and-dwell (S1 base + S2 shoulder + S3 elbow),
 * 0 = continuous. */
extern const uint8_t axis_steps[6];

/* Motion state (written by cmd module G/H/E/M commands, read by Servo_RampStep). */
extern ServoAxis axis[6];
extern uint8_t  motion_done;       /* Q query reply state; only G clears */
extern uint32_t move_start_tick;   /* HAL tick when G was accepted       */
extern uint32_t motion_deadline_ms; /* bite-aware deadline (ms from start) */

/* Exported functions --------------------------------------------------------*/

/* Clamp joint angle to [joint_min, joint_max]. */
uint16_t ClampJoint(uint8_t ch, int32_t v);

/* Adaptive stepping: set target, compute first bite, zero velocity. */
void Axis_SetTarget(uint8_t i, uint16_t target, uint8_t apply_stagger);

/* Set servo angle (clamped) and lazily start its PWM channel. */
void Servo_SetAngle(uint8_t ch, uint16_t angle);

/* Set all six servo angles (each clamped independently). */
void Servo_SetAll(uint16_t angle);

/* Stop all PWM outputs (servos go limp / unpowered). */
void Servo_DisableAll(void);

/* Called from main loop: gradual servo ramp toward targets (H command). */
void Servo_RampStep(void);

/* Estimate worst-case motion duration (ms) for the current G command.
 * Must be called AFTER all six Axis_SetTarget calls. */
uint32_t Servo_EstimateMotionMs(void);

/* Record motion start and compute bite-aware deadline.
 * Called after all Axis_SetTarget calls in the G command handler. */
void Servo_NoteMotionStart(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
