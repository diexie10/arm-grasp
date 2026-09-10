#pragma once
/* mahony.h — Mahony AHRS filter, Arduino-free.
 *
 * Standalone header: depends only on <math.h>.  Can be compiled on both
 * ESP32 (via main.cpp) and host (via clang++/gcc for parity tests).
 *
 * Exact C++ port of pc/glove/imu_filter.py — same quaternion convention
 * [w,x,y,z], same gravity reference [0,0,1], same ZYX Euler extraction
 * with pitch clamped to [-90,90].
 */

#ifndef MAHONY_H
#define MAHONY_H

#include <math.h>

/* ── Platform-adaptive helpers ────────────────────────────────────────
 * When compiled inside Arduino (DEG_TO_RAD / degrees() already defined)
 * or on a host compiler (we provide fallbacks). */
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943296f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082321f
#endif

#ifndef MAHONY_DEGREES
#ifdef ARDUINO
/* Arduino core already provides degrees(). */
#define MAHONY_DEGREES(rad) degrees(rad)
#else
static inline float mahony_degrees_(float rad) {
    return rad * 57.295779513082321f;
}
#define MAHONY_DEGREES(rad) mahony_degrees_(rad)
#endif
#endif

/* ── State ─────────────────────────────────────────────────────────── */

struct MahonyState {
    float q[4];           // quaternion [w, x, y, z]
    float integral_fb[3]; // integral feedback [x, y, z]
};

/* ── API ───────────────────────────────────────────────────────────── */

static inline void mahony_init(MahonyState *s) {
    s->q[0] = 1.0f; s->q[1] = 0.0f; s->q[2] = 0.0f; s->q[3] = 0.0f;
    s->integral_fb[0] = 0.0f;
    s->integral_fb[1] = 0.0f;
    s->integral_fb[2] = 0.0f;
}

static inline void mahony_normalize(float q[4]) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm < 1e-10f) {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / norm;
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

/* ZYX Euler extraction — exact copy of imu_filter._quat_to_euler(). */
static inline void mahony_to_euler(const float q[4],
                                   float *roll, float *pitch, float *yaw) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    // roll (X)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = MAHONY_DEGREES(atan2f(sinr_cosp, cosr_cosp));
    // pitch (Y) — clamped to [-90, 90]
    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = MAHONY_DEGREES(asinf(sinp));
    // yaw (Z)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = MAHONY_DEGREES(atan2f(siny_cosp, cosy_cosp));
}

/* One-step Mahony update.
 * Inputs:  body-frame gyro [rad/s], accel [m/s²], timestep dt [s].
 * Outputs: ZYX Euler (roll, pitch, yaw) in degrees.
 * Exact port of imu_filter.MahonyFilter.update(). */
static inline void mahony_update(MahonyState *s,
                                 float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt,
                                 float kp, float ki,
                                 float *roll, float *pitch, float *yaw) {
    // Normalize accel (gravity reference)
    float a_norm = sqrtf(ax*ax + ay*ay + az*az);
    float ax_n, ay_n, az_n;
    if (a_norm < 1e-10f) {
        ax_n = 0.0f; ay_n = 0.0f; az_n = 1.0f;
    } else {
        float inv_a = 1.0f / a_norm;
        ax_n = ax * inv_a;
        ay_n = ay * inv_a;
        az_n = az * inv_a;
    }

    // Estimated gravity direction from current quaternion
    float q0 = s->q[0], q1 = s->q[1], q2 = s->q[2], q3 = s->q[3];
    float v_x = 2.0f * (q1 * q3 - q0 * q2);
    float v_y = 2.0f * (q0 * q1 + q2 * q3);
    float v_z = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    // Error = accel × gravity (cross product)
    float ex = ay_n * v_z - az_n * v_y;
    float ey = az_n * v_x - ax_n * v_z;
    float ez = ax_n * v_y - ay_n * v_x;

    // Integral feedback (only when accel is valid)
    if (a_norm >= 1e-10f) {
        s->integral_fb[0] += ki * ex * dt;
        s->integral_fb[1] += ki * ey * dt;
        s->integral_fb[2] += ki * ez * dt;
    }

    // Corrected gyro
    float gx_corr = gx + kp * ex + s->integral_fb[0];
    float gy_corr = gy + kp * ey + s->integral_fb[1];
    float gz_corr = gz + kp * ez + s->integral_fb[2];

    // Quaternion derivative: dq/dt = 0.5 * q ⊗ [0, gx_corr, gy_corr, gz_corr]
    float qDot[4];
    qDot[0] = 0.5f * (-q1 * gx_corr - q2 * gy_corr - q3 * gz_corr);
    qDot[1] = 0.5f * ( q0 * gx_corr + q2 * gz_corr - q3 * gy_corr);
    qDot[2] = 0.5f * ( q0 * gy_corr - q1 * gz_corr + q3 * gx_corr);
    qDot[3] = 0.5f * ( q0 * gz_corr + q1 * gy_corr - q2 * gx_corr);

    // First-order integration
    s->q[0] += qDot[0] * dt;
    s->q[1] += qDot[1] * dt;
    s->q[2] += qDot[2] * dt;
    s->q[3] += qDot[3] * dt;

    // Normalize to prevent drift
    mahony_normalize(s->q);

    // Extract Euler
    mahony_to_euler(s->q, roll, pitch, yaw);
}

#endif /* MAHONY_H */
