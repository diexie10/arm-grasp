#pragma once
/* config.h — all tunables in one place, mirroring pc/config.py.
 *
 * Override WiFi credentials at build time via config_secrets.h:
 *   #define WIFI_SSID "MyNetwork"
 *   #define WIFI_PASS "MyPassword"
 */
#include <stdint.h>

// ── Optional secret overrides (git-ignored) ──────────────────────────
#if __has_include("config_secrets.h")
#include "config_secrets.h"
#endif

// ── WiFi ─────────────────────────────────────────────────────────────
#ifndef WIFI_SSID
#define WIFI_SSID "GLOVE_AP"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "12345678"
#endif

// ── Serial / Baudrate (mirrors config.py BAUDRATE) ───────────────────
#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

// ── UDP target (mirrors config.py GLOVE_BRIDGE_UDP_PORT) ─────────────
#ifndef GLOVE_BRIDGE_UDP_PORT
#define GLOVE_BRIDGE_UDP_PORT 8766
#endif
#define UDP_BUF_SIZE 256

// ── I2C ──────────────────────────────────────────────────────────────
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ_HZ 400000

// ── MPU6050 ──────────────────────────────────────────────────────────
#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG     0x1A
#define MPU6050_REG_GYRO_CFG   0x1B
#define MPU6050_REG_ACCEL_CFG  0x1C
#define MPU6050_REG_DATA_START 0x3B

// Full-scale ranges
#define GYRO_FS_SEL  0x01   // ±500 °/s
#define ACCEL_FS_SEL 0x00   // ±2 g

// Sensitivities (LSB per unit)
#define GYRO_SENSITIVITY  65.5f        // LSB/(°/s)  at ±500 °/s
#define ACCEL_SENSITIVITY 16384.0f     // LSB per g at ±2 g (multiply by G_TO_MS2 to get m/s²)

// Gravity constant for accel conversion
#define G_TO_MS2 9.80665f

// ── IMU timing (mirrors config.py GLOVE_IMU_HZ) ─────────────────────
#ifndef GLOVE_IMU_HZ
#define GLOVE_IMU_HZ 100
#endif
#define IMU_DT_S (1.0f / (float)GLOVE_IMU_HZ)

// ── Send rate (mirrors config.py GLOVE_SEND_HZ) ─────────────────────
#ifndef GLOVE_SEND_HZ
#define GLOVE_SEND_HZ 20
#endif
// Every Nth IMU sample triggers a JSON emit
#define SEND_EVERY_N ((GLOVE_IMU_HZ) / (GLOVE_SEND_HZ))

// ── Mahony filter (mirrors config.py GLOVE_MAHONY_KP / KI) ──────────
#ifndef GLOVE_MAHONY_KP
#define GLOVE_MAHONY_KP 0.5f
#endif
#ifndef GLOVE_MAHONY_KI
#define GLOVE_MAHONY_KI 0.0f
#endif

// ── Gyro-bias calibration ─────────────────────────────────────────
// Number of IMU samples to average for bias estimation at startup.
// 200 samples × 10 ms = 2.0 s at 100 Hz.
#ifndef GLOVE_CAL_SAMPLES
#define GLOVE_CAL_SAMPLES 200
#endif

// Output-suppression window after a filter reset (ms).  Gives the
// Mahony filter time to re-converge before JSON streaming resumes.
#ifndef GLOVE_SETTLE_MS
#define GLOVE_SETTLE_MS 800
#endif

// Max length of a serial command line (including '\0').
#ifndef GLOVE_CMD_LINE_MAX
#define GLOVE_CMD_LINE_MAX 32
#endif
