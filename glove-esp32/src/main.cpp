/* main.cpp — Gesture-glove ESP32 transmitter.
 *
 * Reads MPU6050 at IMU_HZ via I2C, runs Mahony AHRS filter, emits
 * one-line JSON {"p":pitch,"r":roll,"y":yaw} at SEND_HZ over both
 * WiFi UDP broadcast and Serial.
 *
 * Wire format matches pc/glove/glove_bridge.py UDPSource / SerialSource.
 * Mahony filter is in mahony.h (testable on host without Arduino).
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "mahony.h"

// =====================================================================
//  MPU6050 driver
// =====================================================================

static void mpu6050_init() {
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);

    // Wake up (clear sleep bit)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();

    // Sample rate divider: 100 Hz = 1 kHz / (1 + 9)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_SMPLRT_DIV);
    Wire.write(9);
    Wire.endTransmission();

    // DLPF config ~44 Hz bandwidth
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_CONFIG);
    Wire.write(0x03);
    Wire.endTransmission();

    // Gyro ±500 °/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_GYRO_CFG);
    Wire.write((GYRO_FS_SEL & 0x03) << 3);
    Wire.endTransmission();

    // Accel ±2 g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_ACCEL_CFG);
    Wire.write((ACCEL_FS_SEL & 0x03) << 3);
    Wire.endTransmission();
}

/* Burst-read 14 bytes from 0x3B → ax,ay,az,temp,gx,gy,gz (int16 BE). */
static void mpu6050_read(int16_t raw[7]) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_DATA_START);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) {
        buf[i] = Wire.read();
    }
    // Parse big-endian int16
    for (int i = 0; i < 7; i++) {
        raw[i] = (int16_t)((buf[i*2] << 8) | buf[i*2 + 1]);
    }
}

// =====================================================================
//  Global state
// =====================================================================

static MahonyState mahony;
static WiFiUDP udp;
static bool udp_started = false;

// Timing
static unsigned long last_imu_ms = 0;
static int sample_counter = 0;

// WiFi reconnect tracking
static unsigned long last_wifi_attempt_ms = 0;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;

// =====================================================================
//  WiFi — non-blocking with reconnect
// =====================================================================

static void wifi_init() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    last_wifi_attempt_ms = millis();
}

static void wifi_connect_nonblocking() {
    if (WiFi.status() == WL_CONNECTED) return;

    unsigned long now = millis();
    if (now - last_wifi_attempt_ms < WIFI_RETRY_INTERVAL_MS) return;
    last_wifi_attempt_ms = now;

    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    udp_started = false; // re-bind after reconnect
}

static bool wifi_ready() {
    return WiFi.status() == WL_CONNECTED;
}

static void udp_ensure_started() {
    if (udp_started) return;
    if (!wifi_ready()) return;
    udp.begin(GLOVE_BRIDGE_UDP_PORT);
    udp_started = true;
}

// =====================================================================
//  Output — JSON line on UDP + Serial
// =====================================================================

static void emit_json(float pitch, float roll, float yaw) {
    // Format: {"p":pitch,"r":roll,"y":yaw}\n
    char buf[UDP_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf),
                       "{\"p\":%.2f,\"r\":%.2f,\"y\":%.2f}\n",
                       (double)pitch, (double)roll, (double)yaw);

    // Always emit on Serial (wired fallback)
    Serial.print(buf);

    // Emit on UDP broadcast if WiFi is up and UDP is bound
    if (udp_started) {
        udp.beginPacket(IPAddress(255, 255, 255, 255), GLOVE_BRIDGE_UDP_PORT);
        udp.write((const uint8_t *)buf, len);
        udp.endPacket();
    }
}

// =====================================================================
//  Arduino entry points
// =====================================================================

void setup() {
    Serial.begin(BAUDRATE);
    delay(200); // let serial settle

    mpu6050_init();
    mahony_init(&mahony);

    wifi_init();

    Serial.println("[glove] firmware ready");
    Serial.printf("[glove] IMU %d Hz, send %d Hz, Mahony Kp=%.2f Ki=%.2f\n",
                  GLOVE_IMU_HZ, GLOVE_SEND_HZ,
                  (double)GLOVE_MAHONY_KP, (double)GLOVE_MAHONY_KI);
}

void loop() {
    unsigned long now_ms = millis();

    // ── Non-blocking WiFi reconnect + UDP bind ──
    wifi_connect_nonblocking();
    udp_ensure_started();

    // ── IMU sampling at IMU_HZ ──
    unsigned long imu_interval_ms = 1000UL / GLOVE_IMU_HZ;
    if (now_ms - last_imu_ms < imu_interval_ms) return;
    last_imu_ms += imu_interval_ms;
    // Clamp to prevent spiral-of-death on long pauses
    if (now_ms - last_imu_ms > imu_interval_ms * 4) {
        last_imu_ms = now_ms;
    }

    // Read raw IMU
    int16_t raw[7];
    mpu6050_read(raw);

    // Convert to physical units
    float ax = (float)raw[0] / ACCEL_SENSITIVITY * G_TO_MS2;
    float ay = (float)raw[1] / ACCEL_SENSITIVITY * G_TO_MS2;
    float az = (float)raw[2] / ACCEL_SENSITIVITY * G_TO_MS2;
    float gx = (float)raw[4] / GYRO_SENSITIVITY * DEG_TO_RAD;
    float gy = (float)raw[5] / GYRO_SENSITIVITY * DEG_TO_RAD;
    float gz = (float)raw[6] / GYRO_SENSITIVITY * DEG_TO_RAD;

    // Mahony filter update
    float roll, pitch, yaw;
    mahony_update(&mahony, gx, gy, gz, ax, ay, az, IMU_DT_S,
                  GLOVE_MAHONY_KP, GLOVE_MAHONY_KI,
                  &roll, &pitch, &yaw);

    // ── Emit every SEND_EVERY_N samples to hit SEND_HZ ──
    sample_counter++;
    if (sample_counter >= SEND_EVERY_N) {
        sample_counter = 0;
        emit_json(pitch, roll, yaw);
    }
}
