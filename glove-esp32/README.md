# Gesture Glove — ESP32 Transmitter

Firmware for the MPU6050 gesture glove. Reads IMU at 100 Hz, runs a Mahony
AHRS filter, and transmits Euler angles at 20 Hz as one-line JSON over both
WiFi UDP and Serial.

The PC-side receiver is `pc/glove/glove_bridge.py`.

## Wiring — GY-521 ↔ ESP32-DevKitC

| GY-521 Pin | ESP32 Pin | Notes                     |
|------------|-----------|---------------------------|
| VCC        | 3V3       | 3.3 V supply              |
| GND        | GND       | Common ground             |
| SDA        | GPIO 21   | I2C data (GY-521 has onboard pull-ups — no external needed) |
| SCL        | GPIO 22   | I2C clock                 |
| INT        | —         | Not connected (polled)    |

## Build & Flash

```bash
cd glove-esp32
pio run               # compile
pio run -t upload     # flash
pio device monitor    # serial monitor (115200 baud)
```

## WiFi Credentials

By default the firmware connects to `GLOVE_AP` / `12345678` (placeholder).

To use your own network, create `src/config_secrets.h` (git-ignored):

```c
#pragma once
#define WIFI_SSID "MyHomeNetwork"
#define WIFI_PASS "MySecretPassword"
```

Or pass them via build flags:

```bash
pio run -e esp32dev --build-flags '-DWIFI_SSID=\"MySSID\" -DWIFI_PASS=\"MyPW\"'
```

## Running the PC Bridge

### UDP (wireless)

```bash
cd pc
python glove\glove_bridge.py --glove-source udp:8766
```

The firmware broadcasts JSON to `255.255.255.255:8766`. The bridge binds
on `0.0.0.0:8766` to receive it.

### Serial (wired USB fallback)

Connect ESP32 via USB, find the COM port, then:

```bash
cd pc
python glove\glove_bridge.py --glove-source serial:COM8
```

The firmware always emits on Serial regardless of WiFi status.

## Wire Format

One JSON object per line, UTF-8, `\n` terminated:

```
{"p":12.34,"r":-5.67,"y":90.12}
```

| Key | Field      | Unit  |
|-----|------------|-------|
| `p` | pitch (Y)  | deg   |
| `r` | roll (X)   | deg   |
| `y` | yaw (Z)    | deg   |

## Tuning

All constants are in `src/config.h`. Key values mirror `pc/config.py`:

| Define            | Default | Description                 |
|-------------------|---------|-----------------------------|
| `GLOVE_IMU_HZ`    | 100     | IMU sample rate             |
| `GLOVE_SEND_HZ`   | 20      | JSON emission rate           |
| `GLOVE_MAHONY_KP` | 0.5     | Mahony proportional gain     |
| `GLOVE_MAHONY_KI` | 0.0     | Mahony integral gain          |
| `BAUDRATE`        | 115200  | Serial baud rate             |
