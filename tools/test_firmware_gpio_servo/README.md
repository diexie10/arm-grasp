# 极简舵机测试固件（bit-bang GPIO 版）

日期：2026-08-23
用途：绕开所有定时器/HAL/应用层代码，用最笨的方式验证"舵机 + 供电 + 信号路径"是否健康。
结论判定：**如果这个固件都不能让舵机动，问题一定不在固件配置层。**

## 它做什么

- PA8 和 PA0 同时输出 50Hz 舵机脉冲（纯 GPIO 寄存器拉高拉低，无定时器）
- 脉宽每 ~2 秒在 **2000µs ↔ 1000µs** 之间切换 → 好舵机会大幅来回扫动
- 板载 LED（PC13）每 2 秒翻转一次 = 心跳

## 使用步骤

### 1. 替换 main.c

```
copy Core\Src\main.c Core\Src\main.c.current   （可选：备份当前版本）
copy tools\test_firmware_gpio_servo\main.c Core\Src\main.c
```

### 2. 编译

Keil GUI：打开 `MDK-ARM\arm-grasp.uvprojx`，F7。
或命令行走 keil-mcp_build_project。应 0 Error。

### 3. 烧录（重要：用 pyocd，别用 Keil 的下载按钮）

本项目的克隆 ST-Link 与 Keil 的 ST-LINKIII-KEIL_SWO.dll 不兼容
（报 "Internal DLL Error"），但 pyocd 实测可用：

```powershell
pip install pyocd                        # 首次
pyocd pack install stm32f103c8           # 首次，下载 F1 烧录算法包

pyocd flash "MDK-ARM\arm-grasp\arm-grasp.axf" -t stm32f103c8 --frequency 100000
pyocd commander -t stm32f103c8 --frequency 100000 -c "reset" -c "go"
```

SWD 接线：ST-Link SWDIO→DIO、SWCLK→CLK、GND↔GND（3.3V 不接，板子 USB 供电）。
SWD 时钟必须降到 100kHz（`--frequency 100000`），默认速度克隆头连不稳。

### 4. 接舵机

```
舵机红线  → 6V 电源（经 PTC/滤波模块）
舵机棕线  → 6V 地 + 板子 GND（共地必须）
舵机黄线  → PA0 或 PA8（两脚都在发脉冲，插哪个都行）
```

### 5. 判读

| 现象 | 结论 |
|------|------|
| 舵机每 2 秒大幅换位扫动 | 舵机+供电+信号路径全部健康 |
| 完全不动，但手扭后会用力弹回、碰一下会抽动 | 舵机在"失联保持"模式：没收到有效信号（查信号线最后一厘米）|
| 万用表 DC 200mV 档量黄↔棕：~330mV ↔ ~165mV 每 2 秒交替 | 脉冲已送达舵机插口（路径无罪）|
| 万用表读数死平 | 断点在 PA0/PA8 → 100Ω → 黄线之间 |

### 6. 恢复生产固件

```
git checkout HEAD -- Core\Src\main.c
```

重新编译 + pyocd 烧录（同第 3 步）。

## 本固件的诊断战绩（2026-08-23）

- 证明 PA8/PA0 引脚可输出 3.3V（前一版恒高电平测试）
- 证明信号路径 PA→100Ω→舵机黄线全程导通（实测 100Ω 整）
- 证明脉冲以正确节奏到达舵机插口（330/165mV 两档交替实测）
- 排除：时钟假设错误（pyod 读 RCC->CFGR = HSI 8MHz 确认）
- **根因定位**：bit-bang GPIO 能驱动 ≠ TIM PWM 能驱动。HAL 的 `HAL_TIM_PWM_Init()` 调用 `HAL_TIM_PWM_MspInit()`（__weak 空默认），项目只定义了 `HAL_TIM_Base_MspInit()`（CubeMX 生成），函数名不匹配 → TIM 时钟 never enabled → 寄存器全零 → PWM 不输出。修复见 KNOWN_TRAPS #24
