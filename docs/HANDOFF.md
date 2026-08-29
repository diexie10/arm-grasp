# 机械臂项目交接指南

> 最后更新：2026-08-27

## 一、项目概述

基于 STM32F103C8T6 + CH340 的 6 轴机械臂，通过 USB CDC 串口接收 PC 指令控制 MG996R 舵机。PC 端使用 Python + WebSerial 控制台。

## 二、固件状态（arm-grasp）

### 已完成

| 功能 | 状态 | 说明 |
|---|---|---|
| ADR-3 运动控制 | ✅ | G 批量指令、Q 查询、M 单轴覆盖、H 回零 |
| 梯形速度曲线 | ✅ | MAX_VEL=0.6°/tick, ACC=0.006°/tick² |
| 阶梯渐进（S1/S3）| ✅ | 自适应步长 15°/10°/5°，停留 0.66s |
| 无符号溢出修复 | ✅ | `(uint32_t)(-7)` → 有符号 int32 数学 |
| 逐轴最大速度 | ✅ | S1/S3=7.5°/s, 其他=30°/s |

### 舵机校准数据

| 舵机 | Trim | 控制范围 | 物理范围 | 启停位 |
|---|---|---|---|---|
| S1 J1 | -11 | 11~191° | 7~197° | 101° |
| S2 J2 | -3 | 3~183° | 0~191° | 122° |
| S3 J3 | -5 | 5~167° | 0~191° | 167° |
| S4 J4 | -7 | 7~187° | 0~189° | 97° |
| S5 J5 | ⏳ | 待校准 | 待校准 | 90° |
| S6 J6 | ⏳ | 待校准 | 待校准 | 90° |

### 关键固件文件

```
arm-grasp/Core/Src/usbd_cdc_interface.c   ← 舵机控制核心（joint_min/max, trim, RampStep）
arm-grasp/pc/config.py                     ← PC 端配置（JOINT_MIN/MAX/HOME）
arm-grasp/tools/servo_6axis.html           ← 6 轴控制台（G 指令）
arm-grasp/tools/servo_simple.html          ← 单轴调试台（M 指令）
arm-grasp/tools/servo_calibration.py       ← 交互式校准工具
```

## 三、载板 PCB 状态（arm-carrier）

### 已完成

| 步骤 | 状态 | 说明 |
|---|---|---|
| 原理图设计 | ✅ | 23+ 元件，ERC 0 错误 |
| 网表审核 | ✅ | 46 个网络，电源链完整 |
| 四层板设置 | ✅ | Top / GND / +6V / Bottom |
| DSN 导出 | ✅ | 修复 BOM + pcb 大小写 |
| FreRouting 布线 | ✅ | `-inc GND,+6V` 跳过电源网络 |
| SES 导入 | ✅ | 108 根信号走线 |

### 当前状态

| 项目 | 数量 | 说明 |
|---|---|---|
| 信号走线 | 108 根 | S1~S6 + GPIO 连线 |
| GND 过孔 | 0 | 需要通过内层平面连接 |
| +6V 过孔 | 0 | 需要通过内层平面连接 |
| 内层铺铜 | 0 | ⚠️ 待解决 |

### 未解决问题

#### 1. 内层铺铜 DRC 错误

**问题**：Inner1 (GND) 和 Inner2 (+6V) 没有铺铜对象，DRC 报连接性错误。

**根本原因**：嘉立创EDA Pro 的铺铜工具**只能在 Top/Bottom 层画**，内层 Plane 类型不允许手动铺铜。

**已尝试的解法**：
- API `pcb_PrimitivePour.create()` 只支持 Top/Bottom 层
- 尝试创建后改层 → 改不了
- 把内层从 PLANE 改成 SIGNAL → 可以改，但用户反馈还是不能选内层

**建议解法**：
- 方案 A：忽略 DRC 错误（Gerber 导出时内层会自动填充铜面）
- 方案 B：把内层改成 SIGNAL 类型后，手动在 GUI 画铺铜（文档说可以改层）

#### 2. FreRouting `-inc` 参数

**问题**：FreRouting 的 `-inc GND,+6V` 参数**确实生效**（SES 文件中 GND/+6V 无走线），但导入 SES 后 GND/+6V 的信号层走线又出现了。

**可能原因**：用户重新导入了 SES 文件覆盖了之前的修改。

**正确工作流**：
1. 导出 DSN
2. 修复 BOM（去掉 EF BB BF）+ 修复 `(pcb` 大小写
3. 给 Inner1/Inner2 加 `(conduction)` 定义
4. FreRouting 用 `-inc GND,+6V -do output.ses`
5. 导入 SES 到嘉立创EDA
6. **不要再重新导入 SES**

### 关键文件

```
arm-carrier.dsn                              ← FreRouting 输入（需修复 BOM + pcb）
arm-carrier-routed.ses                       ← FreRouting 输出
Autorouter_PCB1_2026-8-27.dsn               ← 嘉立创EDA 导出的 DSN
```

## 四、FreRouting 工作流

### 完整流程

```powershell
# 1. 修复 DSN（BOM + 大小写）
$bytes = [IO.File]::ReadAllBytes("board.dsn")
if ($bytes[0] -eq 0xEF) { $bytes = $bytes[3..($bytes.Length-1)] }  # 去 BOM
$content = [System.Text.Encoding]::UTF8.GetString($bytes)
$content = $content -replace '^\(PCB', '(pcb'  # 修复大小写
# 2. 给 Inner1/Inner2 加 conduction 定义（用正则替换）
# 3. 跑 FreRouting
& "C:\Program Files\Java\jdk-26.0.2.1\bin\java.exe" `
  -jar "C:\Users\diexie\Downloads\freerouting-2.3.0.jar" `
  -de "board.dsn" `
  -do "routed.ses" `
  -inc GND,+6V `
  -mt 1
```

### 关键注意事项

| 项目 | 说明 |
|---|---|
| DSN BOM | 嘉立创EDA 导出的 DSN 有 UTF-8 BOM，FreRouting 不认 |
| DSN `(pcb` | FreRouting 要求小写 `(pcb`，嘉立创EDA 导出大写 `(PCB)` |
| 内层 conduction | DSN 中 Inner1/Inner2 需要 `(conduction (path ...))` 定义 |
| `-inc` 参数 | 忽略指定网络类的布线，GND/+6V 应跳过 |
| `-mt 1` | 单线程优化，避免多线程导致的间距违规 |

## 五、下一步

### 优先级排序

| 优先级 | 任务 | 说明 |
|---|---|---|
| P0 | 解决内层铺铜 | 选择方案 A（忽略 DRC）或方案 B（改 SIGNAL 类型后画铺铜）|
| P0 | S5/S6 校准 | 接上舵机，用 servo_simple.html 交互式校准 |
| P1 | 导出 Gerber | DRC 通过后（或确认假阳性），导出 Gerber 下单 |
| P1 | JLCPCB 下单 | 4 层板，≤100×100mm，5 片免费 |
| P2 | 焊接测试 | 先焊电源部分，验证 6V/3.3V |
| P2 | 固件烧录 | 通过 ST-Link 烧录固件 |
| P3 | 舵机联调 | 6 轴全部接上，运行 G 指令测试 |

### S5/S6 校准步骤

1. 接上 S5 舵机到 H5
2. 打开 `servo_simple.html`，切换到 **M5**
3. 运行 `servo_calibration.py` 交互式校准
4. 记录 trim、控制范围、物理范围
5. 更新固件 `joint_min[4]`/`joint_max[4]` 和 PC config
6. 重复 S6

## 六、环境依赖

| 工具 | 版本 | 路径 |
|---|---|---|
| Java | 26.0.2.1 | `C:\Program Files\Java\jdk-26.0.2.1\` |
| FreRouting | 2.3.0 | `C:\Users\diexie\Downloads\freerouting-2.3.0.jar` |
| 嘉立创EDA Pro | 3.2.186 | 已安装 |
| Bridge Server | - | `C:\Users\diexie\.config\opencode\skills\easyeda-api\` |
| OpenCode | - | 已安装 |

## 七、已知陷阱

1. **不要在嘉立创EDA 里重新导入 SES**——会覆盖之前的修改
2. **FreRouting GUI 导出 SES**：用 `-do` 命令行参数更可靠
3. **内层 Plane 类型**：嘉立创EDA Pro 不允许在 Plane 层画铺铜
4. **铺铜工具只能选 Top/Bottom**：需要改内层为 SIGNAL 类型才能画
5. **DRC 连接性错误**：可能是假阳性，内层平面在 Gerber 导出时自动填充
6. **Bridge Server**：需要手动启动 `node scripts/bridge-server.mjs`
7. **DSN 文件编码**：必须用 UTF-8 无 BOM 编码保存
