# CLAUDE.md — arm-grasp 项目规则

> 本文件是项目铁律 + 阅读顺序。任何 AI/人动这个工程前先读这里。
> 文档地图：`docs/docs/README.md`（所有文档按 决策/状态/知识 三类归类）。
> ⚠️ 注意路径：文档实体在 **`docs/docs/`**（工程根 `docs/` 内嵌一层 `docs/`），文档内引用均以该目录为基准。

## 阅读顺序

1. `docs/docs/architecture/1-架构设计.md` — 架构决策书（先看这个）【决策】
2. 本文件（规则）【入口】
3. `docs/docs/project/UNIMPLEMENTED.md` — 半截功能清单（防"做完伪装"）【状态】
4. `docs/docs/project/SESSION_LOG.md` — 会话日志（最近一次改动）【状态】
5. `docs/docs/knowledge/KNOWN_TRAPS.md` — 已知陷阱（踩过的坑）【知识】
6. `docs/docs/architecture/5-柱坐标运动学与双视图模型.md` · `6-工程规范与最佳实践.md` — 坐标模型 · 工程规范【决策】
7. `docs/docs/project/要干的事-装机与联调待办.md` — 上机行动清单（Phase A–E + S1 专项 + 测试/解耦现状）【状态】

## 项目铁律

### 硬件/引脚（勿改）
- 舵机1-6 = PA8/PA9/PA10 (TIM1_CH1-3) + PA0/PA1/PA2 (TIM2_CH1-3)
- 红外 = PB12/PB13（输入上拉，遮挡=0）
- **PA11/PA12 是 USB 专用，绝不可做 PWM/GPIO 输出**
- SWD = PA13/PA14（ST-Link 烧录）

### 供电（炸机高发区）
- 6V 8A 独立电源给舵机，USB 只给 STM32
- **必须共地**（6V GND 与 USB GND 相连）
- 舵机电源严禁接 STM32 3.3V/5V 引脚
- 信号线串联 100Ω；与电源线间距 >2cm

### 固件（编译纪律）
- 任何改动后必须编译验证：**0 Error 才算完成**
- 编译: `keil-mcp_build_project`（uvprojx: `MDK-ARM\arm-grasp.uvprojx`）
- MDK-Lite 32KB 代码上限（当前 ~17KB）
- 编码 UTF-8 + LF，全英文注释（无编码风险）

### 上位机回显铁律（联调生死线）
- 固件对 `M<n>` 回显 **clamp 后的实际角度**（如请求 200 回显 170）
- `arm_serial.py` **必须解析回显值** 并更新 `kinematics` 内部关节状态，**严禁忽略回显**
- 请求角度与回显偏差 >5° → 报 `WARNING: Joint limit mismatch` 并暂停
- 违反此铁律 → 虚拟角度与物理角度脱节 → IK 漂移 → 臂撞限位

### 跨端常量铁律（P0 之后新增，2026-09-10）
- **限位真源 = 舵机域**（固件 `servo.c joint_min/max`）；**关节域由 offset 派生**（`JOINT_MIN/MAX = SERVO − JOINT_OFFSET`）
- **禁止手工镜像跨端常量**；连接时固件 `L` 握手校验（`arm_serial.verify_limits`），失配**硬停**
- 兜底：`pc/tests/test_firmware_contract.py` 断言 PC config ↔ 固件源码一致

### 代码规范
- 无裸数字：固件用宏定义，上位机用 config.py
- 所有错误码检查，无静默失败
- 中断回调不做浮点/长任务
- 禁止 time.sleep 长等待（上位机状态机用非阻塞计时器）

### 诚实纪律
- 半截功能 → 写进 `docs/docs/project/UNIMPLEMENTED.md`
- 推测值/未验证假设 → 写进 `docs/docs/project/诚实声明与技术债务清单.md`
- 发现 bug 修复后 → 抽象成陷阱写进 `docs/docs/knowledge/KNOWN_TRAPS.md`
- 每次会话结束 → 更新 `docs/docs/project/SESSION_LOG.md`

## Commit 前自检

- [ ] 编译 0 Error（固件）
- [ ] 无裸数字（全部走宏/config.py）
- [ ] 无 try/except:pass（上位机）
- [ ] 无 time.sleep 长等待
- [ ] `pytest pc/tests` 通过
- [ ] `docs/docs/project/UNIMPLEMENTED.md` 已检查
- [ ] `docs/docs/project/SESSION_LOG.md` 已更新
