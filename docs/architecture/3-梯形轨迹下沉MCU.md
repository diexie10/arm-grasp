# 3-梯形轨迹下沉MCU.md — 固件自主运动执行（G/DONE 协议）

> **版本**: v1.0（三轮对抗审查后定稿） | **日期**: 2026-08-24 | **状态**: 待施工
> **前置**: `2-架构升级计划-MOVE下沉.md`（本文是其改动①的详细设计，取代其中 MOVE/DONE 草案）
> **需求来源**: 用户 2026-08-24——"转动不要太快，机械结构质量一般，起停要加阻尼更顺滑，上电压力小一点"
> **审查记录**: 三轮对抗（用户引入外部 AI 两轮 + 本地逐条验算一轮），全部分歧已收敛

---

## 1. 需求与方法论依据

| 来源 | 结论 |
|------|------|
| robot-arm-algorithms ch03 §3.2 | 梯形轨迹（加-匀-减）；最大速度 = 硬件极限 50%（MG996 带载 60°/s → **30°/s**）；加速度 = 速度的 50%（**15°/s²**） |
| ch03 §3.4 | 到位判定：连续 3 周期误差 < 阈值；关键动作前稳定等待 |
| ch03 §3.3 + arm-grasp skill §3 | 软启动错峰：6×MG996 同时启动瞬态 15A > 电源 8A，必须错开 |
| PAROL6 / AR4 路线 | 运动执行器在 MCU 侧：上位机发目标点，下位机自主规划执行 |

## 2. 协议设计

```
G <j1> <j2> <j3> <j4> <j5> <j6>   批量目标（舵机角整数°），触发六轴梯形运动，
                                   各轴错峰 i×3 tick（60ms）启动
Q                                  查询 → "BUSY" / "DONE"
                                   DONE = 全轴连续 3 tick 误差<0.01°（不含稳定等待）
M <n> <angle>                      单轴立即覆盖（四件套，见 §4）——保留给夹爪/微调
H                                  清急停 + 全轴复位 HOME 态 + 不使能 PWM；
                                   PC 收到 OK H 后跑现有 soft_start；永不自动续走旧目标
E                                  急停：冻结梯形 + PWM 全停（现有行为）
```

**回显铁律适配**：G 回显六个 clamp 后的目标值；Q 不刷新超时以外的状态。

## 3. 梯形轨迹器（每轴独立）

### 常数表（全部带乘回验算——本项目两次换算事故后的铁律）

```c
#define TICK_MS          20U     /* 50Hz，与舵机刷新对齐 */
#define MAX_VEL       0.6f       /* ×50  = 30°/s  ✓（极限 60°/s 的 50%）*/
#define ACC_PER_TICK  0.006f     /* ×2500 = 15°/s² ✓（速度的 50%）*/
#define REACH_TOL      0.01f     /* 到位阈值 ° */
#define SETTLE_TICKS      3U     /* 连续 3 tick < REACH_TOL 判到位 */
#define STAGGER_TICKS     3U     /* 六轴启动错开 i×3 tick = 60ms/轴 */
#define MOTION_HARD_CAP_MS 15000U /* 运动硬上限：超时强制置 DONE，防超时保护被永久压制 */
/* 参考行程：3° 三角轮廓 ≈0.9s；30° ≈2.8s；90° 梯形 ≈5s；180° 满程 ≈7.7s */
```

### 数据结构

```c
typedef struct {
    uint16_t target;      /* 目标角（舵机域 0-180）*/
    float    current;     /* 当前角（浮点，软件浮点 @50Hz×6轴 无性能问题）*/
    float    vel;         /* °/tick 带符号 —— 必须 float：int 取整会使梯形退化为阶跃 */
    uint8_t  settled;     /* 连续到位计数（上限 255 封顶）*/
} ServoAxis;
```

### 每 tick 算法

```
dist = |target - current|
if dist < REACH_TOL: vel=0; settled++(≤255); return
decel_dist = vel² / (2·ACC_PER_TICK)        /* 每 tick 重算，禁止固定减速窗 */
if dist > decel_dist: vel 向 MAX_VEL 加速（+ACC_PER_TICK）
else:                 vel 向 ACC_PER_TICK 减速（保底最小步进）
if vel > dist: vel = dist                    /* 防过冲 */
current += sign(target-current) · vel
写 compare(current)；settled = 0
```

**收敛性**：float vel 下 decel_dist 自适应，数学上保证收敛（int8_t + 固定 3° 窗方案已被证伪废弃）。

## 4. M 命令语义定型：单轴立即覆盖四件套

```c
axis.target[n]   = angle;   /* ① 覆盖目标——否则下一 tick 掉头走向 G 的旧目标 */
axis.current[n]  = angle;   /* ② 瞬时同步位置——保留 M 的"立即"语义 */
axis.vel[n]      = 0.0f;    /* ③ 复位速度——防止残余速度尖峰 */
axis.settled[n]  = 0;       /* ④ 重置稳定计数 */
```

只影响该轴，其余五轴梯形不中断。M 不触发/不复位全局 DONE。

## 5. 超时与安全策略

| 场景 | 行为 |
|------|------|
| 正常运动中 | PC 每 50ms 轮询 Q → 刷新 `last_cmd_tick` → 10s 超时不触发 |
| PC 死于运动中途 | 轮询停止 → `motion_done==0` 抑制超时 → 臂走完轨迹再停（优于半途瘫痪坠落掉木块）；`MOTION_HARD_CAP_MS` 兜底防永久压制 |
| PC 死于静止 | 10s 超时正常触发 → PWM 停（现有行为） |
| E 之后 | 开环舵机位置信息永久丢失，**任何情况下不自动续走旧 target**；H 只复位状态不使能 PWM，PC soft_start 逐轴接管 |
| PC 侧 DONE 超时 | planned_duration×2 + 2s；超时 → 发 E（best effort）→ 状态机 ERROR → 提示断电。**不重试**（IWDG ~2s 是 MCU 侧最后底线） |

## 6. 上位机改造

```
P1  Trajectory 改 waypoint 模式：删除 20ms 插补循环；plan_joint_move 仅保留用于
    DONE 超时估算。90° 运动 = 1 条 G（原 ~250 条 M）
P2  _exec_waypoint：joint→servo(+OFFSET) 取整 → 发 G → 50ms 轮询 Q
P3  500ms 稳定等待只在关键动作前（GRIP 前、DESCEND IR 采样前），共 2~3 处。
    不放固件 DONE 里——静止时 PC 卡顿零物理后果（PWM 保持扭矩），
    放 DONE 会造成 waypoint 数 × 500ms 的实用性炸弹
P4  FINAL_ALIGN_MAX_ITER 10 → 5（像素伺服预期 3-4 步收敛，联调确认）
P5  soft_start 不变（PC 侧逐轴 200ms；懒启动留固件，两者是不同层）
```

## 7. 明确不做（防过度工程）

- 不做 S 曲线（hobby 臂梯形够用）
- 不做 MCU 全量轨迹规划（ADR-2 第 2 步以后）
- 不做对齐快速档（多一套参数表 = 多一个出错源）
- REACH_TOL 保持 0.01° 不按场景切换（放宽仅省 ~100ms/步）

## 8. 审查史（教训存档）

| 轮次 | 被抓错误 | 教训 |
|------|---------|------|
| 1 | vel=int8_t 取整 → 梯形不存在；固定 3° 减速窗不收敛；500ms 放 PC 与初衷矛盾；逐关节 M 与批量 DONE 粒度冲突 | 小数速度是梯形前提；减速窗必须 v²/2a 自适应 |
| 2 | °/s→°/tick 换算忘除 50（MAX_VEL 实为 75°/s 超 60°/s 极限）；M 覆盖不复位 vel；500ms×N 炸弹；stagger 非 tick 整数倍 | **常数注释必须带乘回验算式**（本项目同类错两犯） |
| 3 | （本地验算）M 四件套缺①覆盖目标会掉头；"10s 炸弹"系假警报（忘了 DONE 轮询刷新超时）但引出 PC 死于途中的策略真空；E 后续走旧目标是开环陷阱 | 对抗审查的结论也要验算——假警报和真 bug 都存在 |

## 9. 施工清单与验收

```
F1  ServoAxis + 常数表（§3）
F2  G 命令解析 + 错峰启动 + 回显 clamp 目标
F3  RampStep 梯形算法（替换现恒速版）
F4  Q 查询命令（BUSY/DONE）
F5  超时门控（motion_done==0 抑制 + 15s 硬上限）
F6  M 四件套改造
F7  H 复位语义（不清 PWM 使能，交 PC soft_start）
P1-P5  上位机（§6）
验收：
 [ ] 六轴同时 G 启动，6V 母线跌落 < 0.5V（万用表监测）
 [ ] Windows 人为卡顿 3s（运动中）：臂走完轨迹不瘫
 [ ] 单次 TRANSPORT 串口命令 ≤ 10 条
 [ ] 180° 满程运动平滑无起停冲击（目测 + 听音）
 [ ] E → H → soft_start 恢复流程无 slam
 [ ] DONE 超时路径触发一次演练（拔串口模拟）
```
