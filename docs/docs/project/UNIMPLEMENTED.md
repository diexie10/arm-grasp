# UNIMPLEMENTED — 半截功能清单

> 作用：防"做完伪装"。看起来写完了其实没写完的，都在这。
> 维护：新增半截功能 → 加一行；完成 → 删一行（移入"已完成"）。

## P0 — 阻塞项（不做则后续无法进行）

| 功能 | 位置 | 状态 | 前置依赖 | 验收标准 |
|------|------|------|---------|---------|
| 上位机全部 9 模块（config/kinematics/serial/trajectory/calibration/vision/ir/state_machine/main） | pc/ | ✅ 通过 oracle 审查修复 | — | IK 189 点 0 bug；DRY_RUN 全流程 True；P1×3 已修 |

## P1 — 核心功能（抓取流程必需）

| 功能 | 位置 | 状态 | 前置依赖 | 验收标准 |
|------|------|------|---------|---------|
| IK/FK 实现 + 单测 | kinematics.py | ✅ 189 点 0 bug | 连杆参数实测 | FK(IK(T)) 误差 <1mm（实测后重测） |
| 视觉伺服方案（SEARCH/ALIGN/DESCEND） | state_machine.py | ✅ DRY_RUN True（待真机调参） | 机械臂+摄像头到货 | 伺服收敛像素 <ALIGN_PX_TOL；SERVO_KP 符号/大小实测 |
| 四点标定（旧路线，已降级备用） | calibration.py | ✅ 闭环 0.000mm（不再主用） | 摄像头固定 | 视觉伺服不依赖；仅旧路线兼容 |
| 笛卡尔直线插补（下降段） | trajectory.py | 未开始 | IK | 下降段末端轨迹为直线（关节空间插补是弧线，会撞物体） |
| 相机畸变去畸变 | vision.py | 未开始（视觉伺服不依赖） | 摄像头 | 边缘畸变 <2px（大畸变时 undistort） |
| 状态机框架（视觉伺服版） | state_machine.py | ✅ DRY_RUN 全流程 True | 机械臂+摄像头 | 真机 SEARCH 转圈不漏检 + ALIGN 收敛 |
| 消隙策略（单方向逼近） | trajectory.py | ✅ 实现 | 固件回显解析 | 同点 10 次偏差 <3°（实测） |
| 伺服 PID 调参 | state_machine.py + config.py | 骨架已实现 | 机械臂+摄像头 | Kp 符号正确、无震荡、收敛 <8px；**收敛死区 ≥2° 即判到位（<2° 修正被回差吞掉，调研 2026-08-18）；FINAL_ALIGN_MAX_STEP 可能需 2.5°** |
| J5 夹爪朝向对齐（J5=θ−J1） | state_machine.py FINAL_ALIGN | ✅ 已实现（2026-08-24）**未实测** | 机械臂+摄像头 | θ 符号/象限实测；J5 旋转方向与摄像头画面一致性确认；正方形木块可暂不启用验证 |
| move_to 末尾 S 查询对账（B3） | trajectory.py | ✅ 已实现（2026-08-24）**未实测** | 固件联调 | 实测每次 move_to 增加的延迟（一次串口往返）；若显著拖慢伺服循环则改为每 N 次 move_to 对账一次 |
| 架构升级：MOVE/DONE 运动执行下沉 MCU | docs/architecture/2-架构升级计划-MOVE下沉.md | 方案定稿未施工 | dev 分支 9 修复实机验证通过 | 见 ADR-2 验收标准（卡顿 500ms 臂不停 / TRANSPORT 串口命令 ≤10 条 / 浪涌错峰） |
| 梯形轨迹下沉 MCU（G/DONE 协议） | docs/architecture/3-梯形轨迹下沉MCU.md | **固件+上位机已施工（2026-08-25），单轴待实机验证** | 烧录后 J1 单轴验证 G/Q/M/H | 见 ADR-3 §9 验收清单（多轴错峰/母线跌落项等线材到货后补验） |
| 单目测距（已知尺寸 3.5cm） | 未开始 | 备选方案 | 摄像头装机械臂 | 不训练模型，公式法；仅需精确高度时启用 |
| 电流采样力检测（ADC 读舵机电流，夹住即停） | 未开始 | 调研结论（2026-08-18） | 硬件到货 | 夹住时电流上升检测到即停，防压坏木块 |

## P2 — 鲁棒性（连续运行必需）

| 功能 | 位置 | 状态 | 前置依赖 | 验收标准 |
|------|------|------|---------|---------|
| 硬件安全件（8A PTC/防反接/急停按钮） | 硬件 | 待采购 | 无 | 万用表验证 |
| 红外极性 + 输出类型实测 | 硬件接线 | 待到货 | 无 | `I` 命令遮挡测试 |
| 关节限位值实测校准 | 固件 joint_min/max 表 | 推测值待实测 | 机械臂到货 | 实测机械限位后改表 |
| IK 连杆参数实测标定 | config.py | 推测值待实测 | 机械臂到货 | 实测后改 config |
| 软启动（逐舵机使能 200ms） | arm_serial.soft_start | ✅ 实现 | 固件 | DRY_RUN 通过；真机观察 |
| 对齐环 wrist-first 分层（粗层死区 / J4-slack 微调 / 升级计数器 / 粗层最小步距） | servo_controller.py + state_machine.py + config.py | 2026-09-06 实现，**参数为预置默认待实测** | 虚位量化（阶段 6 第 7 项）+ J4-slack 视觉容忍度（阶段 8 第 5 项） | 末端无持续抖动（极限环消失）；腕层饱和/停滞正确升级粗层；粗层不再发 <3° 小步 |
| dwell 按峰速缩放（近端小 bite 短停） | 固件 servo.c | 未开始（待实测数据） | 每 bite 稳定时间实测（阶段 6 第 8 项） | 大摆时间缩短且末端无新增抖动 |
| J5 横向微调（横向 xy 小误差走腕层） | servo_controller.py | 未开始 | J5 偏轴量实测（阶段 8 第 6 项） | 若 WRIST_LATERAL_MM_PER_DEG>0，横向小误差由 J5 承担 |
| IK 目标语义修正（腕点 vs 爪尖） | kinematics.py | 未开始 | 装机日第 0 项实测（阶段 6 第 5 项） | 语义锁定或 L4 入模；低 z 结论重审 |
| 日志系统 | main.py + logging | ✅ 实现 | 状态机 | 每次状态转换有记录 |

---

## 手套模式（并行手动控制通道，2026-09-10 新增）

| 功能 | 位置 | 状态 | 前置依赖 | 验收标准 |
|------|------|------|---------|---------|
| PC 手套桥（UDP/串口接收 → 滤波 → 映射 → 20Hz M 流 → 断流冻结/急停门控） | pc/glove/glove_bridge.py | 进行中（方案 v0.2 已仿真验证，见 architecture/4 §8） | 手套方案 | DRY_RUN 12s 无错发完；回显对账通过；断流 500ms 冻结；E 可急停 |
| ESP32 手套固件（MPU6050 I2C + Mahony @100Hz + UDP/串口发送） | 待建（手套硬件） | 未开始 | ESP32+MPU6050 到货 | 与 PC 桥联调稳定流；欧拉角与真实手姿态偏差 <3° |
| 手套模式接入调试台 UI（手套态可视化/一键急停） | pc/ui/ | 未开始 | PC 手套桥 | 调试台实时显示手套映射目标与实际 |
| S6 触发方式（按钮 or 手势） | 手套硬件 | 未定案 | 硬件到货实测 | 误触率实测 |

---

## 已完成（历史）

| 功能 | 位置 | 完成时间 | 验证 |
|------|------|---------|------|
| 上位机 9 模块（IK/标定/轨迹/串口/状态机等） | pc/ | 2026-08-17 Phase C | IK 189 点 0 bug；DRY_RUN 全流程 True；待 oracle 审查 |
| 关节角↔舵机角 offset 映射 | config.py + arm_serial.py | 2026-08-17 Phase C | DRY_RUN 通过；offset 待实测 |
| 忙等定时补偿（Windows sleep 粒度坑） | trajectory.py | 2026-08-17 Phase C | move_to 8.58s→理论 5.16s；DRY_RUN 不再超时 |
| 关节限位钳位（每关节 min/max clamp） | 固件 usbd_cdc_interface.c | 2026-08-17 Phase A | 编译 0 Error；M 回显 clamp 后值 |
| 10s 无命令超时自动停止（PWM 停→松脱） | 固件 usbd_cdc_interface.c | 2026-08-17 Phase A+B；**2026-08-24 改 15s**（容纳相机预热+模型加载） | 编译 0 Error；停发超时观察 |
| `H` 命令渐进归中（2°/20ms 斜坡，防六轴同跳浪涌） | 固件 usbd_cdc_interface.c Servo_RampStep | 2026-08-24（commit 714e2e3） | 编译 0 Error；真机 H 观察平滑性待做 |
| 舵机通道映射统一 servo_map[6] | 固件 usbd_cdc_interface.c | 2026-08-24（commit 714e2e3） | 编译 0 Error |
| USB 断开 → PWM 停止 + 清残留命令 | 固件 usbd_cdc_interface.c | 2026-08-17 Phase A+B | 编译 0 Error；拔 USB 观察 |
| IWDG 看门狗 ~2s（寄存器级，PWM 前启动） | 固件 main.c | 2026-08-17 Phase A+B | 编译 0 Error；故意死循环观察复位 |
| `H` 命令（全回中位 90° + 清急停） | 固件 usbd_cdc_interface.c | 2026-08-17 Phase A | 编译 0 Error；发 H 观察 |
| `E` 命令（急停，PWM 停止） | 固件 usbd_cdc_interface.c | 2026-08-17 Phase A | 编译 0 Error；发 E 观察 |
| 堆 1KB / 栈 2KB（修复 USB 枚举失败） | MDK-ARM/startup_stm32f103xb.s | 2026-08-17 Phase B | 编译 0 Error；真机枚举 |
| 急停/超时/断开三状态门控 | 固件 usbd_cdc_interface.c | 2026-08-17 Phase B | 编译 0 Error；review-hw 复核通过 |
| 负角度拒绝（%d 解析） | 固件 usbd_cdc_interface.c | 2026-08-17 Phase B | 编译 0 Error |