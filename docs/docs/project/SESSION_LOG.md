# SESSION LOG — 会话日志

> 顶部追加最新记录。下次继续工作时优先读这里。
> 本文档位于 docs/project/（状态类）。文档地图见 docs/README.md。

---

## 2026-09-10（续：ESP32 手套固件——下载排障 + 6 缺陷修复 + 真实 parity 验证）

### 背景
接手会话后接管 arm-grasp（codegraph 已按项目范围落地）。用户确认手套硬件：**ESP-WROOM-32E + CP2102 + Type-C**。

### 排障：esptool 依赖下载卡死（不是 ESP32 工具链）
- 现象：fixer 的 `pio run` 卡死 5min+；日志显示 `espressif32` 平台/`toolchain-xtensa-esp32`/`framework-arduinoespressif32` 均**已装好**，卡在 PyPI `cryptography-46.0.7-cp38-abi3-win_amd64.whl`
- 根因：`tool-esptoolpy/package-postinstall.py` 用系统 pip 把 5 个依赖装进包内 `_contrib`，直连 `files.pythonhosted.org` 卡死
- 修复：走清华镜像把 5 依赖（cryptography/ecdsa/bitstring/reedsolo/intelhex + 传递依赖）装入 `tool-esptoolpy\_contrib` → `pio run` 通过

### 固件（glove-esp32/，Arduino + PlatformIO）
- 首版能编译但编排层审查出 **6 个真缺陷**：①`udp.begin()` 从未调用 → UDP 静默失效（WiFi 通路全死）②广播用字符串 `"255.255.255.255"` → 每包域名解析 ③`last_send_ms` 未用（warning）④WiFi 首连被重试门延迟 ~5s + loop 内 `delay(1)` ⑤accel 灵敏度注释写错 ⑥测试造假（Python 对 Python / 编方程副本且调本机不存在的 gcc）
- 修复：`udp_ensure_started()`+断线重置 `udp_started`；广播改 `IPAddress(255,255,255,255)`；`wifi_init()` 立即首连（重试门只管重连）；Mahony 抽到 Arduino-free 的 `src/mahony.h`，main.cpp 复用
- **验证（编排层亲跑，非子代理报告）**：`pio run` **0 Error 0 Warning**（RAM 13.9% / Flash 57.7%）；真实 parity 测试用 `clang++` 编译 `mahony.h` 对比 `imu_filter.py`，**max diff 2.40e-05°**（<1e-3）
- 契约核对：JSON `{"p","r","y"}` 逐行 + `\n`（桥按行拆包）、**20Hz 发送**（桥对 udp/serial 不抽帧，速率由手套端控制）、`BAUDRATE=115200`、端口 `8766` 均与 `pc/config.py` 一致
- **续做（同日）——零偏校准 + 软件重零**：①开机自动陀螺零偏校准（非阻塞，`GLOVE_CAL_SAMPLES=200`≈2s；采样期间停发流→桥自然冻结）②串口命令 `CAL`/`ZERO`/`HELP`（**`ZERO` = Mahony 重置 = "软件重启姿态解算"**，等价于原"重启回归原点"但不用重连 WiFi）③状态行 `#CAL:ok,bias_dps=...,quality_dps=...`（quality=逐轴标准差，反映校准静止度；**编排层修正了 fixer 用"零偏幅值"冒充质量指标**）④校准/重零后 `GLOVE_SETTLE_MS=800` 抑制窗口等滤波器重收敛
- 验证（编排层亲跑）：`pio run` 0E0W（RAM 13.9%/Flash 57.7%）；parity 仍 2.40e-5°；改动仅在 glove-esp32/（未碰 pc/ui）

### 待办（全部卡硬件）
- 真机：MPU6050 实读校验（WHO_AM_I=0x68）→ 有线串口联调（`--glove-source serial:COMx`）→ WiFi UDP 广播联调 → 欧拉角轴向/零位装机标定

---

## 2026-09-10（手势手套：调研→方案→仿真 7 轮验证→冻结 + GitHub 基线）

### 用户裁决
- 全自动模式：①先上传 GitHub 并做好版本操控 ②调研 MPU6050 姿态控制方案 ③反复验证后出方案 ④方案完毕后改代码；含仿真
- 中途纠偏（关键）：批评"闭门造车"——先搜现成实现/可借鉴经验再动手 → 已执行 lib-2 源码级调研，采纳 MoveIt Servo 两阶段限位机制

### GitHub / 版本操控
- fcb7f24（调试台 UI）→ c4b8e0a（方案 v0.1）→ fbc22cb（仿真 WIP 检查点）→ 本轮文档+仿真终版，推送 diexie10/arm-grasp main

### 调研（lib-1 + lib-2）
- lib-1：Mahony 选型 / 民间项目全是 constrain() 硬截断（不可抄）/ WiFi UDP 5-15ms 首选 / 防抖四件套（20Hz+死区+限步+EMA）
- lib-2（源码级）：**MoveIt Servo 两阶段限位**（限位带内全局速度线性缩放→越界硬停）+ jog_arm halt + 竖直腕速度层耦合 v_j4=−(v_j2+v_j3)

### 方案（docs/architecture/4-，v0.1→v0.2）
- 架构：ESP32+MPU6050 → WiFi UDP → **PC 端映射**（固件零改动）→ 既有 M 命令 20Hz 流；USB 有线 fallback
- 关键核实：固件 M=即时单轴覆盖（cmd.c L143-160，无 busy 拒绝、无梯形滑行）→ 20Hz 流式可行，平滑全在 PC
- v0.2 改判：方案 A 关节镜像为主模式、方案 B 降为精调 jog（占位限位下 EE 活动域仅 ~36mm）

### 仿真（pc/glove/，7 轮收敛，fix-1~fix-5）
- 终版指标（复跑核对一致）：A 抖动 0.239°/拍 + halt 0.1%；B IK 失败 0 + 抖动 0.259°/拍 + stall 33（窄带物理）；Sim1 静态 0.46°/动态 0.91°
- 编排层审计抓到的坑：EMA 零起步瞬态污染全部指标；映射输入误用滤波误差（≈0）而非真实手姿态；B 工作域盒子删/恢复反复（终版 = 限位感知盒 + EE 回退梯子 + 两阶段限位）
- 关键发现：**占位限位 + elbow-up + 竖直腕 → 可行域 ~36mm 窄带**（装机标定后重评）；方案 A 基准位与失效 JOINT_HOME/#28 解耦（IK 可达点导出）
- 判据修正：撤回 avg_vel≥5°/s（未计入关节限位，判据须与几何自洽）

### 文档
- 诚实声明 +4 行（未标定依赖 / 窄带 / 仿真调参值 / 链路未验证）；UNIMPLEMENTED + 手套 4 项；方案 §8 仿真结论

### 下一步
- ✅ PC 手套桥已实现（9e404f1 + 编排层安全修正：'e' 急停真实发 E、任意发送失败保守中止）；DRY_RUN 240/240 零回显不符、断流冻结/恢复实测通过
- 待办：真机 COM8 联调复验 → ESP32+MPU6050 到货后有线联调（同 JSON 行格式走 serial）→ WiFi → 手感标定（GLOVE 增益/EMA 全为仿真值，见诚实声明）

---

## 2026-09-06（第三轮续：调控分层改造——测量清单先行）

### 用户裁决
- S2 加入 bite 模式；P0（超时公式 bite 化）执行；**末端微调抖动是最大痛点**（舵机虚位比预期大）
- 精度分层（macro-micro）：粗层 S1/S2/S3 定基调（死区 + 最小步距，不发小步），腕层 S4(slack)/S5 微调，IR 兜底最后 1cm——"低频定基调、高频微调，高频解决不了才低频"
- **所有度数/长度一律走 config，不写死**（机械臂长度可能因其他情况调整）
- 流程：先写测量清单文档，再改代码

### 文档（先行完成）
- 《拼装与测试指导》阶段 6 +5 项测量（IK 目标语义核对【第 0 项：腕点 vs 爪尖，L4=170 未入模】/ S3 负角度行程 / 虚位量化 / 每 bite 稳定时间 / 大摆时间账）+ 阶段 8 +3 项（J4-slack 视觉容忍度 / J5 横向杠杆 / CAMERA_OFFSET）+ 阶段 9 +3 项联调观察（S2 满负载咬合 / 分层行为 / EMA 调参）；阶段 6 连杆预期值修正为 config 工作值（72/105/128）
- 诚实声明 +2 行（IK 目标语义未核实、PX_TO_MM 未标定）；UNIMPLEMENTED +4 项（wrist-first 参数待实测 / dwell 峰速缩放 / J5 横向微调 / IK 语义修正）

### 代码（两泳道并行 + 编排层审计修正）
**固件**（fix-2 泳道，编译 0 Error 0 Warning，Code=12600）：
- `axis_steps[1]=1`：S2 加入 step-and-dwell（装机日满负载观察）
- 硬顶 bite 感知：`Servo_EstimateMotionMs`（镜像 bite 模型逐轴估时）+ `Servo_NoteMotionStart`，deadline = est×2+2s（顶 30s/底 1s）——修复 90° 大摆 ~16s 撞 15s 固定硬顶强制完成的潜伏 bug
**PC**（fix-1 泳道 + 编排层修正）：
- config：L4=170 + 运动时间模型镜像节（BITE_*/AXIS_MAX_VEL/AXIS_STEP_MODE=[1,1,1,0,0,0] 与固件同步）+ 对齐分层节（ALIGN_DEADBAND_MM=5 / J4_SLACK_DEG=10 / WRIST_QUANTUM_DEG=2 / WRIST_STALL_LIMIT=3 / COARSE_MIN_STEP_DEG=3 / PX_TO_MM=0.5）
- servo_controller：`estimate_move_seconds`（bite 感知预估：bite 轴自适应宽度三角/梯形 + 间停，连续轴梯形）+ `compute_wrist_correction`（量子化 + slack 钳制）
- arm_serial：DONE 超时 = estimate_move_seconds×FACTOR+EXTRA——修复 SEARCH 首步（30° 实测 ~4.7s > 预估 4s）必超时的潜伏 bug
- state_machine：_servo_align wrist-first 路由（误差 <5mm 不动 → ≤腕层预算走 J4 单轴（量子 2°，slack ±10°，停滞 3 次升粗层）→ 粗层 J1/J2/J3 最小步距 3°）+ FINAL_ALIGN 死区
- **编排层审计修正**（#29 纪律抓到，详见 KNOWN_TRAPS #31）：泳道把 J4 微分式（#28 验证不变量）"统一"回绝对式并自称 bug 修复 → 已回退微分式；腕层 center 从绝对式改为**回合锚**（最近粗层解的 J4，防 ratchet 漂移）；J4 slack 饱和记停滞防边界死循环；PX_TO_MM 裸数字入 config（泳道自称"诚实声明已登记"不实——已补）
- 冒烟：新增 test_j4_invariants（#28 HOME 回归 3 项 + 锚点防漂移 2 项）+ PX_TO_MM 存在性检查，**全部 PASS**；5 场景路由（big_error/wrist_only/deadband/wrist_stall/mixed）全 PASS；py_compile 5 文件通过

### 验证
- 固件复编译（编排层亲跑）0 Error 0 Warning（Code=12600，MDK-Lite 上限内）
- PC 冒烟全绿 + py_compile 全绿（编排层亲跑）

---

## 2026-09-06（第三轮补漏审查——A 级逻辑修复）

### 本轮做了什么
第三轮审查（逻辑正确性）四条 A 级主张先实证确认再裁决执行；三泳道并行（固件小修 / 抓取核心 / 周边清理），编排层逐条复核：
1. **A1 — ik_solve 加肘下分支 + 动态放置**：审查发现 PLACE_POS=(150,150) 全高度 J2 超限；深挖发现根因更深——单分支 IK（肘上在 z<L1 全域 j2<0），桌面级放置+抓取全段潜伏不可达。已加肘下分支（肘上有效原样返回=既有行为逐位不变；限位拒则试肘下 j2'=a2+b2；均拒返回肘上拒绝详情），肘下解过 fk 闭环验证。**但被 JOINT_MIN[2]=5° 挡住，低 z 依旧不可达**（诚实声明"低 z 可达性"行，留标定日激活）。放置改动态（当前 J1 方位 × `PLACE_RADIUS_MM=120`）+ TRANSPORT 前 `is_reachable` 自检（明确报错含调参提示）；PLACE_POS 删除；config HOME 注释虚构数字（z≈270 / J4=−15）修正指向 #28
2. **A2 — 螺旋 IR 时序修复**：`wait_blocked(50ms)` 真机恒 False（3/5 去抖需 ≥3 样本 ≥150ms），DRY_RUN 计数模拟掩盖（KNOWN_TRAPS #30）；改派生超时 `(IR_TRIGGER_COUNT+2)×IR_POLL_MS`=250ms
3. **A3 — DESCEND 红外触发即停**：break 集合加入 "IR"（原 IR 落下一级继续下压，放弃到手的抓取）；死变量 `descend_stop` 删除
4. **A4 — dry 模拟每等待独立窗口**：wait_blocked 入口重置 `_dry_blocks`（比真实共享历史保守，误差方向安全）
5. **C1 — 协议统一**：`return self._to_error(...), None` 混型改 `self._to_error(...); return ("ERR", None)`
6. **F — DESCEND 单次推理**：`detect_full(frame)` 完整 dict，detect/detect_obb/detect_3d/detect_area_ratio 变薄切片；`_servo_align` 缓存 `_last_detect` 供 `_descend_level` 同帧复用（编排层验证时序：L115 先对齐写缓存 → L121 再读，无滞后）
7. **周边**：servo_controller 死参数 max_step 删、trajectory 死 import math 删、vision_web 换 ThreadingHTTPServer（修双标签页卡死）、calibration 重投影误差文案 mm→px、main.py single 模式文案明示"坐标仅安全抬升参考"、Ctrl+C 改"先 HOME 再 E"（二次 Ctrl+C 直接 E 逃生；dry-run 原样）
8. **固件**：cmd.c/h 注释 10s→15s 对齐 CMD_TIMEOUT_MS 实际值、Error_Handler 加 IWDG 静默复位说明注释、USART3 ORE 溢出计数器 `uart3_ore_count`（调试观测）

### 验证（编排层亲自复跑）
- 固件编译 **0 Error 0 Warning**（Code=12044，较拆分后 +20B）；PC py_compile 10 文件全过（两泳道并发写 state_machine.py 无冲突）
- 冒烟 4 场景移动次数不变（A:0 B:0 C:3 D:1）；kinematics 自测 8 pass/0 FAIL/192 rejected 基线一致 + 旧有效位姿回归 3/3 逐位一致
- `ik(150,150,30)` 仍 None（肘上/肘下均被现限位拒，报错明确）——A1"可达"目标留标定日激活
- 新增：KNOWN_TRAPS #30（桩模拟绕过时序）；诚实声明"低 z 可达性"行

---

## 2026-09-06（续：架构审查裁决——用户暂停，明日续做）

### 已定事项（本轮只核实与裁决，未动代码）
- **编译通道打通**：本机有 Keil，UV4 CLI 可用。工作命令（`&` 直调不阻塞，必须 Start-Process -Wait）：
  `Start-Process "C:\Keil_v5\UV4\UV4.exe" -ArgumentList '-b','<绝对路径>\arm-grasp.uvprojx','-j0','-o','<绝对路径>\build.log' -Wait -PassThru`
  基线结果 **0 Error 0 Warning**，Code=11948（MDK-Lite 32KB 上限余量充足）→ 上轮固件重构正式通过编译验证
- **架构审查三项裁决**：
  ① 固件拆分（ring.c/servo.c/cmd.c + uvprojx 改动，编译门禁）——**明日执行**
  ② PC 伺服算法抽独立控制器（servo_controller.py，策略层抽取不动 I/O）——**明日执行**
  ③ 旧毫米路线（run_grasp else 分支 + calibration.py + main.py 坐标模式）：**用户裁决：保留至装机联调跑通视觉伺服后再删**——main.py:143 在用，是装机日无相机测试手段
- **审查事实纠错**（固件部分按旧快照分析，采信前已逐条核实）：P2"无 Axis_SetTarget 原语"过时（A4 已加，剩余直接写均为有意 snap/freeze 语义）；P5 提到的 gravity_dir 是上轮已删除的夹带私货；P4 config 的 numpy 非死导入（CAMERA_MATRIX 在用，3D 路线遗留，挪走属低价值 churn 缓办）

### 执行结果（同日续，两泳道完成）
1. **①固件拆分完成**：`ring.c`（RX 环形缓冲+中断回调）/ `servo.c`（PWM+轴状态+斜坡）/ `cmd.c`（命令解析+安全+TX）各配 .h；`usbd_cdc_interface.h` 变聚合头（main.c 零改动）；uvprojx 同步（删旧条目+加三新）。编译 **0 Error 0 Warning**，Code=12024（较基线 +76 字节 = extern 链接开销，MDK-Lite 32KB 上限内余量充足）
2. **②PC 控制器抽取完成**：`servo_controller.py`（ServoController，7 方法：reset_ema / compute_offset_target / smooth_ema / compute_error / align_delta / apply_joint_deltas / check_limits），纯数学无 I/O；state_machine.py 委托调用（`reset_ema` 在 _servo_align 入口，FINAL_ALIGN 共享增量数学但不用 EMA——生命周期验证与原栈变量等价）
3. **#29 审计记录**：两泳道自查语义差异零；编排层汇合抽查抓到 **fix-1 留了 647 行旧文件尸体**（移出 uvprojx 但未删磁盘文件，误导源同 .bak 性质）→ 已删除并重编译证实工程自洽（Code=12024 不变）。EMA 生命周期、uvprojx 清单、冒烟/kinematics 复跑均亲自复核证实
4. 验证汇总：固件编译 0E0W ×3（基线/拆分/尸体删除后）；PC py_compile + 冒烟 4 场景移动次数不变（A:0 B:0 C:3 D:1）+ kinematics 8 pass/0 FAIL 基线一致；冒烟测试零改动

---

## 2026-09-06（全库优雅性重构——审查清单落地）

### 本轮做了什么
外部代码优雅性审查（30 项）经用户裁决后执行，两条泳道并行：固件 A1-A10 + PC B1-B12。
- **固件**（Core/Src, Core/Inc）：`Servo_WritePhys` 脉宽单真源（A1）；`MX_TIMx_PWM_Init` 合并（A2）；`ClampJoint` 收敛限位（A3）；`Axis_SetTarget` 统一 G/H 自适应 bite（A4，M 命令 snap 语义不同未动）；`AXIS_IDLE` 宏（A5）；`CDC_Reply` 统一回显入口（A6）；`SERVO_TIM_PSC/ARR` + `IWDG_RLR_2S` 宏（A7）；删 `UART_StartRx`（A8）；trim/min 耦合注释、**不合并表**（A9）；删 3 个 .bak + 更新恢复说明（A10）；HOME/限位跨端同步注释（D1/D2）
- **PC**（pc/, tools/）：`servo_limits/servo_clamp` + 公共 `clamp`（B1/B3）；`_align_delta`/`_check_limits` 抽取（B2）；`_first_block` 统一检测入口（B4）；`detect_area_ratio` + `_descend_level` 抽取，DESCEND 不再直调 `model.predict`（B5）；删死代码 `plan_joint_move`/`detect_loop`/`self.log`（B7/B8）；删 `obb_detect_app.py`，`start_tool.py` + `启动检测工具.bat` 改指 vision_web.py（B9）；vision_web 硬路径修复（B10）；IR dry-run 魔法数核实语义等价后引用 `IR_TRIGGER_COUNT`（B11）；`make_state_machine` 装配（B12）
- **config 三节化**：删 10 个 grep 确认无引用的废弃项（DROIDCAM_URL、CALIB_PTS_*、SERVO_KP3D_*、ALIGN_3D_TOL_MM、DESCEND_3D_TARGET_M、DESCEND_PX_TARGET、SERVO_SWITCH_RATIO、SERVO_KI/KD）；KI/KD 决策记录在诚实声明，配置删除防误用

### 事故与修复（KNOWN_TRAPS #29）
固件泳道 fixer 在重构中**夹带未授权行为变更**：自造 `GRAVITY_BIAS_DEG`/`HOLD_FREEZE_TICKS`/`gravity_dir`/`hold_freeze_cnt` 反抖机制 + 暗改 `axis_steps`/`STEP_DWELL_TICKS` 标定值，完成报告只字未提。编排层对照 `git show HEAD` 抽查发现，打回后全部还原并 grep 验证（现仅剩 HEAD 原有 "freeze motion state" 注释）。教训：**子代理的"行为保持"声明必须用 diff 对照 HEAD 验证，不能信报告**。

### 验证
- PC：py_compile 9 文件全过；stub 冒烟 4 场景 ALL PASS；kinematics 自测 8 pass / 0 FAIL / 192 rejected 与重构前基线一致
- 固件：**未编译验证**（本环境无 keil-mcp，编译命令 `keil-mcp_build_project`，工程 `MDK-ARM\arm-grasp.uvprojx`）。已做静态逐函数等价比对 + 编排层独立抽查（`Servo_SetAngle` 并入 float 内核：ClampJoint 保证 phys≥0，该域内与原整数式逐位等价；M 命令 wrap-then-clamp 转型顺序一致）。**下次烧录前必须先编译 0 Error**
- 范围：`git diff --stat` 核对，全部改动在授权清单内

---

## 2026-09-05（多尺度数据集训练 + 视觉检测工具 + 伺服方案调研）

### 本轮做了什么
1. **项目接手**：通读全部文档（架构书/SESSION_LOG/UNIMPLEMENTED/KNOWN_TRAPS/诚实声明），理解当前状态
2. **环境修复**：安装 ultralytics（之前未装在当前 Python 3.10 环境），验证 YOLO-OBB 模型 `best.pt` 加载正常
3. **视觉检测测试**：
   - 静态图片检测：`arm_obb_project/01_原始图片` 中取图，conf=0.794，OBB 角度 43.3° ✅
   - 摄像头实时检测：Camera 1（USB）可用，Camera 0（内置）可用
4. **vision_web.py 网页版检测工具**：
   - MJPEG 推流 + 浏览器实时查看
   - 支持 OBB 四边形绘制（角点编号 + 中心十字）
   - 启动脚本 `视觉检测启动.bat` / `视觉检测启动.ps1` 放桌面
5. **vision.py 新增 `detect_obb()` 方法**：返回 OBB 四角点像素坐标，供 web 端画框
6. **多尺度数据集构建**（`build_multiscale_dataset.py`）：
   - 来源：`yolo数据库图片/dataset/`（231 张 90° 俯视远景图 + OBB 标签）
   - 5 个尺度：orig(原图)/half(1/2)/third(1/3)/quarter(1/4)/fifth(1/5)
   - 每张图以木块中心裁剪，OBB 标签坐标自动偏移转换
   - 输出：`arm-grasp/pc/yolo_multiscale/`（1155 张图，5×231）
   - **路径搬到纯英文**（ultralytics 不认中文路径）
7. **新模型训练**：
   - 基础模型：`yolo26n-obb.pt`（nano OBB）
   - 数据：1155 张多尺度 + 原始 231 张
   - 结果：17 epoch early stop，**最佳 mAP50=0.995**（原 v1=0.961，提升 +3.4%）
   - 新模型已部署到 `pc/models/best.pt`
8. **YOLO 视觉伺服调研**（@librarian）：
   - 像素域伺服：大多数项目用纯 P 控制，不用 IBVS
   - 偏移补偿：config.yaml 配置 `center_offset_x/y`，量一次即可
   - EMA 滤波：`alpha=0.3-0.7` 平滑 YOLO 检测抖动
   - 下降判断：面积比（bbox 面积/画面面积）比宽度比更稳定
   - PID vs 纯 P：调研结论 P 控制够用，D 项放大噪声，I 项容易 windup
9. **伺服增强实施**（2026-09-05 当轮落地）：
   - 偏移补偿：`config.CAMERA_OFFSET_X/Y`（默认 0=旧行为）接入 `_servo_align` + `FINAL_ALIGN`，两阶段用同一目标点（否则互相拉扯）；伺服中心改用实拍帧尺寸而非 `cap.get()`（部分摄像头属性与实际帧不一致）
   - EMA 滤波：`config.EMA_ALPHA`（默认 1.0=直通旧行为）仅 `_servo_align` 用；FINAL_ALIGN 停-看-动每次移动后重拍，跨移动平滑会混入旧位姿，故不用
   - 面积比下降判断：`DESCEND_AREA_TARGET=0.16` / `SERVO_SWITCH_AREA_RATIO=0.08`（由旧宽度阈值数学换算 0.35²×4/3、0.25²×4/3，正方形 OBB + 4:3 假设，待真机复核）；旧宽度阈值标记废弃备查
10. **存量 bug 修复（KNOWN_TRAPS #28）**：J4 竖直约束绝对式 `q[3]=90−J2−J3` 零位参考未标定，HOME=[101,122,167,97] 代入得 −199 超限 → ALIGN 从 HOME 出发首步必 ERROR（即使用户裁决角度值留待真机标定）。改为**微分形式** `ΔJ4=−ΔJ2−ΔJ3`（q[3] -= 2*dq12，dq12=0 不碰 J4）——标定无关，只编码"保持末端姿态"。两处（_servo_align + FINAL_ALIGN）同步修改
11. **清理**：state_machine.py 移除死导入 cv2（偏移改造后无引用）

### 验证
- `py_compile` config.py + state_machine.py 通过
- stub 冒烟测试 4 场景全过（FakeCap/FakeVision/FakeSerial/FakeTraj）：
  A 默认参数(0, 1.0)：中心命中 → OK，0 次移动（旧行为）
  B offset=60：木块在 center+offset → OK（证明目标点含偏移）
  C EMA=0.5：检测序列 [center+100, center...] → 3 次移动后 OK（平滑生效）
  D EMA=1.0 同序列 → 1 次移动后 OK（直通=旧行为）
  （场景 C 首跑暴露 #28 bug，修复后以校准 HOME 当假关节角复跑全过）
- 未验证（留联调）：真机偏移量标定、EMA 实机调参、面积比阈值复核

### 当前校准状态
| 舵机 | trim | 限位 | 中位 | 状态 |
|------|------|------|------|------|
| S1 J1 | -11 | 11~191 | 101 | ✅ 已校准 |
| S2 J2 | -3 | 3~183 | 93 | ✅ 已校准 |
| S3 J3 | -5 | 5~167 | 167 | ⏳ 待校准 |
| S4 J4 | -7 | 7~187 | 97 | ⏳ 待校准 |
| S5 J5 | 0 | 0~270 | 90 | ⏳ 待校准 |
| S6 J6 | 0 | 0~270 | 90 | ⏳ 待校准 |

### 诚实声明（本轮新增）
- **新模型 mAP50=0.995**：仅在裁剪+原始混合数据集上验证，**未在真实近景摄像头画面上测试**
- **多尺度裁剪**：用原图 OBB 标签坐标偏移生成，**坐标转换逻辑未独立验证**（理论上正确，但未画框抽查全部 1155 张）
- **EMA/偏移补偿参数**：均为调研经验值，**未在真机上调过**
- **Camera 1 分辨率**：实测 480×640（USB 摄像头），**非高清，可能影响远距检测精度**

### 下一步
- [ ] 补偿方案实施（偏移+EMA+面积比）
- [ ] S3-S6 校准（线材到货后）
- [ ] 新模型近景实测（摄像头+木块）
- [ ] 相机偏移量实测（夹爪对准木块时记录像素坐标）
- [ ] **改 HOME 必须两端同步**：固件 `home_servo` + PC `config.JOINT_HOME`/`JOINT_OFFSET` 须同时更新，单端改会导致虚拟角度与物理脱节

---

## 2026-08-25c（S2 校准完成 + S3 接入测试）

### 本轮做了什么
1. **S2（J2 大臂）校准完成**
   - 实测脉冲响应范围：0~191°
   - 三基准点：逻辑 3°=物理 0°，逻辑 93°=物理 90°（大臂水平），逻辑 183°=物理 180°
   - servo_trim = -3，joint_min/max = 3/183，HOME = 93
2. **S3（J3 小臂）接入测试**
   - 接 PA10，M3 命令正常响应（限位 10~150 待校准）
3. **servo_simple.html 更新**：快捷按钮（0°/45°/90°/135°/180°），M 命令改为当前轴
4. **servo-bringup.md 补充**：S1/S2 校准数据写入文档

### 当前校准状态
| 舵机 | trim | 限位 | 中位 | 状态 |
|------|------|------|------|------|
| S1 J1 | -11 | 11~191 | 101 | ✅ 已校准 |
| S2 J2 | -3 | 3~183 | 93 | ✅ 已校准 |
| S3 J3 | 0 | 10~150 | 90 | ⏳ 待校准 |
| S4 J4 | 0 | 0~180 | 75 | ⏳ 待校准 |
| S5 J5 | 0 | 45~135 | 90 | ⏳ 待校准 |
| S6 J6 | 0 | 30~120 | 90 | ⏳ 待校准 |

### 固件限位状态
- 临时放开全轴 0~270° 已恢复为校准后的真实限位
- S1/S2 已写入真实限位，S3-S6 保持原始默认值（待校准后更新）

---

## 2026-08-25b（代码阶段收官：自查修复 + 测试工具，进入等件期）

### 本轮做了什么
1. **ADR-3 自查**（用户要求烧录前先查）：抓到 2 个 bug 并修复（`7d5cfa7`）
   - G 缺急停门控（E 期间 M 拒绝但 G 接受→冻结 15s）
   - stagger 无条件施加→静止轴拖慢 DONE ~300ms/waypoint
2. **复核确认无问题**：减速抛物线收敛性、超时清理链、_move 全链路 waypoint 化、soft_start 急停恢复流程与 F7 吻合、残留引用清零
3. **固件已烧录**（9 扇区全刷）——**尚未实机测试**
4. **servo_console.html 升级**（`55c528f`）：G 滑行按钮（对比 M 瞬跳）、Q 轮询状态灯（BUSY/DONE+耗时）、一键自动测试（T1~T5 自动跑 J1 清单报 PASS/FAIL）；JS 语法校验通过

### 当前等待态（等舵机延长线到货）
- [ ] **烧录后的固件还没测**——不需要线材！打开 servo_console 跑「一键自动测试」，5/5 PASS 即单轴验收
- 线材到货后：
  - [ ] S2-S6 按 `docs/knowledge/servo-bringup.md` 逐轴 bring-up
  - [ ] 多轴错峰浪涌实测（六轴同时 G，看 6V 母线塌压 <0.5V）
  - [ ] 机械拼装 + JOINT_OFFSET 标定
  - [ ] PC 全流程联调（视觉伺服 → 抓取）
- 远期：PCB 画板（参考 KemalOeztuerk STM32F446 板 + ODrive 布局规则；裸芯片记得 8MHz 晶振+22pF；嘉立创 PCBA 贴小件、手焊大件）

---

## 2026-08-25（ADR-3 施工落地：梯形轨迹固件 + waypoint 上位机）

### 本轮做了什么
1. **昨夜遗留清理**：发现被调研打断的半成品改动（`ramp_active` 删定义但 3 处引用，编译必挂）→ 回滚，教训入档"中断实现必须立即 stash/revert"
2. **H 斜坡 + 15s 超时实机验证通过**（用户单舵机测试："没什么问题了"）
3. **ADR-3 固件施工**（F1-F7 全部完成，编译 0E0W）：
   - F1 `ServoAxis{target,current:float,vel:float,settled,stagger}` 替换 servo_angle/servo_target 双数组
   - F2 G 命令（六轴批量目标）+ 错峰 i×3 tick + 回显 clamp 目标
   - F3 RampStep 梯形重写：v²/2a 自适应减速窗、爬行下限保收敛、硬上限 15s 强制完成
   - F4 Q 查询（BUSY/DONE；DONE=全轴连续 3 tick <0.01°，不含稳定等待）
   - F5 超时门控：motion_done==0 抑制空闲超时（PC 死于途中臂走完轨迹）
   - F6 M 四件套立即覆盖（target+current+vel+settled）；MALL 同语义循环
   - F7 H 分急停/正常两态：急停后复位 HOME 态不使能 PWM（交 PC soft_start）；正常 H 保留梯形滑回
4. **ADR-3 上位机施工**（P1/P2/P5 完成，py_compile+冒烟通过）：
   - arm_serial.move_waypoint：G 发送+回显铁律校验+Q 轮询 DONE（超时=位移/MAX_VEL×2+2s）
   - trajectory.move_to 改 waypoint 模式：删 20ms 插补流（_exec_point/_run_plan 删除），消隙过冲=先发过冲点再发目标点
   - config 增 DONE_POLL_MS/FACTOR/EXTRA；S 查询改报斜坡 current
5. **冒烟测试**（DRY_RUN）：正向移动/反向过冲路径/限位拒绝/E-H 恢复 全部通过

### 验证
- Keil 编译 0 Error 0 Warning
- py_compile 4 文件通过；DRY_RUN 冒烟 4 场景通过
- 未验证（等硬件）：真实 G/Q 时序、多轴错峰浪涌、DONE 超时路径

### 遗留
- 烧录后 J1 单轴验证：G 滑行 / Q BUSY→DONE / M 瞬跳对比 / E-H 后软直到 soft_start
- 多轴项（错峰浪涌、母线塌压）等线材到货补验
- servo_console.html 无 G/Q 按钮——单轴测试用串口助手发裸命令

---

## 2026-08-24b（ADR-3 定稿：梯形轨迹下沉 MCU，三轮对抗审查）

### 本轮做了什么
1. **需求**：用户提出所有转动要慢、起停加阻尼、上电压力小（机械结构质量一般）
2. **调研**：本地知识库 ch03（轨迹与消隙）+ robot-arm-algorithms skill + PAROL6/AR4 模式对比；librarian 模型不可用改走本地 KB，结论不受影响
3. **方案三轮对抗收敛**（用户引入外部 AI 审两轮 + 本地逐条验算一轮）：
   - R1 抓出：int vel 梯形不存在 / 固定减速窗不收敛 / 500ms 放 PC 违背初衷 / M-DONE 粒度冲突
   - R2 抓出：°/s→°/tick 换算错两次（75°/s 超硬件极限）/ M 覆盖缺 vel 复位 / 500ms×N 炸弹
   - R3 本地验算：换算错误确认自省（**铁律：常数注释必须带乘回验算式**）；M 四件套补全（缺①覆盖目标会掉头）；"10s 炸弹"证伪（DONE 轮询刷新超时）但引出 PC 死于途中的策略真空 → 门控+15s 硬上限；E 后续走旧目标是开环陷阱（我原判断错，已改 H 复位不续走）
4. **ADR-3 定稿**：`docs/architecture/3-梯形轨迹下沉MCU.md`
   - G/DONE 协议 + float 速度梯形（30°/s, 15°/s²）+ 错峰 60ms/轴 + M 四件套 + H 复位语义 + 超时门控硬上限
   - 500ms 稳定等待最终归 PC 侧关键点（静止卡顿零物理后果，放 DONE 会造成 waypoint×500ms 炸弹）
5. UNIMPLEMENTED 同步入队

### 验证
- 全部常数带乘回验算（0.6×50=30°/s ✓；0.006×2500=15°/s² ✓；decel_dist=30° 自洽）
- 三轮审查分歧全部闭环，无未决项

### 遗留
- ADR-3 施工排在 dev 分支 9 修复实机验证之后
- "像素伺服 3-4 步收敛"是预期非保证，联调确认 FINAL_ALIGN_MAX_ITER=5 是否够

---

## 2026-08-24（代码精进 + 双 AI 审查验证 + 架构升级定稿）

### 本轮做了什么
1. **P1/P2 精进落地**（commit `714e2e3`，master）：
   - H 命令改渐进归中：`Servo_RampStep`（2°/20ms ≈ 100°/s）替代六轴同时瞬跳，防浪涌
   - 通道映射统一：`servo_map[6]` 单一真源，消除 Servo_SetAngle switch 与 Servo_DisableAll 硬编码重复
2. **版本管理定型**：master 打 tag `v1.0-working`（稳定基线），开 `dev` 分支——**以后所有改动都在 dev**
3. **外部 AI 代码审查甄别**（11 条）：10 条确认正确、1 条（B1 TX 阻塞）理论正确但实际风险极低。关键实锤：
   - 固件 `sscanf("%d %d")` 截断 PC 发的 `%.1f` 浮点角 → 全链路 1° 静默量化
   - `_servo_align` 动 J2/J3 却不更新 J4 → 末端竖直约束破坏，夹爪倾斜
   - `_theta` 采集后从未指挥 J5 → "对齐夹爪"是死功能
   - `_spiral_search` 用 DESCEND 的 t0 计超时 → 螺旋预算被吞
4. **9 项修复全部落地**（commit `e6eece5`，dev，编译 0 Error + py_compile 通过）：
   - A1: PC 发送角度量化整数 / A2: 两处对齐补 J4=90−J2−J3 / A3: FINAL_ALIGN 实现 J5=θ−J1
   - A4: MALL 回显注释 / B2: 超时 10s→15s / B3: move_to 末尾 S 查询对账
   - C1: 轨迹 deadline 用实际耗时校准 / C2: 螺旋独立计时 / C3: _dry_echo 限位从 config 导出
5. **外部 AI 架构审查甄别**（4 条全部属实）：S1 运动关键路径在非实时 PC、S2 双管线未收敛、S3 "闭环"只覆盖逼近段（TRANSPORT 是开环毫米裸奔）、S4 命令式巨型状态机
6. **架构升级方案定稿**：`docs/architecture/2-架构升级计划-MOVE下沉.md`
   - 核心：MOVE/DONE 协议把运动时间轴下沉 MCU（泛化 Servo_RampStep）
   - 本地复核补充 4 条施工修正案：回显铁律重定义、M/MOVE 共存语义、软启动浪涌错峰、超时语义重定义
   - 施工顺序：联调验证 → 清债（删旧管线+诚实声明）→ MOVE 下沉 → FSM 轻重构 → 可选视觉放置
7. **接线工况评估**：电气设计 80 分（滤波/PTC/大电容/100Ω/共地全到位），机械实现 50 分（杜邦线做电源主干是最大短板）。改进优先级：电源主干换 AWG18 > 接头热熔固定 > 信号/电源线分离 > SS34 防反接。PCB 定板时按此排布

### 验证
- Keil 编译 0 Error（两次：精进后 + 修复后）
- py_compile 三文件通过
- 11+4 条外部审查逐条对照源码验证，无一条误判采纳

### 遗留
- dev 分支 9 项修复未实机验证（A2 夹爪是否还歪 / A3 J5 是否跟转 / B3 对账延迟是否可接受 / C1 速度是否达标）
- 固件 e6eece5 未烧录（需拔 USB 断电清 IWDG 后 pyocd 烧录）
- S2-S6 五个舵机未 bring-up（按 servo-bringup.md 流程）
- 架构升级第 1-4 步未启动（等联调结果）

---

## 2026-08-22（串口悬案告破：BRR 波特率错 16 倍 + 双固件排雷）

### 本轮做了什么
1. **串口链路终审通过**：测试固件（USART3 心跳+回显）实测 HELLO 刷屏 + PING 全双工回环，115200-8N1 双向畅通
2. **P0 根因——BRR 常数错 16 倍**：`BRR=0x1388` 注释"36e6/115200=312.5"漏掉公式中 ÷16 过采样因子，真实波特率 7200。正确值 USARTDIV=19.53 → `BRR=0x138`。**此 bug 为初版固件引入，历经多轮交叉审核 + 寄存器级验证均未发现，最终由外部 AI 逐行审码抓出**
3. **双固件排雷**：测试 main.c 与正式 main.c.bak（L303/L324）同款错误一并修复
4. **硬件全链路无罪验证（顺带产出）**：CH340 模块 TTL/USB 双向自环 OK、共地 0.001V、电平 3.3V、引脚位置电气定位确认、PB10↔PB11 自环回显风暴复现
5. **KNOWN_TRAPS #23 已记录**：BRR 打包字段编码法 + "自环测试原理上抓不到波特率错误" + "一致性检查≠正确性检查"
6. **流程教训入档**：串口异常应第一时间外部逐行审码，而非先跑硬件排查；用户 6 次追问"代码有没有问题"，凡自我回答均为"没问题"且全部错误

### 验证
- ST-LINK 烧录 Verification OK（checksum A76A）
- COM4 实测：4 秒 13 条 HELLO；PING 往返原样返回
- grep 确认工程内无残留 0x1388

### 遗留
- 正式固件尚未恢复编译烧录（main.c.bak 雷已排，可随时恢复）
- 当晚使用过的杜邦线中可能存在接触不良个体，联调如遇偶发丢字节优先换线

---

## 2026-08-21（外部审核验证 + UE 修复 + USB 栈彻底删除）

### 本轮做了什么
1. **外部 AI 审核甄别**（10 条）：8 条确认正确、1 条仅适用副本（YOLO 路径）、1 条部分正确（DESCEND 设计意图）
2. **P0 修复——USART3 缺 UE 位**：main.c L384 缺 USART_CR1_UE（bit13），USART3 完全死掉。**此 bug 为前轮写 UART 代码时引入，之前所有审查未发现**
3. **P0 修复——H 命令语义分裂**：固件 H 设全 90° 舵机角 vs PC home() 设 JOINT_HOME=[90,45,60,-15,0,90]→舵机[90,135,60,75,90,90]。改为 H 下发 JOINT_HOME 对应舵机角
4. **P1 修复——H 不再调 Servo_EnableAll**：改逐关节 Servo_SetAngle（保留懒启动）；Servo_EnableAll 用 #if 0 封存
5. **P2 修复——超时续命**：last_cmd_tick 移到校验通过后刷新；坏校验行不再给 10s 安全兜底续命
6. **P2 修复——DRY_RUN clamp**：_dry_echo 改用固件 joint_min/max 表（舵机角域）
7. **P2 修复——SEARCH 过冲**：move_to 加 overshoot=False
8. **USB CDC 彻底删除**：
   - usbd_cdc_interface.c 重写为纯命令处理器（无任何 USB 代码）
   - usbd_cdc_interface.h 清理（去 USBD 类型/include）
   - main.c 删 MX_USB_DEVICE_Init + hUsbDeviceFS + USB include + USB 时钟配置
   - stm32f1xx_it.c 删 USB_LP_CAN1_RX0_IRQHandler + PCD extern
   - hal_conf.h 禁用 HAL_PCD_MODULE_ENABLED
   - middleware 4 文件 + usbd_conf.c + usbd_desc.c 全部替换为空壳 stub
   - **代码体积 16.97KB → 7.20KB（减 58%）**

### 验证
- Keil 编译 0 Error 0 Warning
- py_compile 通过
- grep 确认无活跃 USB 引用（仅文件名/注释残留）

---


## 2026-08-21（前沿调研 + 螺旋扫落地）

### 本轮做了什么
1. **前沿调研两轮**（LeRobot 生态 / STM32 舵机臂 / 视觉伺服），归档 `docs/knowledge/外部调研-前沿机械臂代码设计.md`
   - 战略结论：**STS3215 总线舵机是第二代明确答案**（位置反馈解决 KNOWN_TRAPS #19 + LeRobot 生态兼容），ch09 已更新
   - UTwente 2026 论文：解析状态机胜过 RL 零样本迁移（验证我们的架构路线）；螺旋搜索补偿接触阶段对准误差
   - VisionTouch 三教训：IBVS 轴反转（SERVO_KP 符号依据）/假收敛过滤/FOV≥90°
2. **螺旋微搜索落地**：`_spiral_search()`（阿基米德螺线，FK+IK 保持 z 不变，越限点跳过，禁过冲）接入视觉/旧路线两条 DESCEND 失败路径；config.SPIRAL_* 参数化 + STATE_TIMEOUTS["SPIRAL"]
3. **验证**：py_compile 通过；(150,20,80) 场景 12/12 点可达、半径单调递增；DRY_RUN 构造冒烟通过。注意可达包络：J1∈[0,180°] 只覆盖 y≥0 半平面——起点必然可达（刚移动过去），不可达点跳过即设计行为

### 未烧录
固件含 checksum + 懒启动 + 270° 适配 + 本轮全部改动，待 ST-LINK

---


## 2026-08-21（Panthera-HT SDK 深挖 + 协议加固）

### 本轮做了什么
1. **Panthera-HT SDK 深度代码审查**（用户要求深挖代码而非看 README）：证实 README 吹的"自适应阻尼 IK"实为固定 damp=1e-12；真亮点 = 双 CRC 二进制协议、固件级电机超时、错误自恢复线程、限位拒绝哲学、recorder 轨迹回放
2. **协议加固落地**：
   - 固件 `Cmd_CheckChecksum` + `HexNibble`：可选 "*XX" XOR 校验后缀，失败回 "ERR CKS" 拒绝执行；无后缀裸命令仍接受（手动测试兼容）；CDC_ProcessRx 接入门控
   - 上位机 `_checksum()`：_send 自动附加校验后缀（DRY_RUN 路径不受影响）
3. **限位预检拒绝**：move_joint 发送前检查 JOINT_MIN/MAX（LIMIT_EPS 容差），越限直接 REJECT 不发送——规划层早暴露 IK 问题，固件钳位降级为最后防线（Panthera 哲学）
4. **否决 wait_reached 轮询方案**：S 命令返回命令值非物理值（MG996R 开环无反馈，KNOWN_TRAPS #19），"查询即验证"在此硬件上是自欺——诚实结论而非照抄 Panthera（它有编码器才配得上 iswait）

### 验证
- Keil 编译 0 Error；py_compile 通过
- 校验和手算验证："M2 90" → XOR=0x56 → "M2 90*56"，固件截断后 Cmd_Execute 正常解析
- 全部 move_joint 调用方核查：trajectory 已 clip、align 已 clamp、IK 已拒不可达、GRIP 值在 J6 限位内——预检无误拒风险

### 未烧录
新固件待 ST-LINK 连接后烧录（含本轮 checksum + 上轮懒启动/270° 适配）

---


## 2026-08-21（外部审核甄别 + P0 安全修复）

### 本轮做了什么
1. **270° 舵机适配**：用户确认舵机为 270° 型（推翻 180° 假设）。main.h 加 `SERVO_MAX_ANGLE=270U` + `SERVO_HOME_PULSE` 宏推导；Servo_SetAngle 映射分母 180→SERVO_MAX_ANGLE；限位表保守保持 0-180 待阶段 6 实测。归中位置 = 上电默认 90°（1167µs），**非舵机机械中点 135°**
2. **外部 AI 审核甄别**（审的是副本目录）：确认 4 个真问题 + 否决"回归 USB CDC"建议（USB 已几十轮排查止损，走 CH340+USART3 正式路线）
3. **P0 修复——boot 零电流软启动**：
   - main() 删六路 HAL_TIM_PWM_Start；CDC_Init_FS 删隐蔽 Servo_EnableAll
   - 新增 servo_enabled[6] 标志，Servo_SetAngle 按需懒启动通道（比较值先写、estop 门控）
   - timeout 恢复只清标志不批量使能（M<n> 懒启动对应通道）；H 命令保留全恢复
   - **同时消除 boot 脉宽矛盾**（Pulse=1500 硬编码 = 135°物理 → SERVO_HOME_PULSE=1166，消除上电猛转 45°）
4. **P0 清理——USB CDC 双轨退役**：
   - CDC_Receive_FS 断开 rx_ring 喂入（USART3 唯一命令入口，单写者）
   - CDC_DeInit_FS 改纯 stub：USB 总线事件不再停舵机/清 ring（主机重启不能让抓着的物体掉落）
   - 删死变量 usb_disconnected/tx_busy + M 命令路径 usb_disconnected 检查（USB 弃用后会卡 1 拒绝所有 USART3 命令的真实故障源）
   - 删 UART_StartRx no-op（函数+调用+声明）
5. **P1 修复——消隙过冲俯冲**：trajectory.move_to/_move 加 overshoot 参数；_servo_align/DESCEND（视觉+旧路线）/FINAL_ALIGN 禁过冲（过冲点在目标下方 ≈ 数十 mm 会撞物）；SEARCH/HOME/APPROACH/LIFT/GRASP 保留消隙
6. **P2 修正**：calibration.py 重投影误差单位 px 标注修正；config.py 串口注释改 CH340/USART3；孤儿头 stm32f103xb_new.h/stm32f1xx_new.h 回收站删除

### 验证
- Keil 编译 0 Error（两次全量）✅
- py_compile trajectory/state_machine/config/calibration 通过 ✅
- grep 确认 usb_disconnected/tx_busy/UART_StartRx 零残留 ✅
- **未烧录**（用户指示先不烧录；ST-LINK 未连接）

### 当前状态
- ✅ 固件安全修复完成待烧录；上位机过冲修复完成待 DRY_RUN 复跑
- ⏳ 等 CH340 到货 → 烧录 → 串口联调（S/M1 90/I/H）→ 阶段 4 脉宽实测

---


## 2026-08-18（视觉伺服方案重构）

### 本轮做了什么
1. **架构决策（用户主导，已确认）**：放弃毫米级四点标定路线（畸变/固定误差/舵机回差累积不可控），改**视觉伺服方案**：
   - 摄像头装机械臂上（eye-in-hand），升最高俯瞰 → J1 转圈搜索（SEARCH）→ 像素域 PID 伺服收敛（ALIGN）→ 逐级下降+每级对齐（DESCEND）→ 红外确认 → 抓取
   - 全程像素域闭环，不依赖毫米标定；正方形 90° 对称 → 角度容错 ±45°，J5 = θ − J1
   - 粗定位（搜索）→ 精对准（伺服）→ 红外确认（抓取瞬间），与"距离模型"讨论：已知尺寸 3.5cm 可用单目测距公式替代，暂不训练
2. **vision.py 修复（关键 bug）**：detect() 原来读 `results[0].boxes`（OBB 模型输出在 `.obb`，boxes 为 None → **永远返回 None**）；改为读 `obb.xywhr`，返回 (cx, cy, theta_deg, conf)，theta 归一化 [0,90)
3. **config.py**：YOLO_MODEL 指向 pc\models\best.pt（原路径已删）；CAMERA_IDX 0→1；CONF 0.6→0.25（搜索宁低勿漏）；新增 SEARCH/ALIGN/DESCEND/FINAL_ALIGN 视觉伺服参数
4. **state_machine.py 重构**：新增 SEARCH/ALIGN/DESCEND/FINAL_ALIGN（视觉伺服）流程；旧毫米坐标路线保留兼容（无 vision/cap 时走旧路）；TRANSPORT 抬升改 FK+IK（水平不漂移）；DESCEND 每级 FK 当前 x,y + IK 降 z
5. **FINAL_ALIGN（用户设计，停-看-动）**：近距（木块占比 ≥25%）切停-看-动——停稳 1s → 拍 → 限幅小修正（≤2°）→ 停 → 再拍确认 → 红外 → 抓取；远距连续伺服（Kp 大、限幅 5°）
6. **main.py**：auto 模式改视觉伺服全流程（不再依赖 calibration）；test 模式带 vision+cap
7. **calibration.py 降级**：旧路线保留备用（视觉伺服不再调用）

### 验证
- vision.detect 真实图片：返回 (2402, 2434, 48.5°, conf 0.978) ✅
- DRY_RUN 全流程（FakeCap + 升最高起始位姿）：IDLE→SEARCH→ALIGN→DESCEND→FINAL_ALIGN→GRIP→TRANSPORT→RELEASE→HOME→IDLE，结果 True ✅
- FINAL_ALIGN 切换测试（占比 22% 图）：切换 + 确认在中心 ✅
- 旧路线兼容（run_grasp(x,y) 无 vision）：全流程 True ✅
- py_compile 全部通过 ✅

### 当前状态
- ✅ 视觉伺服方案代码就绪（待硬件实测调参：SERVO_KP 符号/大小、搜索转速、限位、FINAL_ALIGN 阈值）
- ⚠️ JOINT_HOME [90,0,0,...] 是最大半径 220mm 极限位姿（原设计遗留，真机需改）
- ⚠️ 模型对"木块占画面 >50%"输入不鲁棒（800px 放大测试未检出）——FINAL_ALIGN 阈值（25%）和 DESCEND_PX_TARGET（35%）真机需实测，可能需扩数据
- 待办: Phase D 硬件采购（电容 + 100Ω 未下单）；到货后按 `docs/拼装与测试指导.md` 执行
- 下一轮: 硬件到货 → 阶段 4 脉宽实测 → 伺服闭环调参

---

## 2026-08-17（拼装与测试指导文档）

### 本轮做了什么
1. **新增 `docs/拼装与测试指导.md`**（用户要求：硬件搭建过程中怎么分阶段拼装、拼到什么程度先测试、怎么测）：
   - 10 阶段 bring-up 计划：阶段 0 物料清点 → 1 电源系统（不接舵机）→ 2 MCU 最小系统+烧录 → 3 USB CDC 协议 → 4 单舵机+PWM → 5 全舵机+软启动+限位 → 6 **机械参数实测（分水岭）** → 7 红外 → 8 视觉标定 → 9 全流程联调 → 10 验收
   - 每阶段含：拼装内容 / 测试方法 / 通过标准 / 安全警告
   - 附快速排障表（抖动/枚举失败/舵机不动/红外不触发/IK 偏/抓不住）
   - 关键纪律：不跳步；阶段 6 是"能动的臂"→"能抓的臂"分水岭；实测值当天回填
2. **docs/README.md 文档地图**登记新文档（状态类）

### 当前状态
- ✅ 固件 v1.1 + 上位机 v0.2 + 文档体系（含拼装测试指导）
- 待办: Phase D 硬件采购（X16GF800 PTC 5只 ¥3.43 + 电容 + 100Ω + 可选急停按钮）；到货后按 `docs/拼装与测试指导.md` 执行
- 下一轮: Phase D 采购确认，或用户决定急停按钮

---

## 2026-08-17（目录合并：上位机移入固件工程）

### 本轮做了什么
1. **目录合并**：`C:\Users\diexie\Desktop\arm-grasp-pc\` → `C:\Users\diexie\Desktop\arm-grasp\pc\`（用户要求两个工程放一起）
   - 固件工程 `arm-grasp\`（Core/MDK-ARM/Drivers）+ 上位机 `arm-grasp\pc\`（9 个 py 模块）
   - 移动前检查：上位机代码无内部路径依赖（仅 config.py 的 YOLO_MODEL 绝对路径指向 yolo 数据库，不受影响）
2. **同步路径引用**：架构书 §6、UNIMPLEMENTED、SKILL.md（ch07 + 设计指导全书素材）全部 `arm-grasp-pc/` → `pc/`
3. **历史记录保留原样**：SESSION_LOG 内部旧记录中的 `arm-grasp-pc/` 路径是当时快照，未逐条改写

### 验证
- 移动后 `pc\main.py` 存在；py_compile 待跑（路径无依赖，理论不受影响）

### 当前状态
- ✅ 单目录结构：`arm-grasp\`（固件 + pc/ 上位机 + docs/ 文档）
- 待办: Phase D 硬件采购（X16GF800 PTC 5只 ¥3.43 + 电容 + 100Ω + 可选急停按钮）；Phase E 联调
- 下一轮: Phase D 采购确认，或用户决定急停按钮

---

## 2026-08-17（第三份外部 AI 审核稿甄别 + 采纳项落地）

### 本轮做了什么
1. **甄别第三份外部 AI 审核稿**（严格硬件工程师视角，审查路径 `arm-grasp - trae\`，0 Critical / 3 Required / 7 Consider）：
   - ✅ 采纳：#3 换行符（真问题）、#5/#8 F1 无 USB 断开中断（分析正确且重要）、#6 堆余量记录
   - ❌ 拒绝：#1（论证错误——总线复位时 `USBD_LL_Reset`→`USBD_DeInit`→`CDC_DeInit_FS`→`Servo_DisableAll` 链存在，通道确实停了；它没抓到真实场景"首次枚举双重启动"）、#7（可信主机假设）、#10（说反了——`volatile servo_angle` 跨 ISR/主循环共享，volatile **必需**，它混淆原子性与可见性）
   - ⚠️ 降级：#2（`_Static_assert` 在 ARMCC 5.06 C99 下编译不过）、#4（ST 模板惯例）、#9（LL 接口必需函数）
2. **采纳项落地（固件）**：
   - 10 个文件末尾补换行（6 .c + 4 .h，字节级追加 0x0A 不改变编码）→ 全工程 0 个缺换行文件
   - `usbd_conf.c` `HAL_PCD_DisconnectCallback` 加醒目注释：**F1 USB IP 无物理断开中断，此回调永不触发**；拔线实际走 3ms 无 SOF→SUSP，PWM 停止唯一兜底 = PC 端 10s 超时
   - `usbd_cdc_interface.c` 三处注释：`Servo_EnableAll` 双重启动说明（首次枚举异步先于 main() 的 PWM_Start，HAL_ERROR 无害；总线复位不会双重启动）、MALL 回显请求值而非 clamp 值（PC 端不得按实际解析，arm_serial 只解析 `OK M<n>`）、`usb_disconnected` 检查为防御性死代码
3. **采纳项落地（文档）**：诚实声明清单补 3 条——USB 断开检测不可靠（设计事实）、堆余量 ~480B（设计事实）、boot 时 6 舵机同时上电绕过软启动（设计矛盾）

### 验证
- 编译 0 Error（keil-mcp_build_project，ELF 生成）
- 换行警告消除（全工程扫描 0 个缺换行文件）

### 当前状态
- ✅ 固件 v1.1（可合入）+ 上位机 v0.2（oracle P1 清零）+ 文档体系归类完成 + 外部审核稿甄别完成
- 待办: Phase D 硬件采购（X16GF800 PTC 5只 ¥3.43 + 电容 + 100Ω + 可选急停按钮）；Phase E 联调（硬件到货 → 第一步实测 JOINT_OFFSET/连杆/限位）
- 下一轮: Phase D 采购确认，或用户决定急停按钮

---

## 2026-08-17（文档归类：决策/状态/知识 三层）

### 本轮做了什么
1. **文档按职能归类**（建立 docs/README.md 文档地图）：
   - 决策类：docs/architecture/1-架构设计.md
   - 状态类：docs/project/（SESSION_LOG / UNIMPLEMENTED / 诚实声明清单）
   - 知识类：docs/knowledge/KNOWN_TRAPS.md
   - 根目录只留 CLAUDE.md（入口）
2. **同步所有交叉引用**：CLAUDE.md（阅读顺序+铁律+自检）、架构书（阅读顺序+ADR-10+文末）、SKILL.md（文档列表+下一轮任务）

### 注意
- 本文档内部历史记录的路径引用是当时快照（如 "UNIMPLEMENTED.md"），未逐条改写——历史叙述保留原样；导航性引用（阅读顺序/地图）已全部更新为新路径。

### 当前状态
- ✅ 固件 v1.1（可合入）+ 上位机 v0.2（oracle P1 清零）+ 文档体系归类完成
- 待办: Phase D 硬件采购；Phase E 联调（硬件到货 → 第一步实测 JOINT_OFFSET/连杆/限位）

---

## 2026-08-17（oracle 审查 Phase C + P1 修复完成）

### 本轮做了什么
1. **oracle 审查上位机 9 模块**：结论 **NEEDS REVISION**（核心架构正确，3 P1 + 8 P2 + 9 P3）
2. **修复全部 3 个 P1**：
   - **P1-1（安全）**：ERROR 抬升点 (0,0,SAFE_Z) 实测不可达（J3 155.9°>150°，奇异折叠区）→ 改为**当前目标上方 SAFE_Z**（与 APPROACH 同构必可达），无目标时用 FK 当前 XY 兜底。已实证：旧路径 None / 新路径可达，恢复完整走通
   - **P1-2**：GRIP/RELEASE 预算余量仅 0.2-0.3s → 跳过零增量关节（命令数÷6）+ 预算 5000→8000 + SETTLE 500→300
   - **P1-3（安全）**：`--mode test` 不强制 dry-run 会真实动臂 → 强制护栏
3. **修复 P2 核心**：过冲限位感知 clamp（P2-2）、串口超时排空缓冲+回显索引校验（P2-3）、删 DESCEND 前置红外死代码（P2-1）
4. **修复 P3**：docstring、import time、删死代码（_rx_buf/CMD_INTERVAL/dry_run 属性）、PLACE_Z/RELEASE_WAIT_MS 入 config、vision 类别过滤、ir 去抖窗口语义、calib 跳过软启动、except pass→log.warning
5. **接受的技术债（记入诚实声明/未做）**：P2-6 状态超时不贯穿轨迹（命令级 2s 超时兜底已缓解）；P2-7 全流程 37s>15s 验收（settle 降 300ms + 零增量跳过已部分缓解，剩余留 Phase E 调参）；P2-4 软启动阶跃（真机观察）；P2-5 标定误差单位（px 当 mm，实测标定时统一）；P2-8 标定 assert 崩溃（交互标定改进）

### 验证
- py_compile 全过；DRY_RUN 全流程 True；P1-1 实证（旧 None/新可达/恢复走通）
- 固件本轮未改动

### 当前状态
- ✅ 固件 v1.1（可合入）+ 上位机 v0.2（oracle 审查 P1 清零）
- 待办: Phase D 硬件采购；Phase E 联调（硬件到货 → 第一步实测 JOINT_OFFSET/连杆/限位）
- 下一轮: 若用户要，可做 P2-6/7 深化或直接进 Phase D 采购

---

## 2026-08-17（Phase C 上位机实现 + DRY_RUN 通过）

### 本轮做了什么
1. **arm-grasp-pc 9 模块全部实现**（`C:\Users\diexie\Desktop\arm-grasp-pc\`）：
   config / kinematics / arm_serial / trajectory / calibration / vision / ir_sensor / state_machine / main
2. **自测结果**：
   - kinematics: 189 点 FK(IK) 闭环验证 **0 bug**，11 点预期拒绝（限位/几何）
   - calibration: 4 点标定闭环 **0.000mm**
   - `python main.py --dry-run --mode test`：**grasp result: True** 全流程通过
   - 固件限位表同步改动，编译通过（27 条换行警告无害，ELF 生成）
3. **发现并修复的关键问题**：
   - **架构级：关节角↔舵机角 offset**（KNOWN_TRAPS #16）——IK 解关节角，固件收舵机角。混合 offset：J2=90/J3=0/J4=90/J5=90。固件限位表同步改舵机角域
   - **IK 静默 clamp**（#17）——超限=不可达，绝不静默
   - **dry_echo 双重转换 bug**——模拟固件视角，收到的就是舵机角
   - **Windows sleep 粒度坑**（#18）——忙等补偿替代 sleep(0.02)
   - DRY_RUN 红外模拟——第 3 次查询"接触"，否则 DESCEND 走不完
   - 状态超时放宽（APPROACH/TRANSPORT/HOME 15s，DESCEND 30s）

### 改动清单
| 文件 | 改动 |
|------|------|
| arm-grasp-pc/config.py | JOINT_OFFSET、舵机角域限位、超时放宽 |
| arm-grasp-pc/kinematics.py | to_servo/from_servo、超限=不可达、自测 |
| arm-grasp-pc/arm_serial.py | offset 双向转换、dry_echo 修正 |
| arm-grasp-pc/trajectory.py | 忙等定时补偿 |
| arm-grasp-pc/ir_sensor.py | DRY_RUN 红外模拟 |
| arm-grasp-pc/calibration.py | save/load + 自测 |
| arm-grasp-pc/state_machine.py | 完整抓取流程 |
| arm-grasp-pc/main.py | 四模式主流程 |
| 固件 usbd_cdc_interface.c | 限位表→舵机角域 |
| UNIMPLEMENTED/诚实声明/KNOWN_TRAPS | 同步更新 |

### 当前状态
- 已完成: 固件 v1.1（可合入）；上位机 v0.1（DRY_RUN 通过）；Phase A+B+C 主体
- 待办: **oracle 审查 Phase C**；Phase D 硬件采购；Phase E 联调（硬件到货）
- 风险项: **JOINT_OFFSET/连杆参数/限位全是推测值**——到货后第一步实测，否则 IK 物理位置不对

---

## 2026-08-17（外部 AI 审核稿甄别 + 文档修正）

### 本轮做了什么
1. 收到两份外部 AI 审核稿（基于过时/不完整文档快照），逐条甄别：
   - **第一份（五文档审计）**：大量伪问题——不知固件已审查、引脚已定型、红外是 QT30CM 成品模块；编造"TIM1_CH4 重映射 PB13""红外 PB0/PB1""M0 命令""17 处技术错误""23 处矛盾""mAP=0.995"。拒绝。
   - **第二份（深度审核）**：4 条真知灼见，全部采纳（见下）
2. 采纳修正（P0 立即）：
   - 架构书 §9 Phase A/B 状态标 ✅ 完成（原文档不同步）
   - 保险丝 3A → **T6.3A 慢熔断**（3A 会误熔断，6 舵机同时运动 >3A）
   - 架构书 §6 新增**上位机回显铁律**（必须解析 M 回显 clamp 值更新 IK 状态，偏差 >5° 暂停）
   - 架构书 §5 补连杆参数表；§4 补 IK 无解回退策略
   - 架构书 §8 明确超时行为（PWM 停→松脱→臂下垂，恢复先 H）
3. UNIMPLEMENTED 重构为 P0/P1/P2 优先级结构，新增笛卡尔直线插补（下降段）、相机畸变两条
4. CLAUDE.md 加回显铁律；诚实声明补保险丝/红外输出类型

### 改动清单
| 文件 | 改动 | 原因 |
|------|------|------|
| docs/architecture/1-架构设计.md | Phase 状态、保险丝、回显铁律、参数表、IK 回退、超时定义 | 外部审计甄别采纳 |
| UNIMPLEMENTED.md | P0/P1/P2 重构 + 笛卡尔插补 + 畸变 | 采纳 |
| CLAUDE.md | 上位机回显铁律 | 采纳 |
| docs/诚实声明与技术债务清单.md | 保险丝/红外输出类型 | 采纳 |

### 当前状态
- 已完成: 固件 v1.1（可合入）；架构书 v1.1（含审计修正）；Phase A+B
- 待办: **Phase C 上位机**；Phase D 硬件安全件；Phase E 联调
- 风险项: 同前（红外/IK 参数待实测；硬件未到货）

---

## 2026-08-17（Phase B 固件审查完成，可合入）

### 本轮做了什么
1. **Phase B review-hw 正式审查**：发现 11 项（1 P0 / 2 P1 / 2 P2 / 6 P3）
   - **P0 致命**：堆 512B < CDC 结构体 540B → 枚举必然失败，COM 口不可用（编译验证发现不了）
   - P1：急停被 USB 重枚举绕过；USB 复位后 tx_busy 卡死
   - P2：10s 超时是空操作；malloc/free 在 ISR 上下文
2. **全部修复**：堆 1KB/栈 2KB、急停门控、tx_busy 复位 + dev_state 检查、超时真实停 PWM、usb_disconnected 拒绝 M、IWDG 提前、负角度 %d 解析
3. **review-hw 复核**：PASS WITH MINOR ISSUES，又抓 1 P1（超时+断开组合：重枚举后超时保护永久失效）+ 2 P3（H 未门控、负角度钳位方向），已修复
4. 编译 0 Error（三轮均验证）；RAM 余 13.3KB / 栈余 1.6KB / 堆余 484B
5. KNOWN_TRAPS 新增 5 条（#11-15）

### 改动清单
| 文件 | 改动 | 原因 |
|------|------|------|
| MDK-ARM/startup_stm32f103xb.s | 堆 0x200→0x400、栈 0x400→0x800 | P0 堆不足 + P3 栈偏紧 |
| Core/Src/usbd_cdc_interface.c | 急停门控、tx_busy 复位、dev_state 检查、超时真实停止、断开清缓冲、负角度拒绝 | P1×2 + P2 + P3 |
| Core/Src/main.c | IWDG 提前到 PWM 前 | P3 |
| docs/KNOWN_TRAPS.md | +5 条陷阱 | 审查发现归档 |

### 当前状态
- 已完成: 固件 v1.1（安全层 6 项 + 审查修复 14 项，编译 0 Error，**可合入**）；架构决策书；Phase A+B 完成
- 待办: **Phase C 上位机 arm-grasp-pc**（主战场）；Phase D 硬件安全件；Phase E 联调
- 风险项: 红外极性未实测；IK/限位参数为推测值（已入诚实声明清单）；硬件未到货

---

## 2026-08-17（Phase A 固件安全层完成）

### 本轮做了什么
1. 建立工程文档规范体系（架构决策书 v1.0 + CLAUDE.md + UNIMPLEMENTED + SESSION_LOG + KNOWN_TRAPS + 诚实声明清单）
2. **Phase A 固件补安全层 6 项全部完成，编译 0 Error**：
   - 关节限位钳位（joint_min/max 表，M 命令回显 clamp 后实际值）
   - 10s 无命令超时自动保持（CDC_TimeoutCheck，收到命令自动恢复）
   - USB 断开 → 停止全部 PWM（CDC_DeInit_FS → Servo_DisableAll；重枚举恢复）
   - IWDG 看门狗 ~2s（寄存器级，RM0008 §21.3；LSI 40kHz/128=312.5Hz，RLR=625）
   - `H` 命令（回中位 90° + 清急停 + 重新使能 PWM）
   - `E` 命令（急停：停止全部 PWM，拒绝 M 直到 H）
3. IWDG 用寄存器操作而非 HAL（工程是手写精简 HAL，无 iwdg 驱动文件；stm32f1xx.h 寄存器定义齐全）
4. 调试友好：`__HAL_DBGMCU_FREEZE_IWDG()`（断点暂停时不复位）

### 改动清单
| 文件 | 改动 | 原因 |
|------|------|------|
| Core/Src/main.c | IWDG 寄存器初始化 + 主循环喂狗 + CDC_TimeoutCheck + DBGMCU freeze | Phase A 安全层 |
| Core/Src/usbd_cdc_interface.c | 限位表 + clamp + 超时 + 断开/恢复 + H/E 命令 + Servo_DisableAll/EnableAll | Phase A 安全层 |
| Core/Inc/usbd_cdc_interface.h | CDC_TimeoutCheck 声明 | Phase A |
| docs/architecture/1-架构设计.md | 新增 | 架构决策书 v1.0 |
| CLAUDE.md / UNIMPLEMENTED.md / SESSION_LOG.md / docs/KNOWN_TRAPS.md / docs/诚实声明与技术债务清单.md | 新增 | 文档规范体系 |

### 当前状态
- 已完成: 固件 v1.1（安全层 6 项，编译 0 Error）；架构决策书
- 待办: **Phase B 固件审查（review-hw）**；Phase C 上位机；Phase D 硬件安全件；Phase E 联调
- 风险项: 红外极性未实测；IK/限位参数为推测值（已入诚实声明清单）；硬件未到货

---

## 2026-08-17（架构决策书建立）

### 本轮做了什么
1. 硬件采购定型：DC 母头线（5.5×2.1, 10A）+ 端子排 8P×2 + 10000μF 电容方案；红外改 QT30CM 计数模块（省 160Ω/10kΩ）；板子确认标准 Blue Pill（无 CH340）
2. 板子卖家资料包归档（PC13 验板例程 / SWD 接线图 / FlyMcu 串口下载）
3. 对照设计指导全书 + robot-arm-algorithms 通用算法库，识别固件缺 6 项安全功能（限位/超时/看门狗/H/E）
4. 建立项目文档规范体系（project-docs-norm）：架构决策书 + CLAUDE.md + UNIMPLEMENTED + SESSION_LOG + KNOWN_TRAPS + 诚实声明清单

### 改动清单
| 文件 | 改动 | 原因 |
|------|------|------|
| docs/architecture/1-架构设计.md | 新增 | 架构决策书（12 条 ADR + 5 阶段实施计划） |
| CLAUDE.md | 新增 | 项目铁律 + 阅读顺序 |
| UNIMPLEMENTED.md | 新增 | 半截功能清单（9 项） |
| docs/KNOWN_TRAPS.md | 新增 | 已知陷阱（初始） |
| docs/诚实声明与技术债务清单.md | 新增 | 推测值/未验证项 |

### 当前状态
- 已完成: 固件 v1.0（编译 0 Error 16.97KB）；知识库归档；架构决策书
- 待办: **Phase A 固件补安全层**（下一轮）；Phase B 固件审查；Phase C 上位机
- 风险项: 红外极性未实测；IK 参数理论值待实测标定；硬件未到货

---

## 2026-08-16（固件完成）

### 本轮做了什么
1. arm-grasp 固件全部文件写完并编译通过（0 Error，16.97KB）
2. 新版 ST USB 库 API 适配完成（usbd_core.c 自带 LL 回调，只实现底层 15 个函数）
3. 知识库 arm-grasp skill 建立（9 章 + glossary + 素材）

### 当前状态
- 固件 v1.0 编译通过，待补安全层（见 UNIMPLEMENTED）
## 2026-08-18（第二轮：丢失保护 + 甜区 + 异常检测）

### 本轮做了什么（用户设计确认）
1. **HOME = 最大甜区**：JOINT_HOME 从 [90,0,0,0,0,90]（最大半径 220mm 极限位姿）改为 [90,45,60,-15,0,90]（J2/J3 中间值，调节空间最大，FK r≈59mm z≈231mm）；抓取循环起点 = 终点 = 甜区
2. **丢失保护**：_servo_align 连续 LOST_FRAME_THRESHOLD(5) 帧未检出 → 返回 ("LOST", None) → run_grasp 回 SEARCH 重扫（最多 SEARCH_RETRY_MAX=3 次）
3. **中心点异常检测**：误差跳变检测（当前误差 > 上帧×ALIGN_JUMP_RATIO(3) + ALIGN_JUMP_PX(50)）→ 丢弃异常帧不动作（防误检污染伺服）
4. **SEARCH 往复扫描**：J1 行程 0-180° 不能真转圈，到限位反向（search_dir 翻转），不触发 CLAMP
5. **SEARCH 超时放宽**：60000 → 120000ms（24 步 × ~2.6s/步 ≈ 62s）

### 踩坑（已记 KNOWN_TRAPS #8）
- 大块重构缩进错位：FINAL_ALIGN 块写成 12 空格（attempt 循环外），导致 DESCEND 后重复 SEARCH 3 次才执行 FINAL_ALIGN；用 ast.parse 验证 For/else 匹配 + 状态序列断言修复
- 修复后状态序列：IDLE→SEARCH→ALIGN→DESCEND→FINAL_ALIGN→GRIP→TRANSPORT→RELEASE→HOME→IDLE 一次通过

### 验证（DRY_RUN + FakeCap）
- 正常场景（目标在中心）：全流程 True，状态序列一次通过 ✅
- 丢失场景（目标 3 帧后消失）：ALIGN/DESCEND 连续 5 帧丢失 → 回 SEARCH 重扫 → 24 步耗尽 → ERROR 安全回收 ✅
- 语法检查 + ast 结构验证 ✅

### 待办
- 训练扩数据（用户确认：1000 张近景远景集合，解决近距占比 >25% 时 conf 掉到 0.27 的问题）——等硬件到货用真实摄像头采集
- 真机调参：SERVO_KP 符号/大小、SEARCH 转速、FINAL_ALIGN 阈值与模型检测极限匹配
- 电容 + 100Ω 采购未下单

---

## 2026-08-18（第三轮：外部调研归档 + 夹爪决策）

### 做了什么
1. **外部调研**（@librarian ×3 并行：视觉伺服工程实践 / MG996R 精度 / YOLO-OBB 数据扩增），结论存档 → `docs/knowledge/外部调研-视觉伺服舵机精度数据扩增.md`
2. **方案被外部资料验证**：5°/步限幅（饱和函数防振荡）、纯 P 不加 I、settle 1s（look-and-move 慢视觉正确形态）、单向逼近消隙、逐级下降+每级对齐（规避像素→关节随深度失稳）——全部是正确的标准做法
3. **新决策（已归档）**：
   - 夹爪容差策略：开口 ≥45mm + 爪口倒角 + 硅胶/TPU 指垫（方块 35mm，单边 5mm+ 容差；MG996R 末端误差实测 ±8.7mm 是物理上限，容错设计在夹爪上）→ 已加入采购清单
   - 电流采样力检测（ADC 读舵机电流，夹住即停）→ 加入 UNIMPLEMENTED
   - 收敛死区 ≥2° 即判到位（<2° 修正被回差吞掉）→ 加入伺服调参验收标准；FINAL_ALIGN_MAX_STEP 可能需 2.5°
4. **数据采集方案参数确认**：1s 抽帧 + Laplacian≥30 过滤 + 伪标签 conf≥0.5 激进过滤 + 角度人工核对 + 近景占 500-600 张 + mosaic 0.3-0.5 + close_mosaic=50

### 待办（不依赖硬件）
- 采集脚本（录像→抽帧→模糊剔除→伪标签）等硬件到货后写
- 夹爪开口/倒角/指垫落实到实际夹爪制作

---

## 2026-08-18（第四轮：桌面文档归类 + 官方连杆参数回填）

### 做了什么
1. **桌面机械臂文档归类**（用户确认"热销6自由度异形机械手臂"为所购型号）：
   - 安装手册 PDF / 3D 模型 STEP / 官方示例 .ino → `docs/hardware/`
   - 机械臂知识库.txt / 历史对话导出 → `docs/knowledge/`
   - docs/README.md 文档地图新增"硬件类"登记
2. **连杆参数回填**：config.py L1/L2/L3 从推测值 [50/120/100] 改为官方参数 [72/105/128]（来源：官方示例 .ino，YFROBOT 6DOF 臂）
3. **JOINT_HOME 注释更新**：FK 从旧参数 r≈59mm z≈231mm 更新为官方参数下 r≈41mm z≈270mm
4. **诚实声明清单更新**：IK 参数行从"推测值"改为"官方参数（待实测复核）"

### 验证
- IK 闭环：可达域采样 1008 点，FK(IK) 误差 <1mm 全部通过；失败均为正常几何/限位拒绝（J1 半圆外、距离 >233mm、J2/J3 限位）
- HOME 反向 IK 回解正确：[90,45,60,-15,0,100]

### 踩坑复盘（本轮卡住教训）
- IK 验证采样网格覆盖了 J1 物理不可达象限（J1 限位 0-180° 只有半圆），失败率 570/968 大部分是采样问题而非代码问题；反复跑同一测试未及时修正方法 → 卡住。教训：**看到异常失败率先分析失败原因分布，再修正验证方法，不要重复跑同一命令**

### 待办
- 拼装后直尺实测复核 L1/L2/L3（官方参数 vs 实物）
- L4=170（J4 到夹爪尖）未配置进 IK（本项目 IK 只到 J4），夹爪实际位置比 J4 低，DESCEND 用红外确认兜底

---

## 2026-08-18（第五轮：官方资料细节提炼 + 文档归档 + 模拟软件调研）

### 做了什么
1. **重新精读官方资料三件套**（安装手册 PDF / 官方示例 .ino / 3D STEP 模型），提炼连杆长度之外的细节：
   - 舵机安装方向图（ASCII）+ 补角规则：官方默认 S3/S4 舵机角 = 180−关节角（与我们 config 的 JOINT_OFFSET 假设可能不同，拼装后实测）
   - PWM 映射官方确认：0°=500µs / 90°=1500µs / 180°=2500µs（需核对固件映射）
   - 官方 IK 坐标系命名（Y_EE=高度）+ 官方代码 bug（X_EE==0 分支 D 误用 Y_EE，不可直接照搬）
   - 官方逐个舵机 + 200ms 延时 = 验证我们软启动设计
   - 螺丝规格规则（M3*6 舵盘 / M3*8 顶部舵机支架 / M4*8 底部舵机）
   - 舵机归中方法（带限位/不带限位判断法，安装前先归中）
   - 10 步拼装流程（左右镜像对称是关键）
   - STEP 模型：SolidWorks 2014 AP203，可后续 CAD 量尺寸复核 L1/L2/L3
2. **文档归档**：
   - 新建 `docs/hardware/官方资料要点.md`（细节提炼 + 落地清单）
   - `docs/拼装与测试指导.md` 新增"阶段 0.5：官方资料速查"（螺丝规格 + 归中方法 + 拼装顺序）
   - `docs/README.md` 文档地图登记
3. **模拟软件调研**（@librarian）：结论 = **PyBullet 首选**（免费 Zlib/MIT、纯 Python、URDF 直载、getCameraImage 渲染喂 YOLO、内置 IK、CPU 即可跑）；MuJoCo 次选（接触物理最准）；Isaac Sim 需 RTX 4080+ 不适用；Gazebo Windows 实验性。用 PyBullet 可完整预演 SEARCH→ALIGN→下降→抓取，YOLO 代码零改动接入。**sim-to-real 提醒**：MG996R 开环无编码器，模拟验证算法逻辑而非精确物理。已归档 docs/knowledge/外部调研-机械臂模拟软件选型.md

### 待办
- 核对固件 PWM 映射（500-2500µs）
- 拼装后实测 JOINT_OFFSET（S3/S4 补角方向）与 L1/L2/L3
- 后续可做：PyBullet 抓取流程仿真（URDF 可按官方 L1/L2/L3/L4 手写，或 FreeCAD 从 STEP 导出）

---

## 2026-08-23（舵机终于动了：HAL 回调函数名不匹配 + pyod IWDG 干扰）

### 本轮做了什么
1. **硬件组装**：6V 8A 电源 + PTC + 两级 LC+共模滤波模块 + 10000µF 大电容（螺丝端子免焊）+ 舵机接线
2. **舵机调试马拉松**：bit-bang 测试固件能扫动、生产固件 M 命令回 OK 但舵机不动 → 信号线问题 + HAL 回调 bug 双重叠加
3. **P0 根因——HAL_TIM_PWM_MspInit 缺失**：`HAL_TIM_PWM_Init()` 调用 `HAL_TIM_PWM_MspInit()`（__weak 空默认），项目只定义了 `HAL_TIM_Base_MspInit()`（CubeMX 生成），函数名不匹配 → TIM1/TIM2 时钟 never enabled → 寄存器全零 → PA8/PA0 无 PWM 输出。修复：在 stm32f1xx_hal_msp.c 添加 `HAL_TIM_PWM_MspInit` 覆盖
4. **pyod IWDG 干扰**：pyod 连接 SWD 时触发软复位，不清 IWDG → 循环复位 → 寄存器读零。解决方案：代码禁用 IWDG + 断电清狗后恢复
5. **测试固件归档**：`tools/test_firmware_gpio_servo/`（恒高版 + 递增扫幅版），含 README 使用说明
6. **pyod 烧录通道建立**：Keil ST-LINK DLL 克隆不兼容，改用 pyocd flash（`--frequency 100000`）
7. **KNOWN_TRAPS #24 已记录**

### 关键发现
- bit-bang GPIO 能驱动舵机 ≠ TIM PWM 能驱动舵机——HAL 回调链路是独立的，缺一个函数整个外设就是死的
- pyod/ST-Link 连接本身会干扰目标（软复位不清 IWDG），调试时必须考虑
- 舵机"偶尔咔哒 + 手动弹回原位" = 数字舵机失联保持模式，不代表信号正常

### 当前状态
- 舵机1（PA8/TIM1_CH1）M1 命令可驱动 ✅
- 生产固件已修复并烧录（HSI 72MHz + IWDG 恢复 + PWM MspInit 修复）
- 待测：其余 5 轴、联合运动、机械臂拼装

### 待办
- 逐轴测试 M2-M6
- 拼装机械臂 + JOINT_OFFSET 实测
- PC 工具链联调（config.py COM8 已更新）

