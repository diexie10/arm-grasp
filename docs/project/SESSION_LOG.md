# SESSION LOG — 会话日志

> 顶部追加最新记录。下次继续工作时优先读这里。
> 本文档位于 docs/project/（状态类）。文档地图见 docs/README.md。

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

