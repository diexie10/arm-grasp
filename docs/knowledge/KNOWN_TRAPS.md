# KNOWN_TRAPS — 已知陷阱

> 每次审计/调试发现的 bug 抽象成"反模式"写这里。新增一条陷阱 = 删掉一个同类 bug。

## 1. 新版 USB 库重复定义 LL 回调
**错误表现**: 编译报重复定义 / 链接错误
**原因**: stm32-mw-usb-device master 版 usbd_core.c 自带 SetupStage/DataOutStage/DataInStage/Reset/SetSpeed/Suspend/Resume/SOF/IsoIN/IsoOUT/DevConnected/DevDisconnected 实现
**正确做法**: usbd_conf.c 只实现 Init/DeInit/Start/Stop/OpenEP/CloseEP/FlushEP/StallEP/ClearStallEP/SetUSBAddress/Transmit/PrepareReceive/IsStallEP/GetRxDataSize/Delay
**发现于**: 2026-08-16 固件编译

## 2. USBD_Init 第三参数
**错误表现**: 传 DEVICE_FS 编译不过
**原因**: 新版库第三参是设备 ID，单设备传 0U
**正确做法**: `USBD_Init(pdev, pdesc, 0U)`
**发现于**: 2026-08-16

## 3. HAL_PCD_EP_SetStall 命名
**错误表现**: 用 HAL_PCD_EP_Stall 编译不过
**原因**: F1 HAL 库函数名是 SetStall；且无 EP_IsStall 函数
**正确做法**: `HAL_PCD_EP_SetStall`；IsStall 读 `hpcd->IN_ep[epnum].is_stall`
**发现于**: 2026-08-16

## 4. PA11/PA12 分给 PWM
**错误表现**: USB 失效 / 枚举失败
**原因**: TIM1_CH4 默认映射 PA11，与 USB DM 冲突
**正确做法**: 舵机4 用 TIM2_CH1(PA0)，PA11/PA12 永远留给 USB
**发现于**: 设计阶段（手册 §十五 AI 常犯错误）

## 5. 忘记共地
**错误表现**: 舵机抖动/不响应，PWM 参考电平漂移
**正确做法**: 6V 电源 GND 与 USB GND 必须相连
**发现于**: 手册 §十五

## 6. 舵机电源接 STM32 5V/3.3V 引脚
**错误表现**: 烧板 / 电压拉低复位
**正确做法**: 舵机只走 6V 8A 独立电源，STM32 由 USB 供电
**发现于**: 手册 §十五

## 7. 中文路径 cv2.imread 失败
**错误表现**: 读图返回 None
**原因**: OpenCV imread 不支持中文路径
**正确做法**: `cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)`
**发现于**: YOLO 训练链路

## 8. 3 点仿射标定精度不足
**错误表现**: 标定后边缘偏差大
**正确做法**: 4 点透视 `cv2.getPerspectiveTransform`（单应矩阵），不用 3 点仿射
**发现于**: 手册 §八

## 9. 串口 readline 永久阻塞
**错误表现**: 上位机卡死
**正确做法**: pyserial 必须设 timeout=0.1；接收线程非阻塞读入 deque
**发现于**: 手册 §四

## 10. YOLO 首次推理慢 1.9s
**错误表现**: 首帧卡顿
**原因**: CUDA kernel 首次编译
**正确做法**: 启动预热 1 帧
**发现于**: 手册 §八

## 11. 堆不足 → USB 枚举失败，COM 口不可用（P0）
**错误表现**: 编译 0 Error 但插上电脑没有 COM 口
**原因**: `USBD_CDC_Init` 要 malloc 540B（USBD_CDC_HandleTypeDef），启动文件堆只有 512B → malloc 返回 NULL → STALL SET_CONFIGURATION
**正确做法**: 堆 ≥1KB（startup_stm32f103xb.s `Heap_Size EQU 0x400`）；栈 2KB（`0x800`）
**发现于**: 2026-08-17 review-hw 审查（编译验证发现不了的运行时问题）

## 12. 急停被 USB 重枚举绕过
**错误表现**: 按 E 急停后，拔插 USB/重开 COM 口，臂突然重新上电
**原因**: CDC_Init_FS 无条件 Servo_EnableAll，不检查 estop_active
**正确做法**: `if ((estop_active == 0U) && (timeout_active == 0U)) Servo_EnableAll();`
**发现于**: 2026-08-17 review-hw 审查

## 13. USB 复位后 tx_busy 永久卡死，固件失聪
**错误表现**: 重枚举后所有命令无回显（命令仍执行）
**原因**: 复位瞬间在途 TX 的完成回调永不触发，tx_busy 卡 1
**正确做法**: CDC_Init_FS/DeInit_FS 复位 tx_busy；发送前检查 dev_state == USBD_STATE_CONFIGURED
**发现于**: 2026-08-17 review-hw 审查

## 14. 超时保护被重枚举永久禁用
**错误表现**: 超时卸力后拔插 USB，臂重新上电且不再超时卸力
**原因**: CDC_Init_FS 重上电但不清 timeout_active，CDC_TimeoutCheck 守卫 `(timeout_active == 0U)` 永远为假
**正确做法**: Init 条件加 `&& (timeout_active == 0U)`，等第一条真实命令再上电
**发现于**: 2026-08-17 review-hw 复核

## 15. `%hu` 解析负角度成超大值
**错误表现**: `M1 -5` 被解析成 65531，钳到上限 170° 而非下限
**原因**: `%hu` 接受符号，-5 → 65531
**正确做法**: 用 `%d` 解析 int，`ib < 0` 回 ERR
**发现于**: 2026-08-17 review-hw 复核

## 16. 关节角 ≠ 舵机角（offset 缺失 = IK 全错）
**错误表现**: IK 解出的角度发到固件后，机械位置完全不对
**原因**: IK 解出的是关节角（J2 相对水平），固件 M 命令角度直接转 PWM 是舵机角。J2 舵机 90°=大臂水平 → 关节角 = 舵机角 − 90。J3 舵机角=关节角（offset 0，小臂可折回 150°）。混合 offset！
**正确做法**: config.JOINT_OFFSET = [0, 90, 0, 90, 90, 0]；发送关节角→舵机角，回显舵机角→关节角；固件限位是舵机角域
**发现于**: 2026-08-17 Phase C 自测（FK(IK) 闭环验证暴露）

## 17. IK 静默 clamp → 虚拟角度与物理脱节
**错误表现**: FK(IK) 闭环验证误差几百 mm
**原因**: ik_solve 解出超限角度后静默 clamp，FK 用 clamp 后的值验证必然错位
**正确做法**: IK 超限 = 不可达（返回 None + reason），绝不静默 clamp。clamp 是固件兜底，上位机必须保证解在限位内
**发现于**: 2026-08-17 Phase C 自测

## 18. Windows time.sleep 粒度坑（20ms 实际 31ms）
**错误表现**: 轨迹每点 sleep(0.02)，258 点实际耗时 8.58s（理论 5.16s），长轨迹累计超时
**原因**: Windows 系统时钟粒度 ~15.6ms，sleep(0.02) 向上取整到 ~31ms
**正确做法**: 目标时间戳 + 忙等补偿（perf_counter 循环），保证插补周期精确
**发现于**: 2026-08-17 Phase C DRY_RUN（APPROACH timeout）
## 8. 大块代码重构时缩进层级错位（FINAL_ALIGN 掉出 attempt 循环）
**现象**: DESCEND 完成后不进入 FINAL_ALIGN，而是重复 SEARCH 3 次才继续；或 continue/break 报 "not properly in loop"
**原因**: 用 edit 工具替换大块代码时，新代码缩进与所在循环层级不匹配（FINAL_ALIGN 块写成 12 空格，实际应在 attempt 循环内 16 空格），Python 不报错但语义完全改变
**正确做法**: 大块重构后用 `ast.parse` 验证 For/If/else 的匹配关系 + 状态序列日志断言（期望 IDLE→SEARCH→ALIGN→DESCEND→FINAL_ALIGN→GRIP→TRANSPORT→RELEASE→HOME→IDLE 一次通过，无重复 SEARCH）
**发现日期**: 2026-08-18

## 9. J1 行程 0-180° 不能真"转圈"搜索
**现象**: SEARCH 单向累加 J1 角度，到 180° 触发 CLAMP 报错（joint 1 requested 185.0, echoed 180.0）
**原因**: 机械限位 0-180°，"转圈扫描"实际是往复扫描
**正确做法**: SEARCH 循环内检测即将超限 → 反向步长（search_dir 翻转），不依赖 move_to 的 CLAMP 兜底
**发现日期**: 2026-08-18
