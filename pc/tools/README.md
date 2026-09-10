# pc/tools/ — 快速联调诊断脚本

## 通用参数

| 参数 | 说明 |
|------|------|
| `--dry` | 不连接硬件，使用 dry_run 模式（假回显） |
| `--port COMn` | 指定串口（默认 `config.COM_PORT`） |

## 脚本

| 脚本 | 功能 | 典型用法 |
|------|------|----------|
| `t_link.py` | 信号通路测试：握手 + S/I/Q + 校验和拒绝 | `python pc/tools/t_link.py --dry` |
| `t_center.py` | 舵机归中：移动到 HOME 并核对 ±1° | `python pc/tools/t_center.py --dry` |
| `t_move.py` | 运动轨迹测试：G/DONE + 时间估算对比 | `python pc/tools/t_move.py --dry --joint 1` |
| `t_ir.py` | 红外信号：实时轮询读数 + 跳变统计 | `python pc/tools/t_ir.py --dry --seconds 3` |
| `t_limits.py` | 限位核对：固件 L 命令 vs PC config | `python pc/tools/t_limits.py --dry` |

## 安全提示

- **真实模式**会独占 COM 串口 → 先关闭浏览器 Web-Serial 控制台
- `--dry` 模式无硬件风险，适合 CI/本地验证
- `t_move.py` 真实模式会移动关节 → 确保臂周围无障碍物
