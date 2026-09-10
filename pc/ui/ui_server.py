# -*- coding: utf-8 -*-
"""ui_server.py — arm-grasp 机械臂调试台本地服务器。

纯标准库（http.server）+ pyserial，复用 arm_serial.ArmSerial 协议层
（校验和 / 回显铁律 / 限位预检全部继承，不重写协议）。

启动:  python ui_server.py        → http://127.0.0.1:8765
页面:  同目录 index.html（单文件，无外部依赖，离线可用）

API:
    GET  /                调试台页面
    GET  /api/config      关节限位/HOME/连杆参数（config.py 单一来源）
    GET  /api/ports       可用串口列表
    GET  /api/status      遥测（S+I+Q 查询：关节角/舵机角/红外/忙闲/急停）
    POST /api/connect     {port, dry}   dry=true 为 DRY_RUN 模拟（不发串口）
    POST /api/disconnect  断开
    POST /api/estop       E 急停
    POST /api/home        H 回中（清急停，PWM 保持关闭）
    POST /api/softstart   软启动（逐轴 200ms 回 HOME）
    POST /api/joint       {n, angle}    关节域单轴（走 move_joint，回显铁律）
    POST /api/waypoint    {q[6]}        G 批量目标（MCU 梯形 + Q 握手到 DONE）
    POST /api/cmd         {cmd}         原始命令（自动加 *XX 校验，串口台架用）

安全:
    只绑 127.0.0.1（调试工具不上局域网）；全部串口操作经全局锁串行化；
    E 急停不受任何互斥影响（锁获取后立即发送）。
"""

import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config                                   # noqa: E402
from arm_serial import ArmSerial                 # noqa: E402

HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html")
BIND_HOST = "127.0.0.1"
BIND_PORT = 8765

_lock = threading.Lock()        # 串口操作全局锁（浏览器并发请求串行化）
_arm = None                     # ArmSerial 实例（None = 未连接）
_arm_err = None                 # 最近一次串口故障信息


def _drop(reason):
    """串口故障后强制下线（下次轮询提示重连）。"""
    global _arm, _arm_err
    try:
        if _arm is not None:
            _arm.close()
    except Exception:
        pass
    _arm = None
    _arm_err = reason


def _sync_m_echo(cmd, resp):
    """原始 M 命令的回显同步（DRY_RUN 下 S 轮询也能看到移动；真机无害）。"""
    import re
    m = re.match(r"^M(\d)\s+(-?\d+)", cmd.strip())
    e = re.match(r"^OK M(\d)\s+(-?[\d.]+)$", resp or "")
    if m and e and int(m.group(1)) == int(e.group(1)):
        idx = int(e.group(1)) - 1
        _arm.joint_state[idx] = float(e.group(2)) - config.JOINT_OFFSET[idx]


# ---------------------------------------------------------------- handlers

def api_config():
    return {
        "joint_min": config.JOINT_MIN,
        "joint_max": config.JOINT_MAX,
        "joint_offset": config.JOINT_OFFSET,
        "joint_home": config.JOINT_HOME,
        "grip_open": config.GRIP_OPEN,
        "grip_close": config.GRIP_CLOSE,
        "l1": config.L1, "l2": config.L2, "l3": config.L3, "l4": config.L4,
        "default_port": config.COM_PORT,
        "baudrate": config.BAUDRATE,
        "softstart_interval": config.SOFTSTART_INTERVAL,
        "link_zero_deg": config.LINK_ZERO_DEG,
        "dial_sign": config.DIAL_SIGN,
        "axis_names": ["J1 底座", "J2 大臂", "J3 小臂", "J4 腕俯仰",
                       "J5 腕旋转", "J6 夹爪"],
    }


def api_ports():
    try:
        from serial.tools import list_ports
        return [{"port": p.device, "desc": (p.description or "")}
                for p in list_ports.comports()]
    except Exception as e:
        return [{"port": "", "desc": "枚举失败: %s" % e}]


def api_status():
    with _lock:
        if _arm is None:
            out = {"connected": False}
            if _arm_err:
                out["error"] = _arm_err
            return out
        try:
            ok_s = _arm.query_state()
            ir1, ir2 = _arm.query_ir()
            q_resp = _arm._send("Q")
        except Exception as e:
            _drop(str(e))
            return {"connected": False, "error": "串口故障: %s" % e}
        return {
            "connected": True,
            "telemetry_ok": bool(ok_s),
            "joints": [round(v, 1) for v in _arm.joint_state],
            "servos": [round(_arm.joint_state[i] + config.JOINT_OFFSET[i], 1)
                       for i in range(6)],
            "ir1_blocked": ir1,
            "ir2_blocked": ir2,
            "busy": bool(q_resp and "BUSY" in q_resp),
            "estop": bool(_arm.estop_active),
            "warning": _arm.last_warning,
            "dry_run": bool(_arm.dry_run),
        }


def api_connect(body):
    global _arm, _arm_err
    port = (body.get("port") or "").strip()
    dry = bool(body.get("dry"))
    if not dry and not port:
        return {"ok": False, "error": "未选择串口"}
    with _lock:
        if _arm is not None:
            try:
                _arm.close()
            except Exception:
                pass
            _arm = None
        try:
            _arm = ArmSerial(port or None, dry_run=dry)
            _arm_err = None
        except (SystemExit, Exception) as e:
            _arm = None
            _arm_err = str(e)
            return {"ok": False, "error": "打开串口失败: %s" % e}
        return {"ok": True, "dry_run": dry}


def api_disconnect():
    global _arm, _arm_err
    with _lock:
        if _arm is not None:
            try:
                _arm.close()
            except Exception:
                pass
        _arm = None
        _arm_err = None
    return {"ok": True}


def api_estop():
    with _lock:
        if _arm is None:
            return {"ok": False, "error": "未连接"}
        try:
            ok = _arm.estop()
        except Exception as e:
            _drop(str(e))
            return {"ok": False, "error": str(e)}
        return {"ok": bool(ok)}


def api_home():
    with _lock:
        if _arm is None:
            return {"ok": False, "error": "未连接"}
        try:
            ok = _arm.home()
        except Exception as e:
            _drop(str(e))
            return {"ok": False, "error": str(e)}
        return {"ok": bool(ok)}


def api_softstart():
    with _lock:
        if _arm is None:
            return {"ok": False, "error": "未连接"}
        try:
            ok, msg = _arm.soft_start()
        except Exception as e:
            _drop(str(e))
            return {"ok": False, "error": str(e)}
        return {"ok": bool(ok), "msg": msg}


def api_joint(body):
    n = int(body.get("n", 0))
    angle = float(body.get("angle", 0))
    with _lock:
        if _arm is None:
            return {"ok": False, "msg": "未连接"}
        try:
            ok, msg = _arm.move_joint(n, angle)
            joint = _arm.joint_state[n - 1] if 1 <= n <= 6 else None
        except Exception as e:
            _drop(str(e))
            return {"ok": False, "msg": "串口故障: %s" % e}
    return {"ok": bool(ok), "msg": msg, "joint": joint}


def api_waypoint(body):
    q = [float(v) for v in body.get("q", [])]
    with _lock:
        if _arm is None:
            return {"ok": False, "msg": "未连接"}
        try:
            ok, msg = _arm.move_waypoint(q)
        except Exception as e:
            _drop(str(e))
            return {"ok": False, "msg": "串口故障: %s" % e}
    return {"ok": bool(ok), "msg": msg}


def api_cmd(body):
    cmd = (body.get("cmd") or "").strip()
    if not cmd:
        return {"ok": False, "resp": "空命令"}
    with _lock:
        if _arm is None:
            return {"ok": False, "resp": "未连接"}
        try:
            resp = _arm._send(cmd)
        except Exception as e:
            _drop(str(e))
            return {"ok": False, "resp": "串口故障: %s" % e}
    if resp:
        _sync_m_echo(cmd, resp)
    return {"ok": resp is not None, "resp": resp or "TIMEOUT"}


# ---------------------------------------------------------------- http

class Handler(BaseHTTPRequestHandler):
    def _json(self, obj, code=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _post_body(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/link_annotator"):
            html = ("index.html" if path == "/"
                    else "link_annotator.html")  # 三连杆标注台（独立工具）
            try:
                with open(os.path.join(os.path.dirname(HTML_PATH), html), "rb") as f:
                    data = f.read()
            except OSError:
                self._json({"error": html + " 缺失"}, 500)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif path == "/api/config":
            self._json(api_config())
        elif path == "/api/ports":
            self._json(api_ports())
        elif path == "/api/status":
            self._json(api_status())
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        path = self.path.split("?")[0]
        try:
            body = self._post_body()
        except (ValueError, json.JSONDecodeError):
            self._json({"error": "bad json"}, 400)
            return
        routes = {
            "/api/connect": api_connect,
            "/api/disconnect": lambda b: api_disconnect(),
            "/api/estop": lambda b: api_estop(),
            "/api/home": lambda b: api_home(),
            "/api/softstart": lambda b: api_softstart(),
            "/api/joint": api_joint,
            "/api/waypoint": api_waypoint,
            "/api/cmd": api_cmd,
        }
        fn = routes.get(path)
        if fn is None:
            self._json({"error": "not found"}, 404)
            return
        try:
            self._json(fn(body))
        except (ValueError, TypeError) as e:
            self._json({"ok": False, "error": "参数错误: %s" % e}, 400)

    def log_message(self, fmt, *args):
        pass  # 静默访问日志（调试台轮询频繁）


def main():
    srv = ThreadingHTTPServer((BIND_HOST, BIND_PORT), Handler)
    print("arm-grasp 调试台: http://%s:%d  (Ctrl+C 退出)" % (BIND_HOST, BIND_PORT))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        api_disconnect()
        srv.server_close()


if __name__ == "__main__":
    main()
