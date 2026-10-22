#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test-hardware.py — 交互式硬件测试台（ROV 下位机，走 422 串口）

在一台电脑上通过 422 链路测试：陀螺仪 / 磁力计 / 水深传感器 / 推进器 / 舵机。

    ./test-hardware.py                       # 自动找串口, 115200
    ./test-hardware.py --port /dev/ttyUSB0
    ./test-hardware.py --no-safe             # 退出时不下发停止指令(调试用)

工作原理
--------
  · 传感器：靠固件新增的遥测命令 "OUT:<what>" 把数据周期性回传（文本行）。
            进入某个界面时本程序会切换到对应的数据流。
  · 执行器：靠固件已有的命令直接给 PWM
              TES:<8个PWM>   推进器（PWM 扩展板通道 0-7）
              MOT:<4个PWM>   舵机  （PWM 扩展板通道 8-10/11）
  · 全程只用到 422 串口，不需要 DAP-Link / OpenOCD。

接线（与 Making_Instructions/README.md 一致）
------------------------------------
  C 板 IIC  → I2C 扩展板(TCA9548A, 地址 0x70)
                 通道 0-3 → 4 个 MS5837 水深传感器
                 通道 4   → PWM 扩展板(PCA9685, 0x80, 50 Hz)
   PWM 扩展板 通道 0-7 → 8 个推进器
              通道 8-10(11) → 舵机
  C 板 UART2（丝印！其实是 STM32 USART1）→ TTL 转 422 → 422 转 USB → 电脑

安全
----
  · 推进器/舵机默认停在中位（推进器 1610，舵机 1500）。
  · 离开中位 100 以上会要求二次确认；范围外直接拒绝。
  · 任何界面按 q / Ctrl-C 返回，退出时自动下发停止指令（--no-safe 关闭）。
"""
from __future__ import annotations

import argparse
import glob
import os
import select
import sys
import termios
import threading
import time
from collections import deque

# --------------------------------------------------------------------------
# 固件里的常量（保持与 userCode 一致）
# --------------------------------------------------------------------------
INIT_PWM = 1610                    # Propeller.cpp: InitPWM（V33 中位，停转）
PROP_MIN, PROP_MAX = 1000, 2000    # Propeller.cpp: Handle() 的 clamp 范围
PROP_CH = 8                        # PWM 扩展板通道 0-7
SERVO_CENTER = 1500                # Servo.cpp: 上电中位
SERVO_MIN, SERVO_MAX = 500, 2500   # 500-2500us 对应 0-180 度
SERVO_BOARD_CH = [8, 9, 10, 11]    # Servo.cpp: ID_V33 = {8,9,10,11}
SERVO_INVERT_IDX = 2               # Servo.cpp: data[2] = 3000 - data[2]（10 号通道反向）

TELEMETRY_MODES = ["PS", "RPY", "GYRO", "ACC", "MAG", "TEMP", "ALL", "OFF"]
TELEM_KEYS = ["PS", "RPY", "GYRO", "ACC", "MAG", "TEMP"]   # 遥测行的名字

# 遥测速率的量级（用于界面提示）：OUT:ALL 在固件里是 10 Hz，其余约 100 Hz
MODE_RATE = {"PS": "~100 Hz", "RPY": "~100 Hz", "GYRO": "~100 Hz",
             "ACC": "~100 Hz", "MAG": "~100 Hz", "TEMP": "~100 Hz",
             "ALL": "~10 Hz", "OFF": "-"}

GREEN, RED, YEL, BLUE, DIM, RST = (
    "\033[32m", "\033[31m", "\033[33m", "\033[36m", "\033[2m", "\033[0m")


def ok(s):   return f"{GREEN}{s}{RST}"
def bad(s):  return f"{RED}{s}{RST}"
def warn(s): return f"{YEL}{s}{RST}"
def info(s): return f"{BLUE}{s}{RST}"
def dim(s):  return f"{DIM}{s}{RST}"


def fmt(v, width=9, prec=4):
    """固定宽度格式化一个浮点数，方便列对齐。"""
    s = f"{v:.{prec}f}"
    return s.rjust(width)


# --------------------------------------------------------------------------
# 422 链路 + 遥测解析
# --------------------------------------------------------------------------
class Link:
    """串口封装：后台线程按行解析固件的遥测文本流。"""

    ACK_TIMEOUT = 1.5

    def __init__(self, port: str, baud: int = 115200):
        self.port, self.baud = port, baud
        try:
            import serial
        except ImportError:
            sys.exit("缺少 pyserial：请先安装（pip install pyserial）")
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:
            sys.exit(f"{bad('FAIL')} 打不开串口 {port}: {e}")
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self._lock = threading.Lock()
        self.latest: dict[str, tuple[float, list[float]]] = {}
        self.counts: dict[str, int] = {}
        self._stamps: dict[str, deque] = {}
        self.log = deque(maxlen=400)          # 原始行，供"原始命令"界面看
        self.acks = deque(maxlen=50)
        self.errors = 0
        self._ackev = threading.Event()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    # ---------------------------------------------------------------- 底层
    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(4096)
            except Exception:
                break
            if not data:
                continue
            buf += data
            if b"\n" not in buf and len(buf) > 256:
                buf = b""                       # 没换行的垃圾，丢掉重新同步
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line.rstrip(b"\r"))

    def _handle_line(self, raw: bytes):
        now = time.time()
        text = raw.decode("utf-8", "replace").strip()
        with self._lock:
            self.log.append((now, text))
        if not text:
            return
        if text.startswith("OUT:"):
            with self._lock:
                self.acks.append(text[4:])
            self._ackev.set()
            return
        if text[0] not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
            self.errors += 1
            return
        parts = text.split(",")
        name = parts[0]
        try:
            vals = [float(x) for x in parts[1:]]
        except ValueError:
            self.errors += 1
            return
        with self._lock:
            self.latest[name] = (now, vals)
            self.counts[name] = self.counts.get(name, 0) + 1
            self._stamps.setdefault(name, deque(maxlen=300)).append(now)

    # ---------------------------------------------------------------- 发送
    def send(self, cmd: str, gap: float = 0.03):
        """发一条命令（自动补 '\\n' 并留出空闲间隔，让固件按帧收）。"""
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.ser.write(cmd.encode())
        self.ser.flush()
        time.sleep(gap)

    # ---------------------------------------------------------------- 遥测
    def mode(self, what: str, wait_ack: bool = True):
        """切换遥测内容：OUT:<what>。返回固件回执里的模式名，超时返回 None。"""
        self._ackev.clear()
        self.send(f"OUT:{what}")
        if not wait_ack:
            return what
        if self._ackev.wait(self.ACK_TIMEOUT):
            with self._lock:
                return self.acks[-1]
        return None

    def query_mode(self):
        self._ackev.clear()
        self.send("OUT")
        if self._ackev.wait(self.ACK_TIMEOUT):
            with self._lock:
                return self.acks[-1]
        return None

    def get(self, name: str):
        """返回 (数据年龄秒, 值列表)，没有数据则 (None, None)。"""
        with self._lock:
            item = self.latest.get(name)
        if not item:
            return None, None
        return time.time() - item[0], item[1]

    def newest_age(self):
        """最近一条遥测的年龄（OUT:ALL 时没有名为 ALL 的行，所以不能按行名查）。"""
        with self._lock:
            if not self.latest:
                return None
            return time.time() - max(t for t, _ in self.latest.values())

    def rate_for(self, mode: str):
        """当前模式下每秒的遥测帧数（OUT:ALL = 6 种行之和）。"""
        keys = TELEM_KEYS if mode == "ALL" else [mode]
        return sum(self.rate(k) for k in keys)

    def rate(self, name: str):
        """最近 1 秒内该数据行的到达速率（Hz）。"""
        with self._lock:
            st = self._stamps.get(name)
            n = sum(1 for t in st if t > time.time() - 1.0) if st else 0
        return n

    # ---------------------------------------------------------------- 执行器
    def propeller_stop(self):
        """退出浮潜(PID)模式并把 8 路 PWM 全部置为中位。"""
        self.send("OFF")                       # 仅在 flag_float 为真时被处理，无副作用
        self.send("TES:" + ",".join([str(INIT_PWM)] * PROP_CH))

    def servo_center(self):
        self.send("MOT:" + ",".join([str(SERVO_CENTER)] * len(SERVO_BOARD_CH)))

    def all_stop(self):
        self.propeller_stop()
        self.servo_center()

    def close(self):
        self._stop.set()
        self._th.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# 终端：原始模式下的单键读取 + 全屏刷新
# --------------------------------------------------------------------------
_RAW = {"saved": None}      # 原始模式下保存的终端属性，供 ask_confirm() 临时还原


def _set_cbreak(fd: int):
    new = termios.tcgetattr(fd)
    new[3] &= ~(termios.ICANON | termios.ECHO)
    new[6][termios.VMIN] = 0
    new[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSADRAIN, new)


class RawKeys:
    """cbreak 模式（关 ICANON/ECHO，保留 ISIG 以便 Ctrl-C 仍能中断）。"""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        _RAW["saved"] = self.old
        _set_cbreak(self.fd)
        return self

    def __exit__(self, *exc):
        _RAW["saved"] = None
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        return False

    def key(self, timeout: float):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        ch = os.read(self.fd, 1)
        return ch.decode("utf-8", "replace") or None


def render(title: str, lines, footer: str):
    out = ["\033[H\033[J", info(title), ""]
    out += lines
    out += ["", footer]
    sys.stdout.write("\n".join(out))
    sys.stdout.flush()


def dashboard(link: Link, title: str, draw, footer: str, handlers: dict,
              period: float = 0.1):
    """全屏实时刷新的界面。handlers: 按键 -> 回调，回调返回 'quit' 则退出。"""
    with RawKeys() as keys:
        last = 0.0
        try:
            while True:
                now = time.time()
                if now - last >= period:
                    last = now
                    render(title, draw(), footer)
                k = keys.key(period)
                if k is None:
                    continue
                if k in ("q", "Q"):
                    return
                fn = handlers.get(k)
                if fn and fn() == "quit":
                    return
        finally:
            sys.stdout.write("\033[H\033[J")
            sys.stdout.flush()


def repl(prompt: str, banner, commands: dict, on_exit=None):
    """行输入的小命令行界面。commands: 命令名 -> (帮助, 回调)。"""
    print(banner)
    while True:
        try:
            line = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, _, rest = line.partition(" ")
        cmd = cmd.lower()
        if cmd in ("q", "quit", "exit"):
            break
        if cmd in ("?", "h", "help"):
            for name, (help_text, _) in commands.items():
                print(f"  {name:<18} {help_text}")
            continue
        entry = commands.get(cmd)
        if not entry:
            # 不是关键字：整行当作参数交给默认处理器（例如直接输入 "1700"）
            entry = commands.get("*")
            if entry:
                rest = line
        if not entry:
            print(f"  未知命令 '{cmd}'，输入 ? 看帮助")
            continue
        try:
            entry[1](rest)
        except Exception as e:                       # noqa: BLE001
            print(f"  {bad('错误')}: {e}")
    if on_exit:
        on_exit()


def ask_confirm(question: str) -> bool:
    """二次确认。会临时把终端还原成行模式（当前可能处于全屏原始模式）。"""
    fd = sys.stdin.fileno()
    saved = _RAW.get("saved")
    if saved is not None:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
    try:
        return input(f"  {warn('确认')} {question} [y/N] ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    finally:
        if saved is not None:
            _set_cbreak(fd)


# --------------------------------------------------------------------------
# 界面 1：陀螺仪 / 加速度计 / 姿态
# --------------------------------------------------------------------------
def screen_imu(link: Link):
    state = {"mode": "GYRO"}

    def switch(m):
        if link.mode(m) is None:
            state["err"] = "固件没有回应 OUT 命令"
        else:
            state["err"] = None
        state["mode"] = m

    switch("GYRO")

    def draw():
        age = link.newest_age()
        lines = [f"  数据流 {info('OUT:' + state['mode'])}  ({MODE_RATE[state['mode']]})"
                 f"   实测 {link.rate_for(state['mode']):5.1f} Hz"
                 f"   解析错误 {link.errors}"]
        if state.get("err"):
            lines.append(f"  {bad(state['err'])}")
        if age is None:
            lines.append(f"  {warn('还没有数据……')} 确认已烧录带遥测的固件、板子已运行")
            return lines
        if age > 0.5:
            lines.append(f"  {warn(f'数据已停更 {age:.1f}s')}")
        lines.append("")

        m = state["mode"]
        _, g = link.get("GYRO")
        _, a = link.get("ACC")
        _, r = link.get("RPY")
        _, t = link.get("TEMP")

        if g and m in ("GYRO", "ALL"):
            lines += [
                "  陀螺仪 BMI088  [rad/s]   x " + fmt(g[0]) + "  y " + fmt(g[1]) + "  z " + fmt(g[2]),
                "                 [deg/s]   x " + fmt(g[0] * 180 / 3.14159265, prec=2)
                + "  y " + fmt(g[1] * 180 / 3.14159265, prec=2)
                + "  z " + fmt(g[2] * 180 / 3.14159265, prec=2),
            ]
        if a and m in ("ACC", "ALL"):
            mag = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5
            verdict = ok("正常(~9.8)") if 9.2 < mag < 10.4 else warn("偏离 9.8，静止时请检查")
            lines.append("  加速度 BMI088  [m/s^2]   x " + fmt(a[0]) + "  y " + fmt(a[1]) + "  z " + fmt(a[2]))
            lines.append(f"  合加速度 |a| = {mag:.3f} m/s^2   {verdict}")
        if r and m in ("RPY", "ALL"):
            lines.append("  姿态 (Mahony+磁力计) [deg]   yaw " + fmt(r[0], prec=2)
                         + "  pitch " + fmt(r[1], prec=2) + "  roll " + fmt(r[2], prec=2))
        if t and m in ("TEMP", "ALL"):
            lines.append(f"  温度 = {t[0]:.2f} C   "
                         f"{'(正在加热到 45C)' if t[0] < 43 else '(已到目标附近)'}")
        if m == "ALL" and g and a and r:
            lines.append("")
            lines.append(dim("  提示：固件里 raw_data.gyro_offset 固定为 0（calibrate_offset() 从未调用），"))
            lines.append(dim("        所以静止时陀螺仪可能有小的固定零偏，属正常，不是硬件故障。"))
        return lines

    footer = ("  " + "  ".join(f"[{k}] {v}" for k, v in
                               [("1", "GYRO"), ("2", "ACC"), ("3", "RPY"),
                                ("4", "TEMP"), ("5", "ALL")]) + "   [q] 返回")
    dashboard(link, "陀螺仪 / 加速度计 / 姿态 (BMI088)", draw, footer, {
        "1": lambda: switch("GYRO"),
        "2": lambda: switch("ACC"),
        "3": lambda: switch("RPY"),
        "4": lambda: switch("TEMP"),
        "5": lambda: switch("ALL"),
    })


# --------------------------------------------------------------------------
# 界面 2：磁力计
# --------------------------------------------------------------------------
def screen_mag(link: Link):
    link.mode("MAG")

    def draw():
        age, v = link.get("MAG")
        lines = [f"  数据流 {info('OUT:MAG')}   实测 {link.rate('MAG'):5.1f} Hz   解析错误 {link.errors}", ""]
        if age is None or len(v) < 6:
            lines.append(f"  {warn('还没有数据……')}")
            return lines
        cx, cy, cz, rx, ry, rz = v[0], v[1], v[2], v[3], v[4], v[5]
        norm_c = (cx ** 2 + cy ** 2 + cz ** 2) ** 0.5
        norm_r = (rx ** 2 + ry ** 2 + rz ** 2) ** 0.5
        lines += [
            "  校准后 pro_data.mag [uT]  x " + fmt(cx, prec=3) + "  y " + fmt(cy, prec=3) + "  z " + fmt(cz, prec=3),
            f"  磁场强度 |B| = {norm_c:7.2f} uT   " +
            (ok("正常(地磁 25-65uT)") if 15 < norm_c < 80 else warn("超出地磁范围")),
            "",
            "  原始   raw_data.mag [uT]  x " + fmt(rx, prec=3) + "  y " + fmt(ry, prec=3) + "  z " + fmt(rz, prec=3),
            f"  原始强度     = {norm_r:7.2f} uT",
            "",
            dim("  · 校准是硬编码的椭圆修正 (MAG_OFFSET_*/MAG_SCALE_*, IMU.h)，换磁力计/换安装位置需要重标。"),
            dim("  · 转动整台机器，|B| 应该基本不变；若某个轴恒为 0 或数值打满，说明 IST8310 没读到。"),
            dim("  · 磁力计在 BMI088 上电时由 ist8310_init() 配置；若 yaw 一直漂，先看这里。"),
        ]
        return lines

    dashboard(link, "磁力计 (IST8310)", draw, "  [q] 返回", {})


# --------------------------------------------------------------------------
# 界面 3：水深传感器
# --------------------------------------------------------------------------
def screen_depth(link: Link):
    link.mode("PS")
    stats = {"min": [None] * 4, "max": [None] * 4, "t0": time.time(), "n": 0}

    def reset_stats():
        stats["min"] = [None] * 4
        stats["max"] = [None] * 4
        stats["t0"] = time.time()
        stats["n"] = 0

    def calibrate():
        print()
        if ask_confirm("重新校准零偏? 4 个传感器要在同一水平面（初始化时已减掉当地大气压）"):
            link.send("CA")            # 固件: flag_calibrate=true，下一次 Handle 里重测
            time.sleep(2.5)
            print("  已下发 CA，零偏已重测")

    def draw():
        age, v = link.get("PS")
        rate = link.rate("PS")
        lines = [f"  数据流 {info('OUT:PS')}   实测 {rate:5.1f} Hz "
                 f"(固件正常应约 100 Hz)   解析错误 {link.errors}", ""]
        if age is None or len(v) < 8:
            lines.append(f"  {warn('还没有数据……')}（固件需要是带遥测的版本）")
            return lines
        if rate < 20:
            lines.append(f"  {bad('警告')} 遥测只有 {rate:.0f} Hz —— 固件主循环被拖慢了"
                         f"（Handle() 正常是 300 Hz）。")
            lines.append(f"  {bad('     ')} 最常见原因：I2C 总线不通，HAL 的 I2C 调用一路超时"
                         f"（PCA 写入的超时是 10 秒）。")
            lines.append(f"  {bad('     ')} 这种情况下下面的读数和统计都不可信，先修 I2C。")
            lines.append("")
        if age > 1.5:
            lines.append(f"  {warn(f'最近一帧已经是 {age:.1f}s 前')}，"
                         f"数据太少时最小/最大/波动出不来。")
            lines.append("")
        p, present = v[0:4], v[4:8]
        stats["n"] += 1
        lines.append("                              当前读数        最小        最大      波动")
        for i in range(4):
            lo, hi = stats["min"][i], stats["max"][i]
            stats["min"][i] = p[i] if lo is None else min(lo, p[i])
            stats["max"][i] = p[i] if hi is None else max(hi, p[i])
            if stats["n"] > 1:
                st = (f"{stats['min'][i]:11.2f}  {stats['max'][i]:11.2f}"
                      f"  {stats['max'][i] - stats['min'][i]:8.2f}")
            else:
                st = f"{'--':>11}  {'--':>11}  {'--':>8}"
            lines.append(f"  #{i} (I2C 扩展板通道 {i})  {p[i]:9.2f} cm   "
                         + (ok("在位") if present[i] else bad("无应答"))
                         + "   " + st)
        depth = sum(p) / 4.0
        spread = max(p) - min(p)
        alive = [i for i in range(4) if present[i]]
        lines.append("")
        if alive:
            lines.append(f"  深度 = {depth:9.2f} cm  = {depth / 100.0:6.3f} m   (4 路平均)")
        else:
            lines.append(f"  深度 = {depth:9.2f} cm   {bad('<- 无效')}：4 路都没应答，"
                         f"这只是负的标定常数，不是真的水深")
        lines.append(f"  4 路最大差 = {spread:6.2f} cm   "
                     + (ok("一致") if spread < 5 else warn("偏差偏大")))
        lines.append("")
        if len(alive) < 4:
            lines.append(f"  {bad('诊断')} 通道 {[i for i in range(4) if i not in alive]} 没有应答"
                         f"（I2C 上没有读到有效 PROM）。")
            lines.append("        先量 I2C：C 板 IIC 口的 SCL/SDA 对 GND 不应是 0 欧；"
                         "两线空闲都该是 3.3V。")
            lines.append("        TCA9548A 的 A0-A2 开关要都接地（地址 0x70），"
                         "扩展板的 VCC 要有电。")
        elif all(abs(x) < 0.05 for x in p):
            lines.append(dim("  4 路都在位且读数≈0：在水面上是正常的（初始化已减掉大气压零偏）。"))
            lines.append(dim("  放进水里，读数应随深度每 1cm 增加约 1.0。"))
        else:
            lines.append(dim("  4 路都在位。注意：这里是『压强-零偏』，空气中≈0，水中≈水深(cm)。"))
        lines.append(dim(f"  统计窗口 {time.time() - stats['t0']:.0f}s / {stats['n']} 帧"))
        return lines

    dashboard(link, "水深传感器 (4x MS5837-30BA, I2C 扩展板通道 0-3)", draw,
              "  [c] 重新校准零偏(CA)   [r] 清统计   [q] 返回",
              {"c": calibrate, "r": reset_stats})


# --------------------------------------------------------------------------
# 界面 4：推进器
# --------------------------------------------------------------------------
def screen_propeller(link: Link):
    sel = {"ch": 0}
    prev_mode = link.query_mode()
    link.mode("OFF", wait_ack=False)
    time.sleep(0.1)
    link.propeller_stop()
    pwm = [INIT_PWM] * PROP_CH

    def push():
        link.send("TES:" + ",".join(str(x) for x in pwm))

    def show(_rest=""):
        print("  通道  " + "  ".join(f"{i:>5}" for i in range(PROP_CH)))
        print("  PWM   " + "  ".join(f"{v:>5}" for v in pwm))
        print("  偏量  " + "  ".join(f"{v - INIT_PWM:>+5}" for v in pwm))
        print(dim(f"  (中位 {INIT_PWM}=停转, 1000..2000 对应反转..正转; 固件会再 clamp 到 1000-2000)"))

    def set_channel(i, value, confirm=True):
        if not (PROP_MIN <= value <= PROP_MAX):
            print(f"  {bad('拒绝')} PWM {value} 超出 {PROP_MIN}-{PROP_MAX}")
            return
        delta = value - INIT_PWM
        if confirm and abs(delta) > 100:
            if not ask_confirm(f"通道 {i} 从 {pwm[i]} 改到 {value} (偏量 {delta:+d})，"
                               f"推进器会明显转动?"):
                print("  已取消")
                return
        pwm[i] = value
        push()
        print(f"  通道 {i} = {value}  (偏量 {delta:+d})")

    def do_set(rest):
        if not rest.strip():
            show()
            return
        set_channel(sel["ch"], int(rest.strip()))

    def do_ch(rest):
        i = int(rest.strip())
        if not (0 <= i < PROP_CH):
            print(f"  {bad('通道号必须 0-{PROP_CH - 1}')}")
            return
        sel["ch"] = i
        print(f"  当前通道 = {i}")

    def do_all(rest):
        value = int(rest.strip())
        if not (PROP_MIN <= value <= PROP_MAX):
            print(f"  {bad('拒绝')} PWM {value} 超出 {PROP_MIN}-{PROP_MAX}")
            return
        if abs(value - INIT_PWM) > 100 and not ask_confirm(f"8 个推进器全部设为 {value}?"):
            print("  已取消")
            return
        for i in range(PROP_CH):
            pwm[i] = value
        push()
        show()

    def do_stop(_rest=""):
        for i in range(PROP_CH):
            pwm[i] = INIT_PWM
        link.propeller_stop()
        print(f"  {ok('已停止')} 8 路全部回中位 {INIT_PWM}")

    def do_sweep(rest):
        parts = rest.split()
        if len(parts) != 3:
            print("  用法: sweep <A> <B> <秒>   例: sweep 1500 1700 3")
            return
        a, b, secs = int(parts[0]), int(parts[1]), float(parts[2])
        if not (PROP_MIN <= a <= PROP_MAX and PROP_MIN <= b <= PROP_MAX):
            print(f"  {bad('范围必须是 1000-2000')}")
            return
        n = max(2, int(secs * 20))
        i = sel["ch"]
        print(f"  通道 {i} 从 {a} 扫到 {b}，{secs}s（{n} 步）…")
        for k in range(n + 1):
            v = int(a + (b - a) * k / n)
            pwm[i] = v
            push()
            print(f"\r    通道 {i} = {v:>5}", end="")
            time.sleep(secs / n)
        print()
        pwm[i] = INIT_PWM
        push()
        print(f"  扫频结束，通道 {i} 回中位")

    banner = f"""
{info('推进器测试')}  PWM 扩展板通道 0-7
  {warn('注意')} 推进器一旦远离中位就会转，先把桨叶周围清空 / 或断开电机电源做信号测试。
  中位 1610 = 停转；1000-1500 反转，1500-2000 正转（越远越快）。
  8 个通道的物理方位由固件 Propeller.cpp 的 InID/OutID/Sign 决定。
"""
    print(banner)
    show()
    repl("  推进器> ", "", {
        "*":      ("<1000-2000>        设置当前通道", do_set),
        "ch":     ("<0-7>              选择当前通道", do_ch),
        "all":    ("<1000-2000>        8 个通道一起设", do_all),
        "stop":   ("                   全部回中位并退出浮潜模式", do_stop),
        "sweep":  ("<A> <B> <秒>       当前通道线性扫频", do_sweep),
        "p":      ("                   显示当前 PWM", show),
    }, on_exit=lambda: (do_stop(), restore_mode(link, prev_mode)))


def restore_mode(link: Link, mode):
    if mode in TELEMETRY_MODES and mode != "OFF":
        link.mode(mode, wait_ack=False)


# --------------------------------------------------------------------------
# 界面 5：舵机
# --------------------------------------------------------------------------
def screen_servo(link: Link):
    pwm = [SERVO_CENTER] * len(SERVO_BOARD_CH)
    sel = {"ch": 0}
    prev_mode = link.query_mode()
    link.mode("OFF", wait_ack=False)
    time.sleep(0.1)
    link.servo_center()

    def push():
        link.send("MOT:" + ",".join(str(x) for x in pwm))

    def angle(v):
        return (v - 500) * 180.0 / 2000.0       # 500-2500 -> 0-180

    def show(_rest=""):
        print("  固件索引   " + "  ".join(f"{i:>6}" for i in range(len(pwm))))
        print("  扩展板通道 " + "  ".join(f"{c:>6}" for c in SERVO_BOARD_CH))
        print("  PWM(us)    " + "  ".join(f"{v:>6}" for v in pwm))
        print("  估算角度   " + "  ".join(f"{angle(v):>6.1f}" for v in pwm))
        print(dim("  (500-2500us 线性对应 0-180 度；你的接线是 8-10 三个舵机, 11 号空着)"))

    def do_set(rest):
        if not rest.strip():
            show()
            return
        value = int(rest.strip())
        if not (SERVO_MIN <= value <= SERVO_MAX):
            print(f"  {bad('拒绝')} PWM {value} 超出 {SERVO_MIN}-{SERVO_MAX}")
            return
        i = sel["ch"]
        pwm[i] = value
        push()
        note = ""
        if i == SERVO_INVERT_IDX:
            note = warn("  (注意: 固件对索引 2 做了 3000-x 反向)")
        print(f"  索引 {i} (通道 {SERVO_BOARD_CH[i]}) = {value}us  ≈ {angle(value):.1f} 度{note}")

    def do_ch(rest):
        i = int(rest.strip())
        if not (0 <= i < len(pwm)):
            print(f"  {bad('索引必须 0-{len(pwm) - 1}')}")
            return
        sel["ch"] = i
        print(f"  当前索引 = {i} (扩展板通道 {SERVO_BOARD_CH[i]})")

    def do_all(rest):
        value = int(rest.strip())
        if not (SERVO_MIN <= value <= SERVO_MAX):
            print(f"  {bad('拒绝')} PWM {value} 超出 {SERVO_MIN}-{SERVO_MAX}")
            return
        for i in range(len(pwm)):
            pwm[i] = value
        push()
        show()

    def do_center(_rest=""):
        for i in range(len(pwm)):
            pwm[i] = SERVO_CENTER
        link.servo_center()
        print(f"  {ok('已回中位')} 500/1500/2500 对应 0/90/180 度")

    def do_sweep(rest):
        parts = rest.split()
        if len(parts) != 3:
            print("  用法: sweep <A> <B> <秒>   例: sweep 1000 2000 4   (先确认舵机不会撞到限位!)")
            return
        a, b, secs = int(parts[0]), int(parts[1]), float(parts[2])
        if not (SERVO_MIN <= a <= SERVO_MAX and SERVO_MIN <= b <= SERVO_MAX):
            print(f"  {bad('范围必须是 500-2500')}")
            return
        i = sel["ch"]
        n = max(2, int(secs * 10))
        print(f"  索引 {i} 从 {a} 扫到 {b}，{secs}s …")
        for k in range(n + 1):
            v = int(a + (b - a) * k / n)
            pwm[i] = v
            push()
            print(f"\r    索引 {i} = {v:>5}us ≈ {angle(v):5.1f} 度", end="")
            time.sleep(secs / n)
        print()
        pwm[i] = SERVO_CENTER
        push()
        print(f"  扫频结束，索引 {i} 回中位")

    print(f"\n{info('舵机测试')}  PWM 扩展板通道 {SERVO_BOARD_CH}\n"
          f"  {warn('注意')} 舵机会立刻转到指定位并堵转用力，先确认机械行程和供电(V+)。\n"
          "  1500us = 中位；500/2500us 是 0/180 度的极限，别一上来就打到极限。\n")
    show()
    repl("  舵机> ", "", {
        "*":      ("<500-2500>         设置当前索引", do_set),
        "ch":     ("<0-3>              选择当前索引", do_ch),
        "all":    ("<500-2500>         4 个一起设", do_all),
        "center": ("                   全部回中位 1500", do_center),
        "sweep":  ("<A> <B> <秒>       当前索引线性扫频", do_sweep),
        "p":      ("                   显示当前 PWM", show),
    }, on_exit=lambda: (do_center(), restore_mode(link, prev_mode)))


# --------------------------------------------------------------------------
# 界面 6：总览
# --------------------------------------------------------------------------
def screen_all(link: Link):
    link.mode("ALL")

    def draw():
        age = link.newest_age()
        lines = [f"  数据流 {info('OUT:ALL')} (~10 Hz)   实测 {link.rate_for('ALL'):5.1f} Hz"
                 f"   解析错误 {link.errors}", ""]
        if age is None:
            lines.append(f"  {warn('还没有数据……')}")
            return lines
        _, p = link.get("PS")
        if p and len(p) >= 8:
            lines.append("  水深 4 路 [cm]  " + "  ".join(f"{v:8.2f}" for v in p[:4])
                         + "   在位 " + "".join("1" if x else "0" for x in p[4:8]))
            lines.append(f"  深度 = {sum(p[:4]) / 4:8.2f} cm")
        _, r = link.get("RPY")
        if r:
            lines.append("  姿态 [deg]      yaw " + fmt(r[0], prec=2) + "  pitch " + fmt(r[1], prec=2)
                         + "  roll " + fmt(r[2], prec=2))
        _, g = link.get("GYRO")
        if g:
            lines.append("  陀螺 [rad/s]    x " + fmt(g[0]) + "  y " + fmt(g[1]) + "  z " + fmt(g[2]))
        _, a = link.get("ACC")
        if a:
            lines.append("  加速度 [m/s^2]  x " + fmt(a[0]) + "  y " + fmt(a[1]) + "  z " + fmt(a[2]))
        _, m = link.get("MAG")
        if m and len(m) >= 6:
            lines.append("  磁场 [uT]       校准 x " + fmt(m[0], prec=2) + "  y " + fmt(m[1], prec=2)
                         + "  z " + fmt(m[2], prec=2))
            lines.append("                  原始 x " + fmt(m[3], prec=2) + "  y " + fmt(m[4], prec=2)
                         + "  z " + fmt(m[5], prec=2))
        _, t = link.get("TEMP")
        if t:
            lines.append(f"  温度 = {t[0]:.2f} C")
        return lines

    dashboard(link, "全部传感器总览", draw, "  [q] 返回", {})


# --------------------------------------------------------------------------
# 界面 7：原始命令
# --------------------------------------------------------------------------
RAW_HELP = """
{title}  直接把字符串发到 422 串口（固件收到后会分发给各器件）

  固件已支持的命令（来自 userCode）：
    遥测    OUT:PS / OUT:GYRO / OUT:ACC / OUT:RPY / OUT:MAG / OUT:TEMP / OUT:ALL / OUT:OFF
            （单独发 OUT 会回一行当前选择）
    水深    CA                      重新测零偏        VA   置 flag_read_pres（固件里没人用）
    推进器  TES:<8个PWM>           直接给 PWM，例 TES:1610,1610,1500,1610,1610,1610,1610,1610
            ON / OFF                开/关浮潜 PID（ON 需要水深传感器可用）
            W/S/A/D/Z/E/Q           前/后/左/右/停/顺/逆 运动状态（仅 ON 之后有效）
            DN / UP / H:<值>        目标深度 -1 / +1 / 设为 值/10
            RPY:ON / RPY:OFF        开/关 yaw 角度控制
            ACL:ON / ACL:OF         角度闭环（仅 ON 之后有效）
            ANG:<角度>              设定 yaw 目标（仅 ON 之后有效）
    舵机    MOT:<4个PWM>           例 MOT:1500,1500,1500,1500   (索引 2 固件内部会 3000-x)
    惯导    BEG / END / IVA:BEG / IVA:END   （在固件里是空动作）

  输入 q 返回。输入 :hex 可以发原始字节，例如  :AA 55 01 。
""".format(title=info("原始命令 / 串口终端"))


def screen_raw(link: Link):
    print(RAW_HELP)
    while True:
        try:
            line = input("  422> ").rstrip("\n")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if line.strip() in ("q", "quit", "exit"):
            break
        if not line:
            continue
        if line.startswith(":"):
            try:
                raw = bytes(int(x, 16) for x in line[1:].split())
            except ValueError:
                print(f"  {bad('十六进制格式不对')}，例如 :AA 55 01")
                continue
            link.ser.write(raw)
            link.ser.flush()
            print(f"  -> {raw.hex(' ')}")
        else:
            link.send(line, gap=0.0)
            print(f"  -> {line}")
        # 收集这次发送之后固件回传的内容
        t_send = time.time()
        time.sleep(0.35)
        with link._lock:
            back = [t for ts, t in link.log if ts >= t_send]
        if back:
            for t in back[-8:]:
                print(f"  <- {t}")
        else:
            print(dim("  <- (0.35s 内没有回传；固件对大多数命令是不回显的)"))


# --------------------------------------------------------------------------
# 帮助
# --------------------------------------------------------------------------
HELP = """
{title}

  接线（和 Making_Instructions/README.md 一致）
    C 板 IIC  ->  I2C 扩展板 (TCA9548A, 地址 0x70)
                    通道 0-3 -> 4 个 MS5837 水深传感器
                    通道 4   -> PWM 扩展板 (PCA9685, 0x80, 50Hz)
    PWM 扩展板  通道 0-7  -> 8 个推进器
                通道 8-10 -> 3 个舵机（固件默认按 8-11 四个处理）
    C 板 UART2（丝印名！实为 STM32 USART1）-> TTL 转 422 -> 422 转 USB -> 本机

  必须先烧录带遥测的固件
    本程序依赖固件里的 "OUT:<what>" 命令把传感器数据回传。
    如果没有回应，请先在 Code/LL_Modified 里 ./build.sh 然后 ./flash-openocd.sh。

  固件里几个"看起来不对但其实正常"的地方
    · 陀螺仪零偏偏置没标定（raw_data.gyro_offset 恒为 0，calibrate_offset() 没被调用），
      所以静止时会有小零偏，这是设计如此，不是硬件坏了。
    · 水面静止时 4 路水深读数应该都≈0（初始化时已经把当地大气压当作零偏减掉了），
      下水后每 1cm 深度大约 +1.0（mbar ≈ cmH2O）。
    · 舵机索引 2（扩展板通道 10）在固件里被写成 3000-x，方向与其它三个相反。
    · 推进器只有 flag_float 为假时 TES: 才生效；如果之前发过 ON，先发 OFF。
    · 一个通道的读数"在位=0"表示 PROM 读不出来，多半是那一路的排线/供电没接好。

  常用排查顺序
    1. 陀螺仪/磁力计：转动机器，看数值是否跟着动；|a| 静止时≈9.8。
    2. 水深：4 路是否都"在位"；放进水里数值是否随深度上升。
    3. 舵机：先 center，再小幅度动，确认方向和行程。
    4. 推进器：先只发信号不接动力电确认；再接电，从小到大。
""".format(title=info("接线与命令帮助"))


# --------------------------------------------------------------------------
# 主菜单
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="ROV 下位机交互式硬件测试")
    ap.add_argument("--port", default=None, help="串口设备，默认自动查找")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-safe", action="store_true",
                    help="退出时不下发停止指令（默认会下发）")
    args = ap.parse_args()

    if not sys.stdin.isatty():
        sys.exit("这个程序需要交互式终端运行（stdin 不是 tty）")

    port = args.port or find_port()
    link = Link(port, args.baud)

    print(f"{info('ROV 硬件测试台')}   串口 {port} @ {args.baud} 8N1")
    mode = link.query_mode()
    if mode:
        print(f"  固件遥测命令可用，当前 {info('OUT:' + mode)}")
    else:
        print(f"  {warn('警告')} 固件没有回应 OUT 命令 —— 传感器界面会一直没数据。")
        print(f"        请先烧录带遥测的固件： ./build.sh && ./flash-openocd.sh")

    screens = {
        "1": ("陀螺仪/加速度计/姿态", screen_imu),
        "2": ("磁力计", screen_mag),
        "3": ("水深传感器", screen_depth),
        "4": ("推进器", screen_propeller),
        "5": ("舵机", screen_servo),
        "6": ("全部传感器总览", screen_all),
        "7": ("原始命令 / 串口终端", screen_raw),
    }

    try:
        while True:
            print()
            print(info("=" * 62))
            print(info("  ROV 硬件测试台") + f"      串口 {port} @ {args.baud}")
            print(info("=" * 62))
            print("   1  陀螺仪 / 加速度计 / 姿态   (BMI088)")
            print("   2  磁力计                     (IST8310)")
            print("   3  水深传感器                 (4x MS5837, I2C 扩展板 0-3)")
            print("   4  推进器                     (PWM 扩展板 0-7)")
            print("   5  舵机                       (PWM 扩展板 8-10)")
            print("   6  全部传感器总览")
            print("   7  原始命令 / 串口终端")
            print("   h  接线与命令帮助")
            print("   q  退出")
            try:
                choice = input("  选择> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if choice in ("q", "quit", "exit"):
                break
            if choice in ("h", "help", "?"):
                print(HELP)
                continue
            entry = screens.get(choice)
            if not entry:
                continue
            try:
                entry[1](link)
            except KeyboardInterrupt:
                print(f"\n  {warn('已中断')}")
                if not args.no_safe:
                    link.all_stop()
    finally:
        if not args.no_safe:
            link.all_stop()
            print(f"  {ok('已下发停止指令')} (推进器回 1610 / 舵机回 1500)")
        link.close()


def find_port():
    cands = sorted(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*")
                   + glob.glob("/dev/ttyACM*"))
    for c in cands:
        if "CMSIS-DAP" in c or "ttyACM" in c:      # 调试器，不是 422 转换器
            continue
        return os.path.realpath(c) if os.path.islink(c) else c
    return "/dev/ttyUSB0"


if __name__ == "__main__":
    main()
