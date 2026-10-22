#!/usr/bin/env python3
"""STM32F407 PF0/PF1 + TCA9548A + PCA9685 独立诊断，仅依赖 Python 标准库。

先启动 OpenOCD，再运行 python3 tmp/pwm_probe.py。
不依赖 ELF/SRAM 地址，不烧录，不写 PWM 配置或占空比寄存器。
测试期间暂停 MCU；现有 PWM 输出仍可能保持，退出时尝试恢复配置及运行状态。
"""

import argparse
import datetime
from pathlib import Path
import socket
import sys
import time


TCL = r"""
proc pp_drive {pin released} {
    if {$released} {set value $pin} else {set value [expr {$pin << 16}]}
    write_memory 0x40021418 32 [list $value]
    after 1
}
proc pp_lines {} {return [expr {[lindex [read_memory 0x40021410 32 1] 0] & 3}]}
proc pp_high {} {
    pp_drive 2 1
    for {set i 0} {$i < 25} {incr i} {
        if {[expr {[pp_lines] & 2}]} {return}
        after 1
    }
    error "SCL 未能升高：已释放开漏输出，但 PF1 仍读低"
}
proc pp_start {} {
    pp_drive 1 1
    pp_high
    if {![expr {[pp_lines] & 1}]} {error "SDA 未能升高：PF0 读低"}
    pp_drive 1 0
    pp_drive 2 0
}
proc pp_stop {} {
    pp_drive 2 0
    pp_drive 1 0
    pp_high
    pp_drive 1 1
    if {[pp_lines] != 3} {error "STOP 后总线未回到两根均高"}
}
proc pp_tx {value} {
    for {set bit 7} {$bit >= 0} {incr bit -1} {
        pp_drive 1 [expr {($value >> $bit) & 1}]
        pp_high
        pp_drive 2 0
    }
    pp_drive 1 1
    pp_high
    set ack [expr {([pp_lines] & 1) == 0}]
    pp_drive 2 0
    return $ack
}
proc pp_expect {value label} {
    if {![pp_tx $value]} {
        pp_stop
        error "NACK: $label"
    }
}
proc pp_rx {} {
    set value 0
    pp_drive 1 1
    for {set bit 7} {$bit >= 0} {incr bit -1} {
        pp_high
        set value [expr {($value << 1) | ([pp_lines] & 1)}]
        pp_drive 2 0
    }
    # 单字节读取结束，主机发送 NACK。
    pp_drive 1 1
    pp_high
    pp_drive 2 0
    return $value
}
proc pp_probe {addr} {
    pp_start
    set ack [pp_tx [expr {$addr << 1}]]
    pp_stop
    return $ack
}
proc pp_read_control {addr} {
    pp_start
    pp_expect [expr {($addr << 1) | 1}] "TCA read address"
    set value [pp_rx]
    pp_stop
    return $value
}
proc pp_select {addr mask} {
    pp_start
    pp_expect [expr {$addr << 1}] "TCA write address"
    pp_expect $mask "TCA channel mask"
    pp_stop
}
proc pp_read_reg {addr reg} {
    pp_start
    pp_expect [expr {$addr << 1}] "PCA write address"
    pp_expect $reg "PCA register pointer"
    pp_start
    pp_expect [expr {($addr << 1) | 1}] "PCA read address"
    set value [pp_rx]
    pp_stop
    return $value
}
"""


class ProbeError(RuntimeError):
    pass


class OpenOCD:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(10)
        self.pending = b""

    def call(self, command):
        # OpenOCD TCL 使用 0x1a 分帧；保留半包及下一帧的数据。
        wrapped = ('if {[catch {\n' + command + '\n} pp_result]} '
                   '{format "ERROR:%s" $pp_result} else {format "OK:%s" $pp_result}')
        self.sock.sendall(wrapped.encode() + b"\x1a")
        while b"\x1a" not in self.pending:
            data = self.sock.recv(65536)
            if not data:
                raise ProbeError("OpenOCD 连接中断；无法保证已自动恢复 MCU")
            self.pending += data
            if len(self.pending) > 1024 * 1024:
                raise ProbeError("OpenOCD 响应过长")
        result, self.pending = self.pending.split(b"\x1a", 1)
        result = result.decode(errors="replace")
        if not result.startswith("OK:"):
            raise ProbeError(result.removeprefix("ERROR:"))
        return result[3:].strip()

    def read(self, addr):
        return int(self.call(f"read_memory 0x{addr:x} 32 1").split()[0], 0)

    def write(self, addr, value):
        self.call(f"write_memory 0x{addr:x} 32 {{0x{value & 0xffffffff:x}}}")

    def masked(self, addr, mask, value):
        self.write(addr, (self.read(addr) & ~mask) | (value & mask))


class Session:
    def __init__(self, ocd, log):
        self.ocd, self.log = ocd, log
        self.saved = {}
        self.running = False
        self.state_known = False
        self.old_channel = None
        self.channel_touched = False

    def begin(self):
        # 0x413 对应 STM32F405/407/415/417 系列。
        ident = self.ocd.read(0xe0042000)
        if ident & 0xfff != 0x413:
            raise ProbeError(f"MCU ID=0x{ident:x}，不是本脚本支持的 STM32F407 系列")
        state = self.ocd.call("stm32f4x.cpu curstate")
        if state not in ("running", "halted"):
            raise ProbeError(f"MCU 状态异常：{state}")
        self.running = state == "running"
        self.state_known = True
        self.ocd.call("halt; wait_halt 1000")
        self.log(f"MCU 已暂停，原状态={state}，DBGMCU_IDCODE=0x{ident:08x}")
        # 保存时钟后，先确保 GPIOF/I2C2 寄存器可访问。
        for addr, bit in ((0x40023830, 1 << 5), (0x40023840, 1 << 22)):
            self.saved[addr] = self.ocd.read(addr)
            self.ocd.masked(addr, bit, bit)
        for addr in (0x40005800, 0x40021400, 0x40021404, 0x4002140c, 0x40021414):
            self.saved[addr] = self.ocd.read(addr)
        self.ocd.masked(0x40005800, 1, 0)  # 关闭 I2C2 PE。
        self.ocd.masked(0x40021400, 15, 0)  # PF0/PF1 输入。
        self.ocd.masked(0x4002140c, 15, 0)  # 无上下拉，观察外部实际电平。
        self.ocd.call(TCL)

    def lines(self):
        bits = int(self.ocd.call("pp_lines"), 0)
        return f"SDA(PF0)={bits & 1}  SCL(PF1)={(bits >> 1) & 1}"

    def gpio_bus(self):
        self.ocd.write(0x40021418, 3)  # 先释放输出锁存，避免切换模式时拉低。
        self.ocd.masked(0x40021404, 3, 3)  # 开漏，绝不主动推挽输出高电平。
        self.ocd.masked(0x40021400, 15, 5)  # PF0/PF1 普通输出。

    def float_bus(self):
        self.ocd.masked(0x40021400, 15, 0)
        self.ocd.masked(0x4002140c, 15, 0)

    def select(self, mask):
        self.channel_touched = True
        self.ocd.call(f"pp_select 0x70 {mask}")
        if mask:
            # 下游PCA9685可能同时响应ALLCALL 0x70，不能用共享地址回读校验TCA。
            self.log(f"TCA 通道掩码0x{mask:02x}已发送并收到ACK；{self.lines()}")
            self.log("非零通道不回读0x70，避免PCA All Call与TCA同时响应导致误判。")
            return
        actual = int(self.ocd.call("pp_read_control 0x70"), 0)
        if actual != mask:
            raise ProbeError(f"TCA 通道回读不符：写入0x{mask:02x}，回读0x{actual:02x}")
        self.log(f"TCA 通道已确认：0x{actual:02x}；{self.lines()}")

    def restore(self):
        errors = []
        if self.channel_touched and self.old_channel is not None:
            try:
                self.gpio_bus()
                self.ocd.call(f"pp_select 0x70 {self.old_channel}")
            except Exception as exc:
                errors.append(f"TCA 通道恢复失败：{exc}")
        # 先回输入，再还原输出锁存/类型/上拉和复用，最后重新使能 I2C。
        operations = []
        if 0x40021400 in self.saved:
            operations.append(lambda: self.ocd.masked(0x40021400, 15, 0))
        if 0x40021414 in self.saved:
            old = self.saved[0x40021414] & 3
            operations.append(lambda: self.ocd.write(0x40021418, old | ((3 & ~old) << 16)))
        for addr, mask in ((0x40021404, 3), (0x4002140c, 15), (0x40021400, 15)):
            if addr in self.saved:
                operations.append(lambda addr=addr, mask=mask: self.ocd.masked(addr, mask, self.saved[addr]))
        if 0x40005800 in self.saved:
            # 不重放已过期的 START/STOP 请求。
            operations.append(lambda: self.ocd.write(0x40005800, self.saved[0x40005800] & ~0x300))
        for addr, bit in ((0x40023840, 1 << 22), (0x40023830, 1 << 5)):
            if addr in self.saved:
                operations.append(lambda addr=addr, bit=bit: self.ocd.masked(addr, bit, self.saved[addr]))
        for operation in operations:
            try:
                operation()
            except Exception as exc:
                errors.append(str(exc))
        if self.state_known and self.running:
            try:
                self.ocd.call("resume")
            except Exception as exc:
                errors.append(f"MCU 恢复运行失败：{exc}")
        for error in errors:
            self.log("恢复警告：" + error)
        if not errors and self.state_known:
            self.log("已恢复 MCU 引脚、I2C 配置及原运行状态。被中断的传输可能由固件重试。")
        return not errors


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6666)
    parser.add_argument("--channel", type=int, choices=range(8), default=4)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x40, help="PCA 的7位地址，默认0x40，不是HAL地址0x80")
    parser.add_argument("--measure-only", action="store_true", help="只释放 MCU 引脚，保持现有 TCA 通道不变，连续采样")
    parser.add_argument("--pin-test", action="store_true", help="PF1开漏拉低/释放两阶段，核对实际SCL针脚；不发送I2C地址")
    parser.add_argument("--channel-only", action="store_true", help="仅选通指定TCA通道并观察电平，不访问PCA")
    parser.add_argument("--seconds", type=int, default=20, help="--measure-only 持续秒数，默认20")
    parser.add_argument("--scan", action="store_true", help="选通PWM分支后扫描普通7位地址，仅探测ACK")
    parser.add_argument("--no-pause", action="store_true", help="跳过万用表测量阶段的回车等待")
    args = parser.parse_args()
    if sum((args.pin_test, args.measure_only, args.channel_only)) > 1:
        parser.error("--pin-test、--measure-only、--channel-only 不可同时使用")
    if not 0x08 <= args.address <= 0x77 or args.seconds <= 0:
        parser.error("地址必须在0x08～0x77，秒数必须大于0")
    log_path = Path(__file__).resolve().parent / f"pwm_probe_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"
    with log_path.open("x", encoding="utf-8") as logfile:
        def log(text):
            line = f"[{datetime.datetime.now():%H:%M:%S}] {text}"
            print(line, flush=True)
            logfile.write(line + "\n")
            logfile.flush()

        def pause(text):
            log(text)
            if not args.no_pause:
                answer = input("总线将保持静止，测量后按回车继续；输入 q 退出：").strip()
                if answer.lower() == "q":
                    raise KeyboardInterrupt

        log(f"日志：{log_path}")
        log("测试期间 MCU 暂停；PCA 现有 PWM 输出可能继续保持。请在允许暂停控制时测试。")
        ocd = None
        session = None
        status = 0
        try:
            ocd = OpenOCD(args.host, args.port)
            session = Session(ocd, log)
            session.begin()
            log("[A] 原通道保持不变，MCU 不驱动、无内部上下拉：" + session.lines())
            if args.pin_test:
                session.gpio_bus()
                for released, name in ((0, "L：PF1主动开漏拉低"), (1, "H：PF1释放，由外部上拉")):
                    ocd.call("pp_drive 1 1")
                    ocd.call(f"pp_drive 2 {released}")
                    log(f"[引脚对应测试 {name}] {session.lines()}")
                    log(f"GPIOF MODER=0x{ocd.read(0x40021400):08x} OTYPER=0x{ocd.read(0x40021404):08x} ODR=0x{ocd.read(0x40021414):08x}")
                    if args.no_pause:
                        log(f"本阶段保持{args.seconds}秒，请测C板SCL对C板GND。")
                        for elapsed in range(args.seconds):
                            if elapsed % 10 == 0:
                                log(f"{name} 已保持{elapsed}秒：{session.lines()}")
                            time.sleep(1)
                    else:
                        pause("请测C板和PWM板的SCL对各自GND，记录本阶段电压。")
                session.float_bus()
                log("两阶段完成，已释放为无上下拉输入：" + session.lines())
            elif args.measure_only:
                for _ in range(args.seconds):
                    log(session.lines())
                    time.sleep(1)
            else:
                pause("请同时测 C 板/PWM 板 SDA、SCL 对各自 GND，并记录电压。")
                session.gpio_bus()
                session.old_channel = int(ocd.call("pp_read_control 0x70"), 0)
                log(f"0x70 应答，原通道回读=0x{session.old_channel:02x}（下游All Call可能使此值失真，退出仅按此值尽力恢复）")
                session.select(0)
                pause("[B] TCA 全通道已断开。测 C 板 SDA/SCL；下游电平此时不代表上游。")
                session.select(1 << args.channel)
                pause(f"[C] 仅选通 {args.channel} 号口。测两侧 SDA/SCL，检查是否出现电平差异。")
                if args.channel_only:
                    for elapsed in range(args.seconds):
                        log(f"通道{args.channel} 已选通{elapsed}秒：{session.lines()}")
                        time.sleep(1)
                    log("通道观察结束，未访问PCA。")
                if args.scan and not args.channel_only:
                    found = []
                    for addr in range(0x08, 0x78):
                        if int(ocd.call(f"pp_probe {addr}"), 0):
                            found.append(f"0x{addr:02x}")
                    log("7位地址扫描 ACK：" + ", ".join(found))
                if not args.channel_only and not int(ocd.call(f"pp_probe {args.address}"), 0):
                    raise ProbeError(f"PCA 7位地址0x{args.address:02x} 无ACK；检查逻辑供电、地址跳线和线序，可加 --scan")

                def reg(addr):
                    return int(ocd.call(f"pp_read_reg {args.address} {addr}"), 0)

                if not args.channel_only:
                    mode1, mode2, prescale = reg(0), reg(1), reg(0xfe)
                    log(f"[D] PCA有效回读：MODE1=0x{mode1:02x} MODE2=0x{mode2:02x} PRESCALE={prescale}")
                    log(f"SLEEP={(mode1 >> 4) & 1} EXTCLK={(mode1 >> 6) & 1}；按内部典型25MHz估算PWM频率={25000000/(4096*(prescale+1)):.2f}Hz")
                    for channel in range(8, 12):
                        base = 6 + channel * 4
                        data = [reg(base + i) for i in range(4)]
                        on = data[0] | ((data[1] & 15) << 8)
                        off = data[2] | ((data[3] & 15) << 8)
                        log(f"通道{channel}: ON={on} OFF={off} FULL_ON={(data[1] >> 4) & 1} FULL_OFF={(data[3] >> 4) & 1}")
                    log("读取完成：低速软件I2C可以通信；这不证明100kHz波形、动力供电或物理PWM输出正常。")
                    pause("测试结束，可再次测量静态电平；随后恢复原配置。")
        except KeyboardInterrupt:
            log("用户结束测试，开始恢复。")
            status = 130
        except (OSError, ValueError, ProbeError) as exc:
            log("测试失败：" + str(exc))
            if ocd is None:
                log("请先启动：openocd -f interface/cmsis-dap.cfg -c 'transport select swd' -c 'adapter speed 1000' -f target/stm32f4x.cfg -c init")
            elif session and session.saved:
                try:
                    # 失败可能发生在STOP的SDA低阶段，先释放引脚再让用户测量。
                    if 0x40021400 in session.saved:
                        session.float_bus()
                    log("失败后的引脚状态：" + session.lines())
                    pause("若需万用表核对，请现在测量并记录；不会自动发送9脉冲或复位来清除现场。")
                except (KeyboardInterrupt, EOFError, OSError, ProbeError):
                    pass
            status = 1
        except EOFError:
            log("无交互输入；批处理请加 --no-pause。")
            status = 1
        finally:
            if session and not session.restore():
                status = 1
            if ocd:
                ocd.sock.close()
        return status


if __name__ == "__main__":
    sys.exit(main())
