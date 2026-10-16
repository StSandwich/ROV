#!/usr/bin/env python3
"""
test-422.py — end-to-end test of the 422 upper-computer link.

It exercises both directions and, when a CMSIS-DAP probe is attached, also
observes the MCU side over SWD (OpenOCD) so a half-broken link can be told
apart from a firmware problem.

    ./test-422.py                       # auto: /dev/ttyUSB0, 115200, uses SWD
    ./test-422.py --port /dev/ttyUSB0 --baud 115200
    ./test-422.py --no-swd              # serial only (no DAP-Link needed)
    ./test-422.py --flood 0x55 --flood-seconds 30    # bench: measure the pair
    ./test-422.py -v                    # verbose

Link under test (RoboMaster C board):
    connector pin 1 = RXD -> STM32 PB7 (USART1_RX)
    connector pin 2 = TXD -> STM32 PA9 (USART1_TX)
    pin 3 = GND, pin 4 = 5V
Reminder: the case silkscreen is misleading - "UART2" is the STM32 USART1.
"""
from __future__ import annotations

import argparse
import os
import re
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ELF = os.path.join(HERE, "CubeMX", "build", "RM_Frame_C.elf")

# --- MCU addresses -----------------------------------------------------------
USART1 = 0x40011000
U1 = {"SR": 0x00, "DR": 0x04, "BRR": 0x08, "CR1": 0x0C, "CR2": 0x10, "CR3": 0x14}
GPIOB = 0x40020400
GB = {"MODER": 0x00, "PUPDR": 0x0C, "IDR": 0x10, "AFRL": 0x20}
GPIOA = 0x40020000
GA = {"MODER": 0x00, "IDR": 0x10, "AFRH": 0x24}
PIN_RX, PIN_TX = 7, 9          # PB7, PA9

OC_ARGS = ["openocd", "-f", "interface/cmsis-dap.cfg",
           "-c", "transport select swd", "-c", "adapter speed 1000",
           "-f", "target/stm32f4x.cfg"]

GREEN, RED, YEL, DIM, RST = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def ok(s):   return f"{GREEN}{s}{RST}"
def bad(s):  return f"{RED}{s}{RST}"
def warn(s): return f"{YEL}{s}{RST}"


# ---------------------------------------------------------------------------
class Symbols:
    """Resolve global symbols from the built ELF so addresses stay correct."""

    WANT = ["RxBuffer", "system_init_flag", "tick_300hz", "huart1"]

    def __init__(self, elf: str):
        self.addr = {}
        nm = subprocess.run(["arm-none-eabi-nm", "-C", elf],
                            capture_output=True, text=True)
        if nm.returncode != 0:
            raise RuntimeError("arm-none-eabi-nm failed: " + nm.stderr.strip())
        for line in nm.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[2] in self.WANT:
                self.addr[parts[2]] = int(parts[0], 16)

    def __getitem__(self, k):
        return self.addr[k]


class OpenOCD:
    """Talk to a running OpenOCD through its telnet port."""

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.p = subprocess.Popen(OC_ARGS, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        self.s = None
        for _ in range(60):
            try:
                self.s = socket.create_connection(("127.0.0.1", 4444), 0.5)
                break
            except OSError:
                if self.p.poll() is not None:
                    raise RuntimeError("openocd exited immediately")
                time.sleep(0.2)
        if self.s is None:
            raise RuntimeError("openocd telnet port 4444 did not come up")
        time.sleep(0.4)
        self.s.settimeout(0.3)
        self._drain()
        self.cmd("init", 1.5)

    def _drain(self):
        out = b""
        try:
            while True:
                d = self.s.recv(8192)
                if not d:
                    break
                out += d
        except OSError:
            pass
        return out

    def cmd(self, c, wait=0.15):
        self.s.sendall((c + "\n").encode())
        t0, out = time.time(), b""
        while time.time() - t0 < wait:
            try:
                d = self.s.recv(8192)
                if d:
                    out += d
            except OSError:
                pass
        if self.verbose and out:
            print(f"{DIM}    > {c}{RST}")
        return out.decode(errors="replace")

    def word(self, addr):
        for line in self.cmd("mdw 0x%08X 1" % addr).splitlines():
            if "%08x" % addr in line:
                for t in line.split(":")[-1].split():
                    if len(t) == 8 and all(c in "0123456789abcdefABCDEF" for c in t):
                        return int(t, 16)
        return None

    def bytes_at(self, addr, n):
        out = self.cmd("mdb 0x%08X %d" % (addr, n), 0.25)
        vals = []
        for line in out.splitlines():
            if "%08x" % addr in line:
                for t in line.split(":")[-1].split():
                    if len(t) == 2 and all(c in "0123456789abcdefABCDEF" for c in t):
                        vals.append(int(t, 16))
        return bytes(vals[:n])

    def close(self):
        try:
            self.s.sendall(b"shutdown\n")
        except OSError:
            pass
        time.sleep(0.3)
        self.p.terminate()
        try:
            self.p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.p.kill()


# ---------------------------------------------------------------------------
def find_port():
    import glob
    cands = sorted(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*"))
    for c in cands:
        if "CMSIS-DAP" in c or "ttyACM" in c:
            continue
        return os.path.realpath(c) if os.path.islink(c) else c
    return "/dev/ttyUSB0"


def main():
    ap = argparse.ArgumentParser(description="422 link test")
    ap.add_argument("--port", default=find_port())
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-swd", action="store_true", help="serial only")
    ap.add_argument("--flood", metavar="BYTE",
                    help="continuously send this byte (0x..) and exit - use it "
                         "so you can measure the RS-422 pair with a meter while "
                         "the PC is definitely transmitting")
    ap.add_argument("--flood-seconds", type=float, default=20.0,
                    help="duration for --flood (default 20 s)")
    ap.add_argument("--loopback", action="store_true",
                    help="send a unique pattern and report whether it comes "
                         "back (use it to test a cable/converter in isolation)")
    ap.add_argument("--no-reset", action="store_true",
                    help="do not reset the target before/after the test "
                         "(the RX interrupt can be left disabled by a previous "
                         "run, which would make the result misleading)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    import serial

    # ---------------- bench-measurement flood --------------------------------
    if args.loopback:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        pat = b"422LOOP-" + str(int(time.time())).encode() + b"\r\n"
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(pat)
        ser.flush()
        got = b""
        t0 = time.time()
        while time.time() - t0 < 2.0:
            d = ser.read(4096)
            if d:
                got += d
        ser.close()
        print(f"serial  : {args.port} @ {args.baud}")
        print(f"sent    : {pat!r}")
        print(f"received: {got!r}")
        if pat in got:
            print(f"{ok('LOOPBACK OK')} - driver + receiver + wiring all work")
            return 0
        if got:
            print(f"{bad('LOOPBACK GARBLED')} - something came back but not the "
                  f"pattern -> likely the pair is inverted (try T+<->R-, T-<->R+)")
        else:
            print(f"{bad('LOOPBACK FAILED')} - nothing came back: the TX pair is "
                  f"not reaching the RX pair (open wire / dead driver)")
        return 1

    if args.flood is not None:
        val = int(args.flood, 0)
        if not 0 <= val <= 255:
            sys.exit("--flood expects a byte value such as 0x00, 0x55 or 0xFF")
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"PC idle levels = record now, then flooding {args.flood} on "
              f"{args.port} for {args.flood_seconds:g} s ...")
        t0 = time.time()
        try:
            while time.time() - t0 < args.flood_seconds:
                ser.write(bytes([val]) * 256)
                ser.flush()
        except KeyboardInterrupt:
            pass
        ser.close()
        print("flood stopped")
        return 0

    result = {}

    # ---------------- serial --------------------------------------------------
    print(f"=== 422 link test  ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===")
    print(f"serial : {args.port} @ {args.baud} 8N1")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        sys.exit(f"{bad('FAIL')} cannot open {args.port}: {e}")
    time.sleep(0.2)
    ser.reset_input_buffer()

    oc = None
    sym = None
    if not args.no_swd:
        try:
            sym = Symbols(ELF)
            oc = OpenOCD(args.verbose)
            print("swd    : CMSIS-DAP / OpenOCD connected")
            if not args.no_reset:
                # start from a known-good state: a previous run (or an overrun
                # during normal use) can leave the RX interrupt disabled
                oc.cmd("reset run", 1.2)
                # wait for the firmware to finish Usermain()'s Init() (it does
                # I2C / PCA9685 setup with delays) before reading anything
                for _ in range(60):
                    if (oc.word(sym["system_init_flag"]) & 0xFF) == 1:
                        break
                    time.sleep(0.25)
                time.sleep(0.3)
        except Exception as e:
            print(f"{warn('WARN')} SWD unavailable ({e}); running serial-only")
            oc = None

    # ---------------- 1. firmware / UART configuration -----------------------
    if oc:
        print("\n--- 1. firmware + USART1 configuration (read over SWD) ---")
        flag = oc.word(sym["system_init_flag"]) & 0xFF
        tick = oc.word(sym["tick_300hz"]) & 0xFF
        cr1 = oc.word(USART1 + U1["CR1"])
        brr = oc.word(USART1 + U1["BRR"])
        hl_baud = oc.word(sym["huart1"] + 4)          # huart1.Init.BaudRate
        mb = oc.word(GPIOB + GB["MODER"])
        ma = oc.word(GPIOA + GA["MODER"])
        ab = oc.word(GPIOB + GB["AFRL"])
        aa = oc.word(GPIOA + GA["AFRH"])
        print(f"  system_init_flag = {flag}   tick_300hz = {tick}")
        print(f"  huart1 BaudRate  = {hl_baud}   (asked: {args.baud})")
        if brr:
            div = (brr >> 4) + (brr & 0xF) / 16.0
            print(f"  USART1 BRR       = 0x{brr:X} -> {84e6/16/div:.0f} baud")
        if cr1 is not None:
            print(f"  USART1 CR1       = 0x{cr1:04X}  UE={cr1>>13&1} TE={cr1>>3&1} "
                  f"RE={cr1>>2&1} IDLEIE={cr1>>4&1} RXNEIE={cr1>>5&1}")
            if not ((cr1 >> 5) & 1):
                print(f"  {bad('WARN')} the RX interrupt is OFF (RXNEIE=0) - the "
                      f"firmware stopped receiving")
                print("       (an overrun makes the HAL disable it; see "
                      "HAL_UART_ErrorCallback)")
                result["rx_armed"] = False
            else:
                result["rx_armed"] = True
        print(f"  PB7 (RX) MODER={mb>>14&3} AF={ab>>28&0xF}   "
              f"PA9 (TX) MODER={ma>>18&3} AF={aa>>4&0xF}")
        result["firmware"] = flag == 1

        # ---------------- 2. RX pin idle level / pull-up ---------------------
        print("\n--- 2. RXD (PB7) idle level ---")
        oc.cmd("halt", 0.3)
        pupdr = oc.word(GPIOB + GB["PUPDR"])
        raw = [(oc.word(GPIOB + GB["IDR"]) >> PIN_RX) & 1 for _ in range(15)]
        new = (pupdr & ~(3 << (PIN_RX * 2))) | (1 << (PIN_RX * 2))
        oc.cmd("mww 0x%08X 0x%08X" % (GPIOB + GB["PUPDR"], new), 0.3)
        time.sleep(0.2)
        up = [(oc.word(GPIOB + GB["IDR"]) >> PIN_RX) & 1 for _ in range(15)]
        oc.cmd("mww 0x%08X 0x%08X" % (GPIOB + GB["PUPDR"], pupdr), 0.3)
        print("  PB7 as-is        : " + "".join(map(str, raw)))
        print("  PB7 + internal PU: " + "".join(map(str, up)))
        idle_high = set(raw) == {1}
        can_float = set(up) == {1}
        if idle_high:
            print(f"  {ok('OK')} idle HIGH - looks like a proper UART line")
            result["rx_idle"] = True
        else:
            why = ("externally held LOW / shorted to GND"
                   if not can_float else "low while idle (suspicious)")
            print(f"  {bad('BAD')} idle LOW - {why}")
            print(f"       (an idle UART RX must be HIGH; with the {40} kOhm "
                  f"internal pull-up it {'went HIGH -> floating' if can_float else 'stayed LOW'})")
            result["rx_idle"] = False

    # ---------------- 3. TX direction: MCU -> PC -----------------------------
    print("\n--- 3. ROV -> PC  (transmit path) ---")
    if oc:
        magic = bytes([0x55, 0xAA, 0x31, 0x32, 0x33, 0x0D, 0x0A])
        ser.reset_input_buffer()
        oc.cmd("halt", 0.3)
        for b in magic:
            oc.cmd("mww 0x%08X 0x%02X" % (USART1 + U1["DR"], b), 0.08)
        oc.cmd("resume", 0.2)
        got = b""
        t0 = time.time()
        while time.time() - t0 < 1.5 and len(got) < len(magic):
            d = ser.read(64)
            if d:
                got += d
        print(f"  injected {magic!r} into USART1->DR")
        print(f"  received {got!r}")
        result["tx"] = got.startswith(magic)
        print(f"  {ok('OK') if result['tx'] else bad('FAIL')} ROV -> PC "
              f"{'works' if result['tx'] else 'did NOT deliver the injected bytes'}")
    else:
        got = b""
        t0 = time.time()
        while time.time() - t0 < 3.0:
            d = ser.read(4096)
            if d:
                got += d
        print(f"  passive listen 3 s: {len(got)} bytes {got[:80]!r}")
        result["tx"] = bool(got)
        print(f"  {ok('OK') if got else warn('?')} "
              f"(no SWD: only spontaneous firmware output can be seen)")

    # ---------------- 4. RX direction: PC -> MCU -----------------------------
    print("\n--- 4. PC -> ROV  (receive path) ---")
    stop = threading.Event()

    def flood(byte):
        while not stop.is_set():
            try:
                n = ser.write(bytes([byte]) * 32)
                ser.flush()
                if not n:
                    return
            except Exception:
                return

    # (a) deterministic test: halt the core so the USART cannot be drained by the
    #     ISR, then send a pattern and look at the RXNE flag / data register.
    if oc:
        pattern = bytes([0x41, 0x42, 0x43, 0x44])       # "ABCD"
        oc.cmd("halt", 0.3)
        ser.reset_input_buffer()
        n = ser.write(pattern)
        ser.flush()
        time.sleep(0.5)
        sr = oc.word(USART1 + U1["SR"])
        dr = oc.word(USART1 + U1["DR"])
        rxbuf = oc.bytes_at(sym["RxBuffer"], 16)
        print(f"  core halted, PC sent {n} bytes {pattern!r}")
        if sr is not None:
            print(f"  USART1 SR = 0x{sr:03X}  RXNE={sr>>5&1} ORE={sr>>3&1} "
                  f"FE={sr>>1&1} NE={sr>>2&1} IDLE={sr>>4&1}")
        print(f"  DR = {('0x%02X' % dr) if dr is not None else '?'}"
              f"{'  (expected 0x41)' if dr is not None else ''}")
        print(f"  RxBuffer = {rxbuf.hex(' ')}")
        rxne = bool(sr and (sr >> 5) & 1)
        dr_ok = (dr == pattern[0])
        oc.cmd("resume", 0.2)
        result["rx_halt"] = rxne
        if rxne and dr_ok:
            print(f"  {ok('OK')} data arrives AND decodes correctly (DR=0x{dr:02X})")
        elif rxne:
            print(f"  {bad('BAD')} data arrives but is CORRUPTED "
                  f"(DR=0x{dr:02X}, expected 0x{pattern[0]:02X})")
        else:
            print(f"  {bad('FAIL')} RXNE never set - data did not arrive")
    else:
        rxne = False
        print("  (no SWD: skipping the halt/RXNE test)")

    # (b) level test: sample PB7 (RX) and PA9 (TX) while the PC floods
    follows = False
    if oc:
        levels = {}
        for byte, name in ((0x00, "0x00"), (0xFF, "0xFF")):
            stop.clear()
            th = threading.Thread(target=flood, args=(byte,), daemon=True)
            th.start()
            time.sleep(0.3)
            rx = [(oc.word(GPIOB + GB["IDR"]) >> PIN_RX) & 1 for _ in range(20)]
            tx = [(oc.word(GPIOA + GA["IDR"]) >> PIN_TX) & 1 for _ in range(20)]
            stop.set()
            th.join(timeout=1)
            time.sleep(0.15)
            levels[name] = rx
            print(f"  PC sends {name}: PB7(RX)=" + "".join(map(str, rx))
                  + "  PA9(TX)=" + "".join(map(str, tx)))
        follows = len({tuple(v) for v in levels.values()}) > 1
        if not follows and oc:
            # does anything external disturb the MCU's own TX pin?
            if any(0 in levels[b] for b in levels):
                print(f"  {warn('note')} PA9 (the MCU's TX output) shows dips while "
                      f"the PC transmits")
                print("       -> the adapter's TX appears to be wired onto the MCU's")
                print("          TXD pin: the cable is crossed. Revert it.")
                result["crossed"] = True

    # (c) does the firmware answer the "VA" command at all?
    stop.set()
    ser.reset_input_buffer()
    ser.write(b"VA")
    ser.flush()
    reply = b""
    t0 = time.time()
    while time.time() - t0 < 1.5:
        d = ser.read(4096)
        if d:
            reply += d
    print(f"  reply to b'VA': {len(reply)} bytes {reply[:60]!r}")

    result["rx"] = rxne and dr_ok
    if result["rx"]:
        print(f"  {ok('OK')} PC -> ROV works")
    elif rxne and not dr_ok:
        crossing = not result.get("rx_idle", True)
        print(f"  {bad('FAIL')} the signal reaches the MCU but cannot be decoded"
              f"{' - and PB7 idles LOW' if crossing else ''}")
        print(f"  {warn('-> the pair is inverted: swap T/R+ <-> T/R- for THIS pair')}")
        result["inverted"] = True
    else:
        print(f"  {bad('FAIL')} PB7 never sees the PC traffic - data does not "
              f"reach the MCU")

    # ---------------- 5. passive listen --------------------------------------
    print("\n--- 5. spontaneous firmware output (5 s) ---")
    ser.reset_input_buffer()
    got = b""
    t0 = time.time()
    while time.time() - t0 < 5.0:
        d = ser.read(4096)
        if d:
            got += d
    print(f"  {len(got)} bytes")
    if got:
        print("  " + repr(got[:200]))
    result["stream"] = bool(got)

    ser.close()
    if oc:
        if not args.no_reset:
            oc.cmd("reset run", 1.0)      # leave the board in a clean state
        oc.close()

    # ---------------- verdict -------------------------------------------------
    print("\n=== verdict ===")
    rows = [("firmware running / UART configured", "firmware"),
            ("RXD (PB7) idle level sane",          "rx_idle"),
            ("ROV -> PC (transmit)",               "tx"),
            ("PC -> ROV (receive)",                "rx")]
    for label, key in rows:
        v = result.get(key)
        s = ok("PASS") if v else (warn("n/a") if v is None else bad("FAIL"))
        print(f"  {s}  {label}")
    if result.get("inverted"):
        print(f"\n  {warn('Diagnosis:')} the pair carrying PC -> ROV is inverted.")
        print("  Data arrives but the line idles LOW, so the UART can never")
        print("  frame a byte (that is also why it reads garbage in DR).")
        print("  -> swap the two wires belonging to THAT pair only")
        print("     (T/R+ <-> T/R-), then re-run ./test-422.py")
    elif result.get("tx") and not result.get("rx"):
        print(f"\n  {warn('Diagnosis:')} the link works in one direction only.")
        print("  The MCU side is fine -> check the RXD wire (connector pin 1 ->")
        print("  PB7): it must not be shorted to GND pin 3, and the RS-422 pair")
        print("  polarity may need swapping (T/R+ <-> T/R-).")
    return 0 if all(result.get(k) for _, k in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
