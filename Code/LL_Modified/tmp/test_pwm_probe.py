"""离线校验；不连接 OpenOCD/串口。运行 python3 -m unittest discover -s tmp -p test_pwm_probe.py"""
import socket
import subprocess
import unittest
from unittest.mock import patch

import pwm_probe


class FakeSocket:
    def __init__(self, chunks):
        self.chunks = iter(chunks)

    def settimeout(self, _):
        pass

    def sendall(self, _):
        pass

    def recv(self, _):
        return next(self.chunks, b"")


class ProtocolTests(unittest.TestCase):
    def test_fragmented_frames_and_error(self):
        sock = FakeSocket([b"OK:0x", b"a1\x1aERROR:NACK\x1a"])
        with patch.object(socket, "create_connection", return_value=sock):
            ocd = pwm_probe.OpenOCD("unused", 0)
            self.assertEqual(ocd.call("read"), "0xa1")
            with self.assertRaisesRegex(pwm_probe.ProbeError, "NACK"):
                ocd.call("read")

    def test_closed_socket(self):
        with patch.object(socket, "create_connection", return_value=FakeSocket([])):
            with self.assertRaisesRegex(pwm_probe.ProbeError, "连接中断"):
                pwm_probe.OpenOCD("unused", 0).call("read")

    def tcl(self, body):
        result = subprocess.run(["tclsh"], input=pwm_probe.TCL + "\n" + body,
                                text=True, capture_output=True, check=True)
        self.assertEqual(result.stderr, "")
        return result.stdout.strip()

    def test_register_read_repeated_start_ack_and_nack(self):
        # 模拟0xa1的从机返回，地址/寄存器指针均ACK。
        result = self.tcl(r'''
set samples {3 2 2 3 2 3 2 3 2 2 2 2 3 3}
set sent {}
set events {}
proc pp_lines {} {
    global samples
    if {![llength $samples]} {error "unexpected extra read"}
    set value [lindex $samples 0]
    set samples [lrange $samples 1 end]
    return $value
}
proc pp_drive {pin released} {global events; lappend events [list $pin $released]}
proc pp_high {} {global events; lappend events H}
rename pp_tx original_tx
proc pp_tx {value} {global sent; lappend sent $value; return [original_tx $value]}
set value [pp_read_reg 0x40 0]
puts "$value|$sent|[llength $samples]|[lrange $events end-7 end]"
''')
        parts = result.split("|")
        self.assertEqual(parts[:3], ["161", "128 0 129", "0"])
        # 接收第8位之后保持SDA释放，发NACK，然后STOP。
        self.assertEqual(parts[3], "{2 0} {1 1} H {2 0} {2 0} {1 0} H {1 1}")

    def test_nack_never_read_as_register_data(self):
        result = self.tcl(r'''
set samples {3 3 3}
proc pp_lines {} {global samples; set v [lindex $samples 0]; set samples [lrange $samples 1 end]; return $v}
proc pp_drive {pin released} {}
proc pp_high {} {}
set code [catch {pp_read_reg 0x40 0} reason]
puts "$code|$reason"
''')
        self.assertEqual(result, "1|NACK: PCA write address")

    def test_scl_stuck_low_timeout(self):
        result = self.tcl(r'''
proc pp_drive {pin released} {}
proc pp_lines {} {return 1}
set code [catch {pp_high} reason]
puts "$code|$reason"
''')
        self.assertTrue(result.startswith("1|SCL 未能升高"))

    def test_restore_continues_after_tca_failure(self):
        class FakeOCD:
            def __init__(self):
                self.mem = {}
                self.calls = []

            def read(self, addr):
                return self.mem.get(addr, 0)

            def write(self, addr, value):
                self.mem[addr] = value

            masked = pwm_probe.OpenOCD.masked

            def call(self, cmd):
                self.calls.append(cmd)
                if cmd.startswith("pp_select"):
                    raise pwm_probe.ProbeError("SCL low")

        ocd = FakeOCD()
        session = pwm_probe.Session(ocd, lambda _: None)
        session.saved = {0x40021400: 0xaaaa000a, 0x40021404: 3,
                         0x4002140c: 5, 0x40021414: 1, 0x40005800: 0x301}
        ocd.mem[0x40021400] = 0xaaaa0005
        session.running = session.state_known = session.channel_touched = True
        session.old_channel = 0x10
        self.assertFalse(session.restore())
        self.assertEqual(ocd.mem[0x40021400], 0xaaaa000a)
        self.assertEqual(ocd.mem[0x40005800], 1)  # 不重放START/STOP。
        self.assertEqual(ocd.calls[-1], "resume")


if __name__ == "__main__":
    unittest.main()
