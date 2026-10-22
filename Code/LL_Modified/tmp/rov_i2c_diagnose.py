import argparse
import collections
import json
import os
import socket
import struct
import time
import serial

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=int, default=45)
p.add_argument('--mode', default='PS')
p.add_argument('--verify-image', action='store_true')
p.add_argument('--isolate', action='store_true')
a = p.parse_args()

def rpc(cmd):
    with socket.create_connection(('127.0.0.1', 6666), timeout=3) as s:
        s.sendall(cmd.encode() + b'\x1a')
        data = b''
        while b'\x1a' not in data:
            data += s.recv(65536)
        return data.split(b'\x1a')[0].decode()

def snapshot():
    regs = {}
    for name, addr, count in [
        ('error_counters_PCA_TCA_RECOVER_I2C', 0x20002af8, 4),
        ('I2C2_CR1_CR2', 0x40005800, 2),
        ('GPIOF_IDR', 0x40021410, 1),
        ('GPIOF_MODER', 0x40021400, 1),
        ('hi2c2_State_Mode', 0x20000958, 1),
        ('hi2c2_ErrorCode', 0x2000095c, 1),
        ('uwTick', 0x20002ad8, 1),
        ('I2C2_CCR_TRISE', 0x4000581c, 2),
        ('TIM1_PSC_ARR', 0x40010028, 2),
        ('servo_targets', 0x2000407c, 4),
        ('sensor_pressure_raw_bits', 0x20003628, 4),
    ]:
        regs[name] = rpc(f'read_memory 0x{addr:x} 32 {count}')
    regs['sensor_pressure_raw'] = [struct.unpack('<f', struct.pack('<I', int(v, 0)))[0] for v in regs['sensor_pressure_raw_bits'].split()]
    return regs

if a.isolate:
    def readword(addr):
        return int(rpc(f'read_memory 0x{addr:x} 32 1').split()[0], 0)
    def writeword(addr, val):
        rpc(f'write_memory 0x{addr:x} 32 {{0x{val:x}}}')
    rpc('halt')
    saved = {}
    try:
        print('HALTED_PC', rpc('reg pc'), flush=True)
        for addr in [0x40005800, 0x40021400, 0x4002140c]:
            saved[addr] = readword(addr)
        print('SAVED', saved, flush=True)
        writeword(0x40005800, saved[0x40005800] & ~1)
        writeword(0x40021400, saved[0x40021400] & ~15)
        writeword(0x4002140c, saved[0x4002140c] & ~15)
        vals = [readword(0x40021410) & 3 for _ in range(20)]
        print('MCU_RELEASED_SDA_BIT0_SCL_BIT1', vals, flush=True)
    finally:
        for addr in [0x4002140c, 0x40021400, 0x40005800]:
            if addr in saved:
                writeword(addr, saved[addr])
        rpc('resume')
        print('RESTORED_AND_RESUMED', flush=True)
    raise SystemExit

if a.verify_image:
    size = os.path.getsize('/tmp/rov_i2c_current.bin')
    print('DUMP', rpc(f'dump_image /tmp/rov_i2c_flash.bin 0x08000000 {size}'), flush=True)
    with open('/tmp/rov_i2c_current.bin', 'rb') as f:
        expected = f.read()
    with open('/tmp/rov_i2c_flash.bin', 'rb') as f:
        actual = f.read()
    print('FLASH_MATCH', actual == expected, 'bytes', size, flush=True)

with serial.Serial('/dev/ttyUSB0', 115200, timeout=0.15) as ser:
    ser.reset_input_buffer()
    ser.write(('OUT:' + a.mode + '\n').encode())
    start = time.monotonic()
    print('START', json.dumps(snapshot()), flush=True)
    lines = collections.Counter()
    samples = collections.defaultdict(list)
    last = {}
    gaps = collections.defaultdict(list)
    raw = []
    buf = b''
    next_report = start + 15
    while time.monotonic() - start < a.seconds:
        buf += ser.read(4096)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.decode(errors='replace').strip()
            now = time.monotonic()
            key = text.split(',')[0]
            lines[key] += 1
            if len(raw) < 10:
                raw.append(text)
            if ',' in text:
                try:
                    vals = [float(v) for v in text.split(',')[1:]]
                    samples[key].append(vals)
                    if key in last:
                        gaps[key].append(now - last[key])
                    last[key] = now
                except ValueError:
                    lines['PARSE_ERROR'] += 1
        if time.monotonic() >= next_report:
            print('PROGRESS', round(time.monotonic()-start, 1), dict(lines), json.dumps(snapshot()), flush=True)
            next_report += 15
    result = {'duration_s': time.monotonic()-start, 'counts': dict(lines), 'first_lines': raw, 'registers_end': snapshot()}
    for key, rows in samples.items():
        result[key] = {'last': rows[-1], 'min': [min(c) for c in zip(*rows)], 'max': [max(c) for c in zip(*rows)], 'max_read_gap_s': max(gaps[key], default=0)}
    print('RESULT', json.dumps(result), flush=True)
