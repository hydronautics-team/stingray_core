#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Эмулятор LC-AH127C для теста ноды без датчика.

Порты: socat -d -d PTY,link=/tmp/ttyEmulator,raw,echo=0 PTY,link=/tmp/ttyROS,raw,echo=0
Запуск: python3 emulator.py [--port /tmp/ttyEmulator] [--rate 10] [--log data.log]

Без --log генерирует синтетические кадры (плавно меняющиеся углы) с правильной
контрольной суммой. С --log проигрывает hex-дамп (ищет кадры 77 38 00 59 ... по 57 байт).
Отвечает на команды 0x0C (частота), 0x56 (тип), 0x0A (сохранить), 0x59 (запрос кадра),
0x04 (три угла), 0x1F (адрес).
"""
import argparse
import math
import re
import sys
import time

try:
    import serial
except ImportError:
    print("pip install pyserial")
    sys.exit(1)

RATE_CODES = {0: 0, 1: 5, 2: 10, 3: 20, 4: 25, 5: 50}


def bcd_angle(v):            # SXXX.YY -> 3 байта
    s = 0x10 if v < 0 else 0
    v = abs(v)
    i = int(v)
    f = int(round((v - i) * 100)) % 100
    return bytes([s | (i // 100), ((i // 10) % 10) << 4 | (i % 10), (f // 10) << 4 | (f % 10)])


def bcd_accel(v):            # SX.YYYY
    s = 0x10 if v < 0 else 0
    v = abs(v)
    i = int(v)
    f = int(round((v - i) * 10000)) % 10000
    d = [f // 1000, (f // 100) % 10, (f // 10) % 10, f % 10]
    return bytes([s | i, d[0] << 4 | d[1], d[2] << 4 | d[3]])


def bcd_magn(v):             # S.YYYYY
    s = 0x10 if v < 0 else 0
    f = int(round(abs(v) * 100000)) % 100000
    d = [f // 10000, (f // 1000) % 10, (f // 100) % 10, (f // 10) % 10, f % 10]
    return bytes([s | d[0], d[1] << 4 | d[2], d[3] << 4 | d[4]])


def bcd_quat(v):             # SX.YYYYYY
    s = 0x10 if v < 0 else 0
    v = abs(v)
    i = int(v)
    f = int(round((v - i) * 1000000)) % 1000000
    d = [(f // 10 ** k) % 10 for k in range(5, -1, -1)]
    return bytes([s | i, d[0] << 4 | d[1], d[2] << 4 | d[3], d[4] << 4 | d[5]])


def frame(cmd, data=b""):
    body = bytes([4 + len(data), 0x00, cmd]) + data   # len = len+addr+cmd+data+chk
    return b"\x77" + body + bytes([sum(body) & 0xFF])


def synth_frame(t):
    pitch = 20 * math.sin(t * 0.5)
    roll = 10 * math.sin(t * 0.3)
    yaw = (t * 10) % 360
    ax, ay, az = 0.01 * math.sin(t), -0.02, 1.0
    gx, gy, gz = 5 * math.cos(t), 0.5, -1.5
    mx, my, mz = 0.15962, 0.16391, -0.41458
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    q0, q1, q2, q3 = cy, 0.0, 0.0, sy
    data = (bcd_angle(pitch) + bcd_angle(roll) + bcd_angle(yaw)
            + bcd_accel(ax) + bcd_accel(ay) + bcd_accel(az)
            + bcd_angle(gx) + bcd_angle(gy) + bcd_angle(gz)
            + bcd_magn(mx) + bcd_magn(my) + bcd_magn(mz)
            + bcd_quat(q0) + bcd_quat(q1) + bcd_quat(q2) + bcd_quat(q3))
    return frame(0x59, data)


def load_log(path):
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        content = re.sub(r"[^0-9a-fA-F]", "", f.read())
    # 77 38 00 59 + 52 байта данных + checksum = 57 байт = 114 hex-символов
    packets = [bytes.fromhex(p) for p in re.findall(r"77380059[0-9a-fA-F]{106}", content)]
    good = [p for p in packets if sum(p[1:56]) & 0xFF == p[56]]
    print(f"в логе кадров: {len(packets)}, с верной суммой: {len(good)}")
    return good or packets


def handle_cmd(ser, cmd, state, t):
    c = cmd[3]
    if c == 0x0C:
        state["rate"] = RATE_CODES.get(cmd[4], 10)
        ser.write(frame(0x8C, b"\x00"))
        print(f"cmd 0x0C: rate -> {state['rate']} Hz")
    elif c == 0x56:
        ser.write(frame(0x56, b"\x00"))
        print(f"cmd 0x56: type -> {cmd[4]}")
    elif c == 0x0A:
        ser.write(frame(0x8A, b"\x00"))
        print("cmd 0x0A: saved")
    elif c == 0x59:
        ser.write(synth_frame(t))
    elif c == 0x04:
        ser.write(frame(0x84, bcd_angle(1.0) + bcd_angle(2.0) + bcd_angle(90.0)))
    elif c == 0x1F:
        ser.write(frame(0x1F, b"\x00"))
    else:
        print(f"cmd 0x{c:02X}: ignored")


def run():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/tmp/ttyEmulator")
    ap.add_argument("--rate", type=int, default=10, help="Гц автовыдачи (0 = только по запросу)")
    ap.add_argument("--log", help="hex-дамп реального датчика для проигрывания")
    ap.add_argument("--noise", action="store_true", help="добавлять мусор между кадрами")
    a = ap.parse_args()

    try:
        ser = serial.Serial(a.port, 115200, timeout=0)
    except Exception as e:
        print(f"{e}\nЗапустите socat: socat -d -d PTY,link=/tmp/ttyEmulator,raw,echo=0 PTY,link=/tmp/ttyROS,raw,echo=0")
        return
    print(f"порт {a.port} открыт")

    packets = load_log(a.log) if a.log else None
    state = {"rate": a.rate}
    rx = b""
    idx = 0
    t0 = time.time()
    next_tx = t0
    n = 0
    while True:
        t = time.time() - t0
        rx += ser.read(64)
        while len(rx) >= 4 and rx[0] == 0x77 and len(rx) >= rx[1] + 1:
            ln = rx[1] + 1
            handle_cmd(ser, rx[:ln], state, t)
            rx = rx[ln:]
        if rx and rx[0] != 0x77:
            rx = rx[1:]

        if state["rate"] > 0 and time.time() >= next_tx:
            next_tx += 1.0 / state["rate"]
            if packets:
                pkt = packets[idx % len(packets)]
                idx += 1
            else:
                pkt = synth_frame(t)
            if a.noise and n % 7 == 0:
                ser.write(b"\x00\xff\x77\x12")
            ser.write(pkt)
            n += 1
            if n % 50 == 0:
                print(f"отправлено кадров: {n}")
        time.sleep(0.001)


if __name__ == "__main__":
    run()
