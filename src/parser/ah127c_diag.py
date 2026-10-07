#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Диагностика датчика LC-AH127C (Bewis) через RS485/RS232.
"""
import argparse
import datetime
import sys
import time

try:
    import serial
except ImportError:
    print("Нужен pyserial:  pip install pyserial   или   sudo apt install python3-serial")
    sys.exit(1)

BAUDS = [9600, 115200, 19200, 38400, 57600, 4800, 2400]
BAUD_CODES = {2400: 0, 4800: 1, 9600: 2, 19200: 3, 115200: 4, 38400: 5, 57600: 6}
RATE_CODES = {0: 0, 5: 1, 10: 2, 20: 3, 25: 4, 50: 5}

LOG = open(f"ah127c_diag_{datetime.datetime.now():%Y%m%d_%H%M%S}.log", "w", encoding="utf-8")

def out(s=""):
    print(s)
    LOG.write(s + "\n")
    LOG.flush()

def hx(b):
    return " ".join(f"{x:02X}" for x in b)

def frame(cmd, data=b""):
    body = bytes([4 + len(data), 0x00, cmd]) + data
    return b"\x77" + body + bytes([sum(body) & 0xFF])

def bcd_angle(b):
    v = (b[0] & 0x0F) * 100 + (b[1] >> 4) * 10 + (b[1] & 0x0F) + (b[2] >> 4) * 0.1 + (b[2] & 0x0F) * 0.01
    return -v if b[0] & 0xF0 else v

def find_frames(buf):
    found = []
    i = 0
    while i + 4 <= len(buf):
        if buf[i] == 0x77 and 4 <= buf[i + 1] <= 0x40 and i + buf[i + 1] + 1 <= len(buf):
            ln = buf[i + 1] + 1
            f = buf[i:i + ln]
            if sum(f[1:-1]) & 0xFF == f[-1]:
                found.append(f)
                i += ln
                continue
        i += 1
    return found

def describe(f):
    cmd = f[3]
    if cmd == 0x59 and len(f) == 57:
        return (f"полный кадр 0x59: pitch={bcd_angle(f[4:7]):.2f} roll={bcd_angle(f[7:10]):.2f} "
                f"yaw={bcd_angle(f[10:13]):.2f}")
    if cmd == 0x84 and len(f) == 14:
        return (f"три угла 0x84: pitch={bcd_angle(f[4:7]):.2f} roll={bcd_angle(f[7:10]):.2f} "
                f"yaw={bcd_angle(f[10:13]):.2f}")
    if cmd == 0x1F:
        return f"адрес датчика = 0x{f[4]:02X}"
    return f"ответ 0x{cmd:02X} ({len(f)} байт)"

def open_port(port, baud):
    return serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0)

def read_for(ser, sec):
    end = time.time() + sec
    buf = b""
    while time.time() < end:
        buf += ser.read(256)
        time.sleep(0.005)
    return buf

def query(ser, cmd_bytes, wait=0.4):
    ser.reset_input_buffer()
    ser.write(cmd_bytes)
    ser.flush()
    rx = read_for(ser, wait)
    if rx.startswith(cmd_bytes):
        rx = rx[len(cmd_bytes):]
    return rx

def test_baud(port, baud, listen_sec):
    out(f"\n=== {baud} бод ===")
    try:
        ser = open_port(port, baud)
    except Exception as e:
        out(f"не открыть порт: {e}")
        return None
    result = {"baud": baud, "auto": 0, "answers": 0, "addr": None}
    try:
        ser.reset_input_buffer()
        buf = read_for(ser, listen_sec)
        out(f"слушал {listen_sec} с: пришло {len(buf)} байт")
        if buf:
            out("  первые байты: " + hx(buf[:40]))
        frs = find_frames(buf)
        result["auto"] = len(frs)
        if frs:
            out(f"  корректных кадров: {len(frs)}  -> {describe(frs[-1])}")

        for name, cmd in [("адрес", frame(0x1F)), ("три угла", frame(0x04)), ("полный кадр", frame(0x59))]:
            rx = query(ser, cmd)
            frs = find_frames(rx)
            out(f"запрос {name} [{hx(cmd)}]: {len(rx)} байт" + (f", ответ: {describe(frs[0])}" if frs else ""))
            if rx and not frs:
                out("  сырые: " + hx(rx[:40]))
            if frs:
                result["answers"] += 1
                if frs[0][3] == 0x1F:
                    result["addr"] = frs[0][4]
    finally:
        ser.close()
    return result

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--listen", type=float, help="только слушать N секунд")
    ap.add_argument("--set-baud", type=int, choices=sorted(BAUD_CODES))
    ap.add_argument("--set-rate", type=int, choices=sorted(RATE_CODES))
    a = ap.parse_args()

    out(f"AH127C diag, порт {a.port}, {datetime.datetime.now():%Y-%m-%d %H:%M:%S}")

    if a.listen:
        baud = a.baud or 9600
        ser = open_port(a.port, baud)
        buf = read_for(ser, a.listen)
        ser.close()
        out(f"{baud} бод, {a.listen} с: {len(buf)} байт")
        out(hx(buf[:400]))
        frs = find_frames(buf)
        out(f"корректных кадров: {len(frs)}")
        for f in frs[:5]:
            out("  " + describe(f))
        return

    bauds = [a.baud] if a.baud else BAUDS
    results = [r for r in (test_baud(a.port, b, 1.0) for b in bauds) if r]
    ok = [r for r in results if r["auto"] or r["answers"]]

    out("\n================ ИТОГ ================")
    if not ok:
        out("Датчик НЕ ответил ни на одной скорости.")
        return
    best = max(ok, key=lambda r: (r["answers"], r["auto"]))
    out(f"Датчик работает на скорости {best['baud']} бод."
        + (f" Адрес 0x{best['addr']:02X}." if best["addr"] is not None else ""))
    if best["auto"]:
        out(f"Автовыдача включена ({best['auto']} кадров за 1 с).")
    else:
        out("Автовыдача выключена (режим по запросу) - нода включит её сама.")

    if a.set_rate is not None or a.set_baud:
        ser = open_port(a.port, best["baud"])
        if a.set_rate is not None:
            rx = query(ser, frame(0x56, b"\x05"))
            rx = query(ser, frame(0x0C, bytes([RATE_CODES[a.set_rate]])))
            rx = query(ser, frame(0x0A))
        if a.set_baud and a.set_baud != best["baud"]:
            ser.write(frame(0x0B, bytes([BAUD_CODES[a.set_baud]])))
            ser.flush()
            time.sleep(0.3)
            ser.close()
            r = test_baud(a.port, a.set_baud, 1.0)
        else:
            ser.close()
    out(f"Лог: {LOG.name}")

if __name__ == "__main__":
    main()
