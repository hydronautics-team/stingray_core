#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import re

def run():
    try:
        ser = serial.Serial('/tmp/ttyEmulator', 115200, timeout=1)
        print("порт открыт")
    except Exception as e:
        print(f"{e} socat или порт")
        return

    while True: 
        try:
            with open('/home/ingibirka/Desktop/data.log', 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
                packets = re.findall(r'7738[0-9a-fA-F]{110}', content)
                
                if not packets:
                    print("нет пакетов")
                    break

                for p in packets:
                    try:
                        ser.write(bytes.fromhex(p))
                        print(25)
                        time.sleep(0.1) 
                    except:
                        continue
                
        except Exception as e:
            print(e)
            time.sleep(1)

if __name__ == "__main__":
    run()
