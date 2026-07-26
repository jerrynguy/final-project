#!/usr/bin/env python3
"""
TCP bridge STM32 <-> Laptop
- Port 9002: laptop gửi lệnh M → STM32
- Port 9003: STM32 gửi ENC → laptop
"""
import socket, serial, threading, sys

STM32_PORT = '/dev/ttyUSB0'
STM32_BAUD = 115200
CMD_PORT   = 9002
ENC_PORT   = 9003

stm32 = serial.Serial(STM32_PORT, STM32_BAUD, timeout=0.1)
print(f"✅ STM32: {STM32_PORT}")

# Đọc "READY" từ STM32
import time; time.sleep(0.5)
while stm32.in_waiting:
    print(f"STM32: {stm32.readline().decode().strip()}")

def make_server(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', port))
    s.listen(1)
    return s

cmd_srv = make_server(CMD_PORT)
enc_srv = make_server(ENC_PORT)
print(f"Chờ laptop kết nối cổng {CMD_PORT} và {ENC_PORT}...")

cmd_conn, _ = cmd_srv.accept()
enc_conn, _ = enc_srv.accept()
print("✅ Laptop connected")

def recv_cmd():
    """Laptop → STM32"""
    buf = ""
    while True:
        try:
            data = cmd_conn.recv(256).decode()
            if not data:
                break
            buf += data
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if line:
                    stm32.write((line + '\n').encode())
                    print(f"→ STM32: {line}")
        except Exception as e:
            print(f"CMD err: {e}"); break

def send_enc():
    """STM32 → Laptop"""
    while True:
        try:
            line = stm32.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"← STM32: {line}")
                enc_conn.sendall((line + '\n').encode())
        except Exception as e:
            print(f"ENC err: {e}"); break

threading.Thread(target=recv_cmd, daemon=True).start()
threading.Thread(target=send_enc, daemon=True).start()

try:
    threading.Event().wait()
except KeyboardInterrupt:
    stm32.write(b'M 0 0\n')  # Dừng motor khi thoát
    print("Dừng")
