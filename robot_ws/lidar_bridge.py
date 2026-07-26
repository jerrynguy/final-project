#!/usr/bin/env python3
"""Raw serial → TCP bridge cho RPLIDAR A1M8"""
import socket, serial, sys

SERIAL_PORT = '/dev/ttyUSB1'
BAUD        = 115200
TCP_PORT    = 9001

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
print(f"✅ LiDAR: {SERIAL_PORT}")

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('0.0.0.0', TCP_PORT))
srv.listen(1)
print(f"Chờ laptop kết nối cổng {TCP_PORT}...")

conn, addr = srv.accept()
print(f"✅ Laptop: {addr}")

try:
    while True:
        data = ser.read(512)
        if data:
            conn.sendall(data)
except (BrokenPipeError, ConnectionResetError, KeyboardInterrupt):
    print("Dừng")
finally:
    conn.close()
    ser.close()
