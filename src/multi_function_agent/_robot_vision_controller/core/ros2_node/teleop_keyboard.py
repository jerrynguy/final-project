#!/usr/bin/env python3

import time
import sys

CMD_QUEUE_FILE = '/tmp/ros2_cmd_queue.txt'

LINEAR_SPEED = 0.15   # m/s
ANGULAR_SPEED = 1.0   # rad/s


def send_cmd(linear: float, angular: float):
    try:
        with open(CMD_QUEUE_FILE, 'w') as f:
            f.write(f"{linear},{angular}\n")
    except Exception as e:
        print(f"Loi ghi queue file: {e}")


def main():
    print("Teleop ban phim - dieu khien thu cong")
    print("Lenh: w=tien s=lui a=xoay-trai d=xoay-phai x=dung q=thoat")
    print(f"LINEAR_SPEED={LINEAR_SPEED} m/s, ANGULAR_SPEED={ANGULAR_SPEED} rad/s\n")

    try:
        while True:
            cmd = input("Nhap lenh: ").strip().lower()
            if cmd == "w":
                send_cmd(LINEAR_SPEED, 0.0)
            elif cmd == "s":
                send_cmd(-LINEAR_SPEED, 0.0)
            elif cmd == "a":
                send_cmd(0.0, ANGULAR_SPEED)
            elif cmd == "d":
                send_cmd(0.0, -ANGULAR_SPEED)
            elif cmd == "x":
                send_cmd(0.0, 0.0)
            elif cmd == "q":
                send_cmd(0.0, 0.0)
                break
            else:
                print("Lenh khong hop le.")
    except KeyboardInterrupt:
        pass
    finally:
        send_cmd(0.0, 0.0)
        print("\nDa dung robot.")


if __name__ == "__main__":
    main()