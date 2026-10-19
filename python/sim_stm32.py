#!/usr/bin/env python3
"""Эмулятор STM32 для проверки Python без платы (Linux): создаёт виртуальный порт и шлёт кадры $JS.

Терминал 1:  python3 sim_stm32.py              (напечатает путь порта, например /dev/pts/5)
Терминал 2:  python3 steering_console.py --port /dev/pts/5

Сценарий: покой -> 1.5 оборота против часовой -> ещё оборот (упор) -> назад 3 оборота -> отпустили.
Эмулятор принимает команды $SV и возвращает применённый импульс в поле servo_us, как настоящая прошивка.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import time

from protocol import frame

CX, CY, RADIUS = 1972, 1983, 1800


def scenario():
    """Генератор (угол_градусов | None) для каждого кадра 50 Гц; None - стик в покое."""
    for _ in range(100):
        yield None
    angle = 0.0
    for deg_per_s, seconds in ((270.0, 2.0), (270.0, 1.6), (-270.0, 4.0), (0.0, 1.0)):
        for _ in range(int(seconds * 50)):
            angle += deg_per_s / 50.0
            yield angle
    for _ in range(150):
        yield None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--loop", action="store_true", help="повторять сценарий")
    args = parser.parse_args()

    master, slave = os.openpty()
    os.set_blocking(master, False)
    print(f"Виртуальный порт: {os.ttyname(slave)}   (Ctrl+C - выход)", flush=True)

    servo_us, seq, rx = 1500, 0, b""
    random.seed(0)
    t0 = time.monotonic()
    while True:
        for angle in scenario():
            if angle is None:
                x, y = CX + random.randint(-3, 3), CY + random.randint(-3, 3)
            else:
                x = round(CX + RADIUS * math.cos(math.radians(angle))) + random.randint(-2, 2)
                y = round(CY + RADIUS * math.sin(math.radians(angle))) + random.randint(-2, 2)
            x, y = max(0, min(4095, x)), max(0, min(4095, y))
            t_ms = int((time.monotonic() - t0) * 1000)
            try:
                os.write(master, frame(f"JS,{seq},{t_ms},{x},{y},{servo_us}"))
                rx += os.read(master, 256)
            except BlockingIOError:
                pass
            except OSError:
                pass
            while b"\n" in rx:
                line, rx = rx.split(b"\n", 1)
                text = line.decode("ascii", "replace").strip()
                if text.startswith("$SV,") and "*" in text:
                    body, _, cs = text[1:].rpartition("*")
                    try:
                        from protocol import checksum
                        if checksum(body) == int(cs, 16):
                            servo_us = max(500, min(2500, int(body[3:])))
                    except ValueError:
                        pass
            seq = (seq + 1) & 0xFFFF
            time.sleep(0.02)
        if not args.loop:
            break
    while True:                       # держим порт открытым, чтобы консоль не получила обрыв
        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
