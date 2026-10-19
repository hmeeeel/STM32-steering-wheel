#!/usr/bin/env python3
"""Консоль «руль на джойстике»: принимает сырой АЦП от STM32, считает руль и кинематику, шлёт импульс на серво.

STM32 только читает джойстик и крутит серво. Вся математика - здесь (steering.py, kinematics.py).

Запуск:   python3 steering_console.py --port /dev/ttyACM0
Клавиши:  d - вперёд | r - назад | s или пробел - стоп | f - медленно/быстро | z - обнулить позицию | q - выход
          (работают и на русской раскладке)
Порт нельзя держать открытым одновременно в этом скрипте и в minicom/консоли CubeIDE.
"""
from __future__ import annotations

import argparse
import contextlib
import shutil
import sys
import time

import serial

from kinematics import BicycleModel, DriveControl, VehicleConfig
from protocol import ParseError, Telemetry, encode_servo, parse_telemetry
from serial_io import Keyboard, LineAssembler, ServoLink, open_port
from steering import ServoConfig, ServoMapper, SteeringConfig, SteeringModel, SteeringState

RENDER_PERIOD_S = 1.0 / 25.0             # 25 строк в секунду
STALL_WARN_S = 1.0
MAX_DT_S = 0.1                           # ограничение шага кинематики при задержках

KEY_ACTIONS = {
    "d": ("gear", "D"), "в": ("gear", "D"),
    "r": ("gear", "R"), "к": ("gear", "R"),
    "s": ("gear", "N"), "ы": ("gear", "N"), " ": ("gear", "N"),
    "f": ("fast", None), "а": ("fast", None),
    "z": ("reset_pose", None), "я": ("reset_pose", None),
    "q": ("quit", None), "й": ("quit", None),
}


class Stats:
    def __init__(self) -> None:
        self.frames = 0
        self.bad = 0
        self.lost = 0
        self.mcu_resets = 0


class App:
    def __init__(self, args: argparse.Namespace, ser: serial.Serial, log) -> None:
        self.args = args
        self.ser = ser
        self.log = log
        self.stats = Stats()
        self.assembler = LineAssembler()
        self.model = SteeringModel(SteeringConfig(invert_y=args.invert_y))
        self.servo_map = ServoMapper(
            ServoConfig(center_us=args.servo_center, min_us=args.servo_min, max_us=args.servo_max,
                        invert=args.servo_invert),
            self.model.cfg.max_wheel_deg)
        self.vehicle = BicycleModel(VehicleConfig(wheelbase_m=args.wheelbase))
        self.drive = DriveControl(self.vehicle.cfg)
        self.link = ServoLink(ser)
        self.last_tel: Telemetry | None = None
        self.state: SteeringState | None = None
        self.t0_ms = 0
        self.last_rx_time = time.monotonic()
        self.next_render = 0.0
        self.in_place = sys.stdout.isatty() and not args.lines
        self.drew_once = False
        self.running = True

    # --- приём ---------------------------------------------------------------------------

    def handle_line(self, raw: bytes, now: float) -> None:
        if self.log is not None:
            self.log.write(raw.decode("ascii", errors="replace").rstrip("\r\n") + "\n")
        try:
            tel = parse_telemetry(raw)
        except ParseError:
            self.stats.bad += 1
            return
        self.last_rx_time = now
        self.on_telemetry(tel, now)

    def on_telemetry(self, tel: Telemetry, now: float) -> None:
        stats = self.stats
        dt_ms = 0
        if self.last_tel is None:
            self.t0_ms = tel.t_ms
        elif tel.t_ms < self.last_tel.t_ms:                  # время ушло назад: STM32 перезагрузилась
            stats.mcu_resets += 1
            self.model.reset()
            self.vehicle.reset()
            self.t0_ms = tel.t_ms
        else:
            dt_ms = tel.t_ms - self.last_tel.t_ms
            stats.lost += (tel.seq - (self.last_tel.seq + 1)) & 0xFFFF
        self.last_tel = tel
        stats.frames += 1

        self.state = self.model.update(tel.t_ms, tel.x, tel.y)
        if not self.state.calibrating:
            self.vehicle.step(self.state.wheel_deg, self.drive.speed_mps, min(dt_ms / 1000.0, MAX_DT_S))
        self.link.send_if_needed(self.servo_map.pulse_us(self.state.wheel_deg), now)

    def handle_keys(self, keys: str) -> None:
        for ch in keys.lower():
            action, value = KEY_ACTIONS.get(ch, (None, None))
            if action == "gear":
                self.drive.set_gear(value)
            elif action == "fast":
                self.drive.toggle_fast()
            elif action == "reset_pose":
                self.vehicle.reset()
            elif action == "quit":
                self.running = False

    # --- вывод ---------------------------------------------------------------------------

    def format_lines(self, now: float) -> tuple[str, str]:
        tel, st = self.last_tel, self.state
        if tel is None or st is None:
            return "ожидание данных от STM32 ...", ""
        t = (tel.t_ms - self.t0_ms) / 1000.0
        head = f"t={t:6.2f} c | X={tel.x:4d} Y={tel.y:4d} | "
        if st.calibrating:
            note = f"калибровка {st.calib_progress:4.0%} - не трогайте стик"
            if self.model.calib_restarts:
                note += f" (повторов: {self.model.calib_restarts})"
            return head + note, ""
        servo = f"{tel.servo_us:4d} мкс" if tel.servo_us else "ВЫКЛ (проверьте TIM3 в CubeMX)"
        line1 = f"{head}руль={st.steer_deg:+7.1f}° | колёса={st.wheel_deg:+6.2f}° | серво={servo}"

        radius = self.vehicle.turning_radius_m(st.wheel_deg)
        r_txt = "∞" if radius is None or abs(radius) > 999.0 else f"{radius:+.2f} м"
        gear = {"N": "N стоп", "D": "D вперёд", "R": "R назад"}[self.drive.gear]
        pace = "быстро" if self.drive.fast else "медленно"
        line2 = (f" ")
        if now - self.last_rx_time > STALL_WARN_S:
            line2 = f"НЕТ ДАННЫХ от STM32 уже {now - self.last_rx_time:.0f} c (кабель/порт/прошивка?)"
        return line1, line2

    def render(self, now: float) -> None:
        if now < self.next_render:
            return
        self.next_render = now + RENDER_PERIOD_S
        line1, line2 = self.format_lines(now)
        if self.in_place:
            cols = max(40, shutil.get_terminal_size().columns - 1)
            prefix = "\r\x1b[1A" if self.drew_once else ""
            sys.stdout.write(f"{prefix}{line1[:cols]}\x1b[K\n{line2[:cols]}\x1b[K")
            self.drew_once = True
        else:
            sys.stdout.write(line1 + (" | " + line2 if line2 else "") + "\n")
        sys.stdout.flush()

    # --- главный цикл --------------------------------------------------------------------

    def run(self, kb: Keyboard) -> None:
        while self.running:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            now = time.monotonic()
            for line in self.assembler.feed(chunk):
                self.handle_line(line, now)
            self.handle_keys(kb.poll())
            self.render(now)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Руль на джойстике: консоль на Python (математика и кинематика здесь).")
    p.add_argument("--port", required=True, help="последовательный порт, например /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="скорость (по умолчанию 115200)")
    p.add_argument("--log", help="сохранять принятые строки в этот файл")
    p.add_argument("--lines", action="store_true", help="печатать построчно, а не обновлять строки на месте")
    p.add_argument("--invert-y", action="store_true", help="сменить знак оси Y (если направление руля обратное)")
    p.add_argument("--servo-invert", action="store_true", help="зеркальное направление серво")
    p.add_argument("--servo-min", type=int, default=ServoConfig.min_us, help="импульс при колёсах -31.58°, мкс (из servo_test.py)")
    p.add_argument("--servo-max", type=int, default=ServoConfig.max_us, help="импульс при колёсах +31.58°, мкс (из servo_test.py)")
    p.add_argument("--servo-center", type=int, default=ServoConfig.center_us, help="импульс при колёсах 0°, мкс (из servo_test.py)")
    p.add_argument("--wheelbase", type=float, default=VehicleConfig.wheelbase_m, help="колёсная база L, м")
    p.add_argument("--no-keys", action="store_true", help="не читать клавиатуру")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        ServoConfig(center_us=args.servo_center, min_us=args.servo_min, max_us=args.servo_max)
    except ValueError as exc:
        print(f"Неверные параметры серво: {exc}", file=sys.stderr)
        return 2
    ser = open_port(args.port, args.baud)
    if ser is None:
        return 2

    log_ctx = open(args.log, "w", encoding="utf-8", buffering=1) if args.log else contextlib.nullcontext()
    code = 0
    with ser, log_ctx as log, Keyboard(not args.no_keys) as kb:
        app = App(args, ser, log)
        ser.reset_input_buffer()

        try:
            app.run(kb)
        except KeyboardInterrupt:
            pass
        except serial.SerialException as exc:
            print(f"\nПорт потерян: {exc}", file=sys.stderr)
            code = 1
        finally:
            with contextlib.suppress(serial.SerialException):
                ser.write(encode_servo(args.servo_center))  # серво в центр при выходе
            s = app.stats
    return code


if __name__ == "__main__":
    sys.exit(main())
