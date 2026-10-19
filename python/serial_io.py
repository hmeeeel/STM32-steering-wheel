"""Общее для консольных утилит: порт, сборка строк, клавиатура, отправка импульса серво."""
from __future__ import annotations

import errno
import os
import select
import sys

import serial

from protocol import encode_servo

try:
    import termios
    import tty
except ImportError:                      # не POSIX - клавиатура отключится
    termios = tty = None

SERVO_HEARTBEAT_S = 0.25                 # повторять команду, даже если импульс не менялся


class LineAssembler:
    """Собирает строки из потока байт. Всё до первого перевода строки (обрезанный хвост) отбрасывается."""

    MAX_LINE = 160

    def __init__(self) -> None:
        self._buf = bytearray()
        self._synced = False
        self._overflow = False

    def feed(self, data: bytes) -> list[bytes]:
        lines: list[bytes] = []
        for byte in data:
            if byte == 0x0A:
                if self._synced:
                    if self._overflow:
                        lines.append(b"")            # слишком длинная строка -> будет посчитана битой
                    elif self._buf:
                        lines.append(bytes(self._buf))
                self._buf.clear()
                self._overflow = False
                self._synced = True
            elif self._synced:
                if len(self._buf) < self.MAX_LINE:
                    self._buf.append(byte)
                else:
                    self._overflow = True
        return lines


class Keyboard:
    """Неблокирующее чтение клавиш (cbreak). Ctrl+C продолжает работать."""

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = bool(enabled and termios is not None and sys.stdin.isatty())
        self._old = None

    def __enter__(self) -> "Keyboard":
        if self.enabled:
            fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        return self

    def __exit__(self, *exc) -> bool:
        if self._old is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old)
        return False

    def poll(self) -> str:
        if not self.enabled:
            return ""
        fd = sys.stdin.fileno()
        keys = ""
        while select.select([fd], [], [], 0)[0]:
            data = os.read(fd, 64)
            if not data:
                break
            keys += data.decode("utf-8", errors="ignore")
        return keys


class ServoLink:
    """Отправка импульса на STM32: при изменении и раз в SERVO_HEARTBEAT_S."""

    def __init__(self, ser: serial.Serial) -> None:
        self._ser = ser
        self._last: int | None = None
        self._last_time = 0.0
        self.tx_errors = 0

    def send_if_needed(self, pulse_us: int, now: float) -> None:
        if pulse_us == self._last and now - self._last_time < SERVO_HEARTBEAT_S:
            return
        try:
            self._ser.write(encode_servo(pulse_us))
        except (serial.SerialTimeoutException, serial.SerialException):
            self.tx_errors += 1
            return
        self._last, self._last_time = pulse_us, now


def explain_open_error(port: str, exc: Exception) -> str:
    code = getattr(exc, "errno", None)
    text = str(exc)
    if code == errno.ENOENT:
        return f"Порт {port} не найден. Подключите Nucleo и проверьте: ls /dev/ttyACM* /dev/ttyUSB*"
    if code in (errno.EACCES, errno.EPERM):
        return (f"Нет прав на {port}. Fedora: sudo usermod -aG dialout $USER, затем выйдите из сессии и зайдите снова "
                f"(на один раз можно: sudo chmod a+rw {port}).")
    if code in (errno.EBUSY, errno.EAGAIN) or "lock" in text.lower():
        return f"Порт {port} занят. Закройте minicom/miniterm/консоль CubeIDE: порт нельзя открыть двумя программами."
    return f"Не удалось открыть {port}: {text}"


def open_port(port: str, baud: int) -> serial.Serial | None:
    """Открывает порт монопольно. При ошибке печатает понятное сообщение и возвращает None."""
    try:
        return serial.Serial(port, baud, timeout=0.02, write_timeout=0.1, exclusive=True)
    except (serial.SerialException, OSError, ValueError) as exc:
        print(explain_open_error(port, exc), file=sys.stderr)
        return None
