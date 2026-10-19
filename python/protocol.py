"""Протокол STM32 <-> ПК (текстовый, NMEA-подобный). Зеркало firmware/Core/Src/protocol.c.

STM32 -> ПК:  $JS,<seq>,<t_ms>,<x>,<y>,<servo_us>*HH\\r\\n
ПК -> STM32:  $SV,<pulse_us>*HH\\r\\n
HH = XOR всех символов между '$' и '*' (две hex-цифры).
"""
from __future__ import annotations

import re
from dataclasses import dataclass

ADC_MAX = 4095


class ParseError(ValueError):
    """Строка битая, обрезанная или чужая - её просто пропускают."""


@dataclass(frozen=True)
class Telemetry:
    seq: int        # счётчик кадров 0..65535
    t_ms: int       # HAL_GetTick() на STM32
    x: int          # сырой АЦП, 0..4095
    y: int
    servo_us: int   # импульс, реально выставленный в TIM3 (0 = серво выключено)


def checksum(body: str) -> int:
    cs = 0
    for byte in body.encode("ascii"):
        cs ^= byte
    return cs


def frame(body: str) -> bytes:
    return f"${body}*{checksum(body):02X}\r\n".encode("ascii")


def encode_servo(pulse_us: int) -> bytes:
    return frame(f"SV,{int(pulse_us)}")


_TELEMETRY_RE = re.compile(
    r"^\$(JS,(\d{1,5}),(\d{1,10}),(\d{1,4}),(\d{1,4}),(\d{1,5}))\*([0-9A-Fa-f]{2})$"
)


def parse_telemetry(raw: bytes) -> Telemetry:
    """Разбирает одну строку. Любая проблема -> ParseError."""
    try:
        text = raw.decode("ascii").strip()
    except UnicodeDecodeError as exc:
        raise ParseError("не ASCII") from exc
    match = _TELEMETRY_RE.match(text)
    if match is None:
        raise ParseError("не кадр JS")
    body = match.group(1)
    if checksum(body) != int(match.group(7), 16):
        raise ParseError("контрольная сумма")
    seq, t_ms, x, y, servo_us = (int(match.group(i)) for i in range(2, 7))
    if seq > 0xFFFF or t_ms > 0xFFFFFFFF or x > ADC_MAX or y > ADC_MAX or servo_us > 0xFFFF:
        raise ParseError("значение вне диапазона")
    return Telemetry(seq, t_ms, x, y, servo_us)
