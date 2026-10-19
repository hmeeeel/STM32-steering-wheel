"""Кинематика автомобиля: велосипедная модель, опорная точка - центр задней оси.

    x' = v cos(psi),  y' = v sin(psi),  psi' = (v / L) tan(delta)

delta > 0 - колёса влево (против часовой), v > 0 - вперёд, v < 0 - назад.
Для постоянных delta и v на шаге dt интегрирование точное (дуга окружности), а не метод Эйлера.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleConfig:
    wheelbase_m: float = 2.635       # колёсная база L. Параметр - уточните по документации своей модификации
    speed_slow_mps: float = 0.5
    speed_fast_mps: float = 1.5


class BicycleModel:
    def __init__(self, cfg: VehicleConfig | None = None) -> None:
        self.cfg = cfg or VehicleConfig()
        self.reset()

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0   # psi, рад

    @property
    def heading_deg(self) -> float:
        return (math.degrees(self.heading) + 180.0) % 360.0 - 180.0

    def turning_radius_m(self, wheel_deg: float) -> float | None:
        """Радиус дуги заднего моста со знаком (+ влево). None - прямая."""
        t = math.tan(math.radians(wheel_deg))
        if abs(t) < 1e-9:
            return None
        return self.cfg.wheelbase_m / t

    def step(self, wheel_deg: float, speed_mps: float, dt_s: float) -> None:
        if dt_s <= 0.0 or speed_mps == 0.0:
            return
        dist = speed_mps * dt_s
        tan_d = math.tan(math.radians(wheel_deg))
        dpsi = dist * tan_d / self.cfg.wheelbase_m
        if abs(dpsi) < 1e-9:                                  # практически прямо
            self.x += dist * math.cos(self.heading)
            self.y += dist * math.sin(self.heading)
            return
        radius = self.cfg.wheelbase_m / tan_d                 # со знаком
        self.x += radius * (math.sin(self.heading + dpsi) - math.sin(self.heading))
        self.y -= radius * (math.cos(self.heading + dpsi) - math.cos(self.heading))
        self.heading += dpsi


class DriveControl:
    """Передача и скорость. Сейчас - с клавиатуры, позже источник можно заменить на кнопки STM32."""

    GEARS = ("N", "D", "R")

    def __init__(self, cfg: VehicleConfig) -> None:
        self.cfg = cfg
        self.gear = "N"
        self.fast = False

    def set_gear(self, gear: str) -> None:
        if gear in self.GEARS:
            self.gear = gear

    def toggle_fast(self) -> None:
        self.fast = not self.fast

    @property
    def speed_mps(self) -> float:
        v = self.cfg.speed_fast_mps if self.fast else self.cfg.speed_slow_mps
        return {"N": 0.0, "D": v, "R": -v}[self.gear]
