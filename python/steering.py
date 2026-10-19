"""Руль: калибровка -> нормировка -> фильтр -> atan2 -> unwrap -> накопление -> угол колёс -> импульс серво.

Чистая математика без ввода-вывода: её можно гонять в тестах и подключать к любому источнику данных.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap180(delta_deg: float) -> float:
    """Приводит приращение угла к диапазону [-180, +180)."""
    return (delta_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class SteeringConfig:
    adc_half_scale: float = 2048.0   # нормировка: x = (X - cx) / 2048
    calib_time_s: float = 0.5        # усреднение при старте
    calib_max_spread: int = 60       # разброс отсчётов при калибровке, иначе «стик не отпущен»
    filter_k: float = 0.5            # fx += k * (x - fx)
    r_take: float = 0.35             # стик «взят»
    r_release: float = 0.20          # стик «отпущен»
    max_steer_deg: float = 540.0     # упор руля, +-1.5 оборота
    steer_ratio: float = 17.1        # передаточное число рулевого механизма
    invert_y: bool = False           # если направление вращения по проводке обратное

    @property
    def max_wheel_deg(self) -> float:
        return self.max_steer_deg / self.steer_ratio    # 31.578...


@dataclass(frozen=True)
class SteeringState:
    calibrating: bool
    calib_progress: float   # 0..1
    taken: bool
    stick_deg: float        # угол стика atan2, -180..+180
    radius: float           # отклонение стика (только для отсечки шума)
    steer_deg: float        # накопленный угол руля, -540..+540
    wheel_deg: float        # угол передних колёс = steer / 17.1


class SteeringModel:
    def __init__(self, cfg: SteeringConfig | None = None) -> None:
        self.cfg = cfg or SteeringConfig()
        self.reset()

    def reset(self) -> None:
        """Полный сброс (старт или перезагрузка STM32): руль в 0, повторная калибровка."""
        self._calib: list[tuple[int, int]] = []
        self._calib_ms = 0.0
        self._last_t: int | None = None
        self.center: tuple[float, float] | None = None
        self.calib_restarts = 0
        self._fx = self._fy = 0.0
        self._taken = False
        self._a_prev = 0.0
        self._angle = 0.0
        self._radius = 0.0
        self._steer = 0.0

    @property
    def calibrated(self) -> bool:
        return self.center is not None

    def set_center(self, cx: float, cy: float) -> None:
        """Задать центр вручную (например, для анализа записанных данных без паузы покоя в начале)."""
        self.center = (float(cx), float(cy))
        self._calib.clear()

    def update(self, t_ms: int, x_raw: int, y_raw: int) -> SteeringState:
        dt_ms = 0 if self._last_t is None else (t_ms - self._last_t) & 0xFFFFFFFF
        self._last_t = t_ms
        if self.center is None:
            self._calibrate(dt_ms, x_raw, y_raw)
        else:
            self._track(x_raw, y_raw)
        return self.state()

    def state(self) -> SteeringState:
        cfg = self.cfg
        progress = 1.0 if self.calibrated else min(1.0, self._calib_ms / (cfg.calib_time_s * 1000.0))
        return SteeringState(
            calibrating=not self.calibrated,
            calib_progress=progress,
            taken=self._taken,
            stick_deg=self._angle,
            radius=self._radius,
            steer_deg=self._steer,
            wheel_deg=self._steer / cfg.steer_ratio,
        )

    # --- внутренние шаги -------------------------------------------------------------------

    def _calibrate(self, dt_ms: int, x: int, y: int) -> None:
        cfg = self.cfg
        self._calib.append((x, y))
        self._calib_ms += dt_ms
        if self._calib_ms < cfg.calib_time_s * 1000.0:
            return
        xs, ys = zip(*self._calib)
        if max(xs) - min(xs) > cfg.calib_max_spread or max(ys) - min(ys) > cfg.calib_max_spread:
            self._calib.clear()             # стик трогали - начинаем заново
            self._calib_ms = 0.0
            self.calib_restarts += 1
            return
        self.center = (sum(xs) / len(xs), sum(ys) / len(ys))

    def _track(self, x_raw: int, y_raw: int) -> None:
        cfg = self.cfg
        cx, cy = self.center  # type: ignore[misc]
        x = (x_raw - cx) / cfg.adc_half_scale
        y = (y_raw - cy) / cfg.adc_half_scale
        if cfg.invert_y:
            y = -y
        self._fx += cfg.filter_k * (x - self._fx)
        self._fy += cfg.filter_k * (y - self._fy)

        self._radius = math.hypot(self._fx, self._fy)
        self._angle = math.degrees(math.atan2(self._fy, self._fx))

        if not self._taken:
            if self._radius > cfg.r_take:
                self._taken = True
                self._a_prev = self._angle          # запоминаем угол - скачка нет
            return
        if self._radius < cfg.r_release:
            self._taken = False                     # отпущен: руль остаётся на месте
            return
        delta = wrap180(self._angle - self._a_prev)
        self._steer = max(-cfg.max_steer_deg, min(cfg.max_steer_deg, self._steer + delta))
        self._a_prev = self._angle                  # на упоре тоже обновляем: обратный ход без мёртвой зоны


@dataclass(frozen=True)
class ServoConfig:
    """Импульсы серво в мкс. Значения min/center/max получают измерением (servo_test.py)."""

    center_us: int = 1500          # колёса прямо
    min_us: int = 1000             # колёса на -max_wheel_deg
    max_us: int = 2000             # колёса на +max_wheel_deg
    invert: bool = False           # если качалка повёрнута «зеркально»

    def __post_init__(self) -> None:
        if not self.min_us < self.center_us < self.max_us:
            raise ValueError(f"нужно min < center < max, получено {self.min_us} / {self.center_us} / {self.max_us}")


class ServoMapper:
    """Переводит угол колёс в угол сервопривода (строго от 0 до 64 градусов) и в микросекунды."""

    def __init__(self, cfg: ServoConfig, wheel_full_deg: float) -> None:
        self.cfg = cfg
        self.wheel_full_deg = wheel_full_deg  # Максимальный угол колёс модели

    def pulse_us(self, wheel_deg: float) -> int:
        cfg = self.cfg

        # 1. Нормализуем угол колёс в диапазон от -1.0 до +1.0
        k = max(-1.0, min(1.0, wheel_deg / self.wheel_full_deg))
        if cfg.invert:
            k = -k

        # 2. Переводим нормализованный коэффициент [-1, 1] в диапазон углов сервы: от 0 до 64 градусов.
        # Центр (k=0) будет соответствовать 32 градусам.
        servo_min_angle = 0.0
        servo_max_angle = 64.0
        servo_center_angle = (servo_min_angle + servo_max_angle) / 2.0 # 32.0°
        servo_span_angle = (servo_max_angle - servo_min_angle) / 2.0   # 32.0°

        target_servo_deg = servo_center_angle + k * servo_span_angle
        # Теперь target_servo_deg гарантированно лежит в диапазоне от 0.0 до 64.0 градусов!

        # 3. Переводим полученные градусы сервы (0..64°) в микросекунды (min_us..max_us)
        deg_fraction = (target_servo_deg - servo_min_angle) / (servo_max_angle - servo_min_angle)
        us = cfg.min_us + deg_fraction * (cfg.max_us - cfg.min_us)

        return int(round(us))
