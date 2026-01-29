from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class NavState:
    vx: float = 0.0
    vy: float = 0.0
    x: float = 0.0
    y: float = 0.0


def lowpass(prev: float, new: float, alpha: float) -> float:
    return prev + alpha * (new - prev)


def flow_to_velocity(
    flow_x: float,
    flow_y: float,
    height_m: float,
    dt_s: float,
) -> Tuple[float, float]:
    if dt_s <= 0.0:
        return 0.0, 0.0
    # flow_x/flow_y are integrated angles (rad). Divide by dt to get rad/s.
    vx = (flow_x / dt_s) * height_m
    vy = (flow_y / dt_s) * height_m
    return vx, vy


def integrate_position(state: NavState, dt_s: float) -> None:
    if dt_s <= 0.0:
        return
    state.x += state.vx * dt_s
    state.y += state.vy * dt_s


def update_state(
    state: NavState,
    flow_x: float,
    flow_y: float,
    height_m: float,
    dt_s: float,
    alpha: float,
    quality: Optional[int],
    min_quality: int,
) -> None:
    if quality is not None and quality < min_quality:
        integrate_position(state, dt_s)
        return

    vx, vy = flow_to_velocity(flow_x, flow_y, height_m, dt_s)
    state.vx = lowpass(state.vx, vx, alpha)
    state.vy = lowpass(state.vy, vy, alpha)
    integrate_position(state, dt_s)


def update_state_from_velocity(
    state: NavState,
    vx: float,
    vy: float,
    dt_s: float,
    alpha: float,
    quality: Optional[int],
    min_quality: int,
) -> None:
    if quality is not None and quality < min_quality:
        integrate_position(state, dt_s)
        return

    state.vx = lowpass(state.vx, vx, alpha)
    state.vy = lowpass(state.vy, vy, alpha)
    integrate_position(state, dt_s)
