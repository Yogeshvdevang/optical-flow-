from collections import deque
import time
from typing import Deque, Dict, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch


class LivePlot:
    def __init__(
        self,
        window: int,
        show_panel: bool = False,
        show_title: bool = False,
        show_help: bool = True,
    ) -> None:
        self.window = window
        self.show_panel = show_panel
        self.show_title = show_title
        self.show_help = show_help
        self.tof_dist: Deque[float] = deque(maxlen=window)
        self.tof_strength: Deque[float] = deque(maxlen=window)
        self.tof_precision: Deque[float] = deque(maxlen=window)
        self.flow_x: Deque[float] = deque(maxlen=window)
        self.flow_y: Deque[float] = deque(maxlen=window)
        self.flow_q: Deque[float] = deque(maxlen=window)
        self.timestamps: Deque[float] = deque(maxlen=window)
        self.rate_hz = 0.0
        self.tof_min_mm = 10.0
        self.tof_max_mm = 8000.0

        plt.ion()
        self.fig = plt.figure(figsize=(9, 5), facecolor="#C9CCD1")
        self.ax = self.fig.add_axes([0.06, 0.12, 0.90, 0.82])

        self.colors: Dict[str, str] = {
            "tof_dist": "#E86E6E",
            "tof_strength": "#F5B766",
            "tof_precision": "#F4E36B",
            "flow_x": "#6ECF7D",
            "flow_y": "#68B4F0",
            "flow_q": "#6B67F0",
        }

        (self.l1,) = self.ax.plot([], [], color=self.colors["tof_dist"], label="TOF_distance", linewidth=1.6)
        (self.l2,) = self.ax.plot([], [], color=self.colors["tof_strength"], label="TOF_strength", linewidth=1.6)
        (self.l3,) = self.ax.plot([], [], color=self.colors["tof_precision"], label="TOF_precision", linewidth=1.6)
        (self.l4,) = self.ax.plot([], [], color=self.colors["flow_x"], label="Flow_vel_x", linewidth=1.2)
        (self.l5,) = self.ax.plot([], [], color=self.colors["flow_y"], label="Flow_vel_y", linewidth=1.2)
        (self.l6,) = self.ax.plot([], [], color=self.colors["flow_q"], label="Flow_quality", linewidth=1.4)
        self.tof_min_line = self.ax.axhline(
            self.tof_min_mm, color="#7A7D83", linewidth=1.0, linestyle="--", alpha=0.6, label="TOF_min_mm"
        )
        self.tof_max_line = self.ax.axhline(
            self.tof_max_mm, color="#7A7D83", linewidth=1.0, linestyle="--", alpha=0.6, label="TOF_max_mm"
        )

        self.ax.legend(
            loc="lower left",
            frameon=True,
            facecolor="#D0D3D8",
            edgecolor="#AEB2B8",
            fontsize=8.5,
        )
        if self.show_title:
            self.ax.set_title("MTF-01 Live Optical Flow + TOF Data", color="#2C2E33")
        self.ax.grid(True, color="#9FA3A9", alpha=0.7)
        self.ax.tick_params(colors="#6A6D72")
        self.ax.yaxis.tick_right()
        self.ax.yaxis.set_label_position("right")
        for spine in self.ax.spines.values():
            spine.set_color("#B4B7BC")
            spine.set_linewidth(1.0)

        self.value_text: Dict[str, object] = {}
        if self.show_panel:
            self.panel_ax = self.fig.add_axes([0.03, 0.12, 0.18, 0.78])
            self.panel_ax.axis("off")
            panel_bg = FancyBboxPatch(
                (0, 0),
                1,
                1,
                boxstyle="round,pad=0.02,rounding_size=0.04",
                transform=self.panel_ax.transAxes,
                facecolor="#C2C5CA",
                edgecolor="#A9ACB1",
                linewidth=1.5,
            )
            self.panel_ax.add_patch(panel_bg)
            self.panel_ax.text(0.08, 0.95, "DATA", color="#2C2E33", fontsize=11, fontweight="bold")
            self.value_boxes: Dict[str, Tuple[float, float]] = {
                "DISTANCE": (0.08, 0.83),
                "STRENGTH": (0.08, 0.73),
                "PRECISION": (0.08, 0.63),
                "STATUS": (0.08, 0.53),
                "FLOW_VEL_X": (0.08, 0.43),
                "FLOW_VEL_Y": (0.08, 0.33),
                "FLOW_QUAL": (0.08, 0.23),
                "FLOW_STA": (0.08, 0.13),
            }
            for key, (x, y) in self.value_boxes.items():
                box = FancyBboxPatch(
                    (x, y),
                    0.84,
                    0.075,
                    boxstyle="round,pad=0.01,rounding_size=0.02",
                    transform=self.panel_ax.transAxes,
                    facecolor="#D9DBDF",
                    edgecolor="#B1B4B9",
                    linewidth=1.0,
                )
                self.panel_ax.add_patch(box)
                self.panel_ax.text(x + 0.04, y + 0.038, key, va="center", fontsize=8.5, color="#4B4E53")
                self.value_text[key] = self.panel_ax.text(
                    x + 0.62,
                    y + 0.038,
                    "0",
                    va="center",
                    ha="right",
                    fontsize=10,
                    color="#2C2E33",
                    fontweight="bold",
                )

        if self.show_help:
            self.ax.annotate(
                "Select the correct COM PORT",
                xy=(0.99, 0.95),
                xycoords="axes fraction",
                xytext=(0.60, 0.95),
                textcoords="axes fraction",
                color="#D25555",
                fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#D25555", lw=1.5),
            )
            self.ax.annotate(
                "Click to connect",
                xy=(0.99, 0.88),
                xycoords="axes fraction",
                xytext=(0.70, 0.88),
                textcoords="axes fraction",
                color="#D25555",
                fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#D25555", lw=1.5),
            )
            self.ax.annotate(
                "Setup button",
                xy=(0.99, 0.14),
                xycoords="axes fraction",
                xytext=(0.78, 0.14),
                textcoords="axes fraction",
                color="#D25555",
                fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#D25555", lw=1.5),
            )

        # Gradient background similar to the reference UI.
        self._set_gradient_background()

    def update(
        self,
        tof_dist_mm: float,
        tof_strength: float,
        tof_precision: float,
        flow_x: float,
        flow_y: float,
        flow_q: float,
        dist_status: Optional[int] = None,
        flow_status: Optional[int] = None,
    ) -> None:
        now_s = time.perf_counter()
        if self.timestamps:
            dt_s = now_s - self.timestamps[-1]
            if dt_s > 0:
                inst_hz = 1.0 / dt_s
                self.rate_hz = inst_hz if self.rate_hz == 0.0 else (0.9 * self.rate_hz + 0.1 * inst_hz)
        self.timestamps.append(now_s)
        self.tof_dist.append(tof_dist_mm)
        self.tof_strength.append(tof_strength)
        self.tof_precision.append(tof_precision)
        self.flow_x.append(flow_x)
        self.flow_y.append(flow_y)
        self.flow_q.append(flow_q)

        x = np.array(self.timestamps, dtype=float) - self.timestamps[-1]
        self.l1.set_data(x, self.tof_dist)
        self.l2.set_data(x, self.tof_strength)
        self.l3.set_data(x, self.tof_precision)
        self.l4.set_data(x, self.flow_x)
        self.l5.set_data(x, self.flow_y)
        self.l6.set_data(x, self.flow_q)

        window_s = self.window / self.rate_hz if self.rate_hz > 0.0 else 0.0
        x_max = x[-1] if len(x) else 0.0
        if window_s > 0.0:
            x_min = -window_s
            self.ax.set_xlim(x_min, max(x_max, 0.0))
        else:
            x_min = min(x) if len(x) else 0.0
            self.ax.set_xlim(min(x_min, 0.0), max(x_max, 1e-3))
        values = (
            list(self.tof_dist)
            + list(self.tof_strength)
            + list(self.tof_precision)
            + list(self.flow_x)
            + list(self.flow_y)
            + list(self.flow_q)
        )
        if values:
            min_val = min(values)
            max_val = max(values)
            pad = (max_val - min_val) * 0.1
            if pad == 0:
                pad = 1.0
            self.ax.set_ylim(min_val - pad, max_val + pad)
        if self.show_title:
            self.ax.set_title(
                "MTF-01 Live Optical Flow + TOF Data | {:.1f} Hz | Window ~{:.1f}s ({} samples)".format(
                    self.rate_hz,
                    window_s,
                    len(self.tof_dist),
                ),
                color="#2C2E33",
            )

        if self.value_text:
            self.value_text["DISTANCE"].set_text(f"{self.tof_dist[-1] if self.tof_dist else 0:.0f}")
            self.value_text["STRENGTH"].set_text(f"{self.tof_strength[-1] if self.tof_strength else 0:.0f}")
            self.value_text["PRECISION"].set_text(f"{self.tof_precision[-1] if self.tof_precision else 0:.0f}")
            if dist_status is None:
                dist_status = 1
            self.value_text["STATUS"].set_text(str(dist_status))
            self.value_text["FLOW_VEL_X"].set_text(f"{self.flow_x[-1] if self.flow_x else 0:.0f}")
            self.value_text["FLOW_VEL_Y"].set_text(f"{self.flow_y[-1] if self.flow_y else 0:.0f}")
            self.value_text["FLOW_QUAL"].set_text(f"{self.flow_q[-1] if self.flow_q else 0:.0f}")
            if flow_status is None:
                flow_status = 1
            self.value_text["FLOW_STA"].set_text(str(flow_status))

        plt.pause(0.001)

    def _set_gradient_background(self) -> None:
        x = np.linspace(-1.0, 1.0, 300)
        y = np.linspace(-1.0, 1.0, 200)
        xx, yy = np.meshgrid(x, y)
        radial = np.sqrt(xx * xx + yy * yy)
        img = 1.0 - np.clip(radial, 0.0, 1.0)
        base = np.array([0.86, 0.87, 0.89])
        tint = np.array([0.80, 0.82, 0.85])
        gradient = base + (img[..., None] * (tint - base))
        self.ax.imshow(
            gradient,
            extent=[0.0, 1.0, 0.0, 1.0],
            transform=self.ax.transAxes,
            zorder=0,
            aspect="auto",
        )
        self.ax.set_facecolor((0, 0, 0, 0))
