from dataclasses import dataclass


@dataclass(frozen=True)
class SerialConfig:
    port: str = "COM11"
    baud: int = 115200


@dataclass(frozen=True)
class PlotConfig:
    window: int = 300


@dataclass(frozen=True)
class FlowConfig:
    # Scale factor to make flow values visible in the plot.
    plot_scale: float = 100.0
    # Minimum quality to accept optical flow for velocity estimation.
    min_quality: int = 60
    # Low-pass filter alpha for velocity and position.
    alpha: float = 0.2


@dataclass(frozen=True)
class GpsConfig:
    # Enable to send GPS_INPUT messages.
    enabled: bool = False
    # Use a local NED origin at (0, 0, 0). Replace with real lat/lon if needed.
    base_lat: float = 0.0
    base_lon: float = 0.0
    base_alt_m: float = 0.0


SERIAL = SerialConfig()
PLOT = PlotConfig()
FLOW = FlowConfig()
GPS = GpsConfig()
