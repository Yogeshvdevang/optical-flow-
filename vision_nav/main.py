import argparse
import time

from pymavlink import mavutil

from vision_nav import config
from vision_nav.gps_emulation import build_gps_input, ned_to_lla
from vision_nav.io_mavlink import open_mavlink, read_messages, send_gps_input
from vision_nav.processing import NavState, update_state, update_state_from_velocity
from vision_nav.visualization import LivePlot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MTF-01 optical flow navigation")
    parser.add_argument("--port", default=config.SERIAL.port)
    parser.add_argument("--baud", type=int, default=config.SERIAL.baud)
    parser.add_argument("--gps", action="store_true", default=config.GPS.enabled)
    parser.add_argument(
        "--no-heartbeat",
        action="store_true",
        help="Skip waiting for a MAVLink heartbeat",
    )
    parser.add_argument(
        "--mavlink1",
        action="store_true",
        help="Force MAVLink1 framing",
    )
    parser.add_argument(
        "--print",
        action="store_true",
        help="Print decoded readings to the console",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    mav = open_mavlink(
        args.port,
        args.baud,
        wait_heartbeat=not args.no_heartbeat,
        mavlink20=not args.mavlink1,
    )

    plot = LivePlot(config.PLOT.window)
    state = NavState()
    last_time_s = time.time()
    last_flow = (0.0, 0.0, 0)
    last_tof_m = 0.0

    for msg in read_messages(mav, blocking=False):
        msg_type = msg.get_type()
        now_s = time.time()
        dt_s = now_s - last_time_s
        last_time_s = now_s

        if msg_type == "DISTANCE_SENSOR":
            last_tof_m = msg.current_distance / 100.0
            tof_strength = msg.signal_quality if hasattr(msg, "signal_quality") else 0
            plot.update(
                msg.current_distance,
                tof_strength,
                0.0,
                last_flow[0],
                last_flow[1],
                last_flow[2],
            )

        elif msg_type == "OPTICAL_FLOW_RAD":
            flow_x = msg.integrated_x
            flow_y = msg.integrated_y
            quality = msg.quality
            last_flow = (
                flow_x * config.FLOW.plot_scale,
                flow_y * config.FLOW.plot_scale,
                quality,
            )

            update_state(
                state,
                flow_x,
                flow_y,
                last_tof_m,
                dt_s,
                config.FLOW.alpha,
                quality,
                config.FLOW.min_quality,
            )

            plot.update(
                last_tof_m * 100.0,
                0.0,
                0.0,
                last_flow[0],
                last_flow[1],
                last_flow[2],
            )

            if args.gps:
                lat, lon, alt = ned_to_lla(
                    state.x,
                    state.y,
                    -last_tof_m,
                    config.GPS.base_lat,
                    config.GPS.base_lon,
                    config.GPS.base_alt_m,
                )
                msg_out = build_gps_input(
                    mav,
                    lat,
                    lon,
                    alt,
                    state.vx,
                    state.vy,
                    0.0,
                )
                send_gps_input(mav, msg_out)

            if args.print:
                print(
                    "tof_cm={:>5.1f} flow_rad_x={:>8.5f} flow_rad_y={:>8.5f} "
                    "quality={:>3} vx={:>6.3f} vy={:>6.3f}".format(
                        last_tof_m * 100.0,
                        flow_x,
                        flow_y,
                        quality,
                        state.vx,
                        state.vy,
                    )
                )

        elif msg_type == "OPTICAL_FLOW":
            flow_vx = msg.flow_comp_m_x
            flow_vy = msg.flow_comp_m_y
            quality = msg.quality
            height_m = msg.ground_distance if msg.ground_distance > 0 else last_tof_m

            update_state_from_velocity(
                state,
                flow_vx,
                flow_vy,
                dt_s,
                config.FLOW.alpha,
                quality,
                config.FLOW.min_quality,
            )

            last_flow = (
                flow_vx * config.FLOW.plot_scale,
                flow_vy * config.FLOW.plot_scale,
                quality,
            )

            plot.update(
                height_m * 100.0,
                0.0,
                0.0,
                last_flow[0],
                last_flow[1],
                last_flow[2],
            )

            if args.print:
                print(
                    "tof_cm={:>5.1f} flow_vx={:>6.3f} flow_vy={:>6.3f} "
                    "quality={:>3} vx={:>6.3f} vy={:>6.3f}".format(
                        height_m * 100.0,
                        flow_vx,
                        flow_vy,
                        quality,
                        state.vx,
                        state.vy,
                    )
                )

        elif msg_type in ("BAD_DATA", "UNKNOWN"):
            continue

        else:
            # Ignore other MAVLink messages.
            continue


if __name__ == "__main__":
    main()
