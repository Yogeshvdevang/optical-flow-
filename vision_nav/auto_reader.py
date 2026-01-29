import argparse
import struct
import time
from typing import Optional, Tuple

import serial
from pymavlink import mavutil

from vision_nav import config
from vision_nav.visualization import LivePlot


MICOLINK_STX = 0xEF
MICOLINK_DEV_ID = 0x0F
MICOLINK_SYS_ID = 0x00
MICOLINK_MSG_ID = 0x51
MICOLINK_PAYLOAD_LEN = 0x14

MSP_HEADER = b"$X<"


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def parse_micolink_frame(frame: bytes) -> Optional[Tuple[int, int, int, int, int, int, int, int, int, int, int]]:
    if len(frame) < 7:
        return None
    if (
        frame[0] != MICOLINK_STX
        or frame[1] != MICOLINK_DEV_ID
        or frame[2] != MICOLINK_SYS_ID
        or frame[3] != MICOLINK_MSG_ID
    ):
        return None
    payload_len = frame[5]
    if payload_len != MICOLINK_PAYLOAD_LEN:
        return None
    checksum = frame[-1]
    if (sum(frame[:-1]) & 0xFF) != checksum:
        return None
    if len(frame) != 6 + MICOLINK_PAYLOAD_LEN + 1:
        return None
    payload = frame[6:-1]
    return struct.unpack("<IIBBBBh hBBH".replace(" ", ""), payload)


def find_micolink_frame(buffer: bytearray) -> Optional[bytes]:
    while True:
        if len(buffer) < 7:
            return None
        if buffer[0] != MICOLINK_STX:
            buffer.pop(0)
            continue
        if len(buffer) < 7:
            return None
        payload_len = buffer[5]
        frame_len = 6 + payload_len + 1
        if len(buffer) < frame_len:
            return None
        frame = bytes(buffer[:frame_len])
        del buffer[:frame_len]
        return frame


def find_msp_frame(buffer: bytearray) -> Optional[Tuple[int, int, bytes]]:
    while True:
        idx = buffer.find(MSP_HEADER)
        if idx == -1:
            buffer.clear()
            return None
        if idx > 0:
            del buffer[:idx]
        if len(buffer) < 9:
            return None
        flags = buffer[3]
        func = int.from_bytes(buffer[4:6], "little")
        size = int.from_bytes(buffer[6:8], "little")
        total_len = 3 + 1 + 2 + 2 + size + 1
        if len(buffer) < total_len:
            return None
        payload = bytes(buffer[8 : 8 + size])
        checksum = buffer[8 + size]
        crc_data = bytes([flags]) + buffer[4:8] + payload
        if crc8_dvb_s2(crc_data) != checksum:
            buffer.pop(0)
            continue
        del buffer[:total_len]
        return flags, func, payload


def detect_protocol(buffer: bytes, mav_parser: mavutil.mavlink.MAVLink) -> Optional[str]:
    if len(buffer) >= 7:
        temp = bytearray(buffer)
        frame = find_micolink_frame(temp)
        if frame and parse_micolink_frame(frame):
            return "micolink"
    if MSP_HEADER in buffer:
        temp = bytearray(buffer)
        if find_msp_frame(temp):
            return "msp"
    for byte in buffer:
        msg = mav_parser.parse_char(bytes([byte]))
        if msg is not None and msg.get_type() != "BAD_DATA":
            return "mavlink"
    return None


def print_micolink(parsed: Tuple[int, int, int, int, int, int, int, int, int, int, int]) -> None:
    (
        sys_time_ms,
        distance_mm,
        dist_strength,
        dist_precision,
        dist_status,
        _reserved0,
        flow_vel_x,
        flow_vel_y,
        flow_quality,
        flow_status,
        _reserved1,
    ) = parsed
    print(
        "time_ms={:>8} distance_mm={:>5} strength={:>3} precision={:>3} "
        "status={} flow_vel_x={:>4} flow_vel_y={:>4} flow_quality={:>3} flow_status={}".format(
            sys_time_ms,
            distance_mm,
            dist_strength,
            dist_precision,
            dist_status,
            flow_vel_x,
            flow_vel_y,
            flow_quality,
            flow_status,
        )
    )


def print_mavlink(msg: object) -> None:
    msg_type = msg.get_type()
    if msg_type == "DISTANCE_SENSOR":
        print(
            "DISTANCE_SENSOR distance_cm={} quality={}".format(
                msg.current_distance, getattr(msg, "signal_quality", 0)
            )
        )
    elif msg_type == "OPTICAL_FLOW_RAD":
        print(
            "OPTICAL_FLOW_RAD flow_x={:.6f} flow_y={:.6f} quality={}".format(
                msg.integrated_x,
                msg.integrated_y,
                msg.quality,
            )
        )
    elif msg_type == "OPTICAL_FLOW":
        print(
            "OPTICAL_FLOW flow_vx={:.6f} flow_vy={:.6f} quality={}".format(
                msg.flow_comp_m_x,
                msg.flow_comp_m_y,
                msg.quality,
            )
        )


def decode_msp_sensor(
    func: int, payload: bytes
) -> Optional[Tuple[Optional[float], Optional[int], Optional[int], Optional[int], Optional[float], Optional[float], Optional[int], Optional[int]]]:
    if func == 0x1F01 and len(payload) == 5:
        strength = payload[0]
        distance_mm = int.from_bytes(payload[1:3], "little")
        precision = payload[3]
        status = payload[4]
        return (
            float(distance_mm),
            strength,
            precision,
            status,
            None,
            None,
            None,
            None,
        )
    if func == 0x1F02 and len(payload) == 9:
        flow_x = int.from_bytes(payload[0:2], "little", signed=True)
        flow_y = int.from_bytes(payload[2:4], "little", signed=True)
        flow_quality = payload[4]
        flow_status = payload[5]
        return (
            None,
            None,
            None,
            None,
            float(flow_x),
            float(flow_y),
            flow_quality,
            flow_status,
        )
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Auto-detect Micolink/MAVLink/MSP streams")
    parser.add_argument("--port", default="COM11")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run, 0 for infinite")
    parser.add_argument("--detect-timeout", type=float, default=3.0)
    parser.add_argument("--no-plot", action="store_true", help="Disable live plot UI")
    parser.add_argument(
        "--retry-timeout",
        type=float,
        default=0.0,
        help="Seconds to keep retrying the serial port (0 = infinite)",
    )
    args = parser.parse_args()

    plot = None if args.no_plot else LivePlot(config.PLOT.window)

    retry_start = time.time()
    while True:
        try:
            ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
            break
        except serial.SerialException as exc:
            elapsed = time.time() - retry_start
            if args.retry_timeout and elapsed > args.retry_timeout:
                raise
            print(
                "Serial open failed for {}: {}. Retrying...".format(
                    args.port, exc
                )
            )
            time.sleep(1.0)

    print(f"Listening on {args.port} @ {args.baud}...")

    buffer = bytearray()
    mav_parser = mavutil.mavlink.MAVLink(None)
    mav_parser.robust_parsing = True
    mode: Optional[str] = None

    tof_dist_mm = 0.0
    tof_strength = 0.0
    tof_precision = 0.0
    dist_status = None
    flow_x = 0.0
    flow_y = 0.0
    flow_q = 0.0
    flow_status = None

    detect_start = time.time()
    start = time.time()

    try:
        while True:
            if args.duration and time.time() - start > args.duration:
                break
            data = ser.read(256)
            if data:
                buffer.extend(data)

            if mode is None:
                if time.time() - detect_start > args.detect_timeout:
                    print("Detect timeout: no valid frames found yet.")
                    detect_start = time.time()
                found = detect_protocol(buffer, mav_parser)
                if found:
                    mode = found
                    print(f"Detected protocol: {mode}")
                else:
                    continue

            if mode == "micolink":
                frame = find_micolink_frame(buffer)
                if not frame:
                    continue
                parsed = parse_micolink_frame(frame)
                if parsed:
                    print_micolink(parsed)
                    (
                        _sys_time_ms,
                        distance_mm,
                        dist_strength,
                        dist_precision,
                        dist_status,
                        _reserved0,
                        flow_vel_x,
                        flow_vel_y,
                        flow_quality,
                        flow_status,
                        _reserved1,
                    ) = parsed
                    tof_dist_mm = float(distance_mm)
                    tof_strength = dist_strength
                    tof_precision = dist_precision
                    flow_x = flow_vel_x
                    flow_y = flow_vel_y
                    flow_q = flow_quality
                    if plot:
                        plot.update(
                            tof_dist_mm,
                            tof_strength,
                            tof_precision,
                            flow_x,
                            flow_y,
                            flow_q,
                            dist_status,
                            flow_status,
                        )
            elif mode == "msp":
                frame = find_msp_frame(buffer)
                if not frame:
                    continue
                flags, func, payload = frame
                print(
                    "MSP frame func=0x{:04X} flags=0x{:02X} payload_len={} payload={}".format(
                        func,
                        flags,
                        len(payload),
                        payload.hex(),
                    )
                )
                decoded = decode_msp_sensor(func, payload)
                if decoded:
                    (
                        d_mm,
                        d_strength,
                        d_precision,
                        d_status,
                        f_x,
                        f_y,
                        f_q,
                        f_status,
                    ) = decoded
                    if d_mm is not None:
                        tof_dist_mm = d_mm
                    if d_strength is not None:
                        tof_strength = d_strength
                    if d_precision is not None:
                        tof_precision = d_precision
                    if d_status is not None:
                        dist_status = d_status
                    if f_x is not None:
                        flow_x = f_x
                    if f_y is not None:
                        flow_y = f_y
                    if f_q is not None:
                        flow_q = f_q
                    if f_status is not None:
                        flow_status = f_status
                if plot:
                    plot.update(
                        tof_dist_mm,
                        tof_strength,
                        tof_precision,
                        flow_x,
                        flow_y,
                        flow_q,
                        dist_status,
                        flow_status,
                    )
            else:
                if not buffer:
                    continue
                for byte in bytes(buffer):
                    msg = mav_parser.parse_char(bytes([byte]))
                    if msg is not None and msg.get_type() != "BAD_DATA":
                        print_mavlink(msg)
                        msg_type = msg.get_type()
                        if msg_type == "DISTANCE_SENSOR":
                            tof_dist_mm = msg.current_distance * 10.0
                            tof_strength = getattr(msg, "signal_quality", 0)
                        elif msg_type == "OPTICAL_FLOW_RAD":
                            flow_x = msg.integrated_x * config.FLOW.plot_scale
                            flow_y = msg.integrated_y * config.FLOW.plot_scale
                            flow_q = msg.quality
                        elif msg_type == "OPTICAL_FLOW":
                            flow_x = msg.flow_comp_m_x * config.FLOW.plot_scale
                            flow_y = msg.flow_comp_m_y * config.FLOW.plot_scale
                            flow_q = msg.quality
                        if plot and msg_type in ("DISTANCE_SENSOR", "OPTICAL_FLOW_RAD", "OPTICAL_FLOW"):
                            plot.update(
                            tof_dist_mm,
                            tof_strength,
                            tof_precision,
                                flow_x,
                                flow_y,
                                flow_q,
                                dist_status,
                                flow_status,
                            )
                buffer.clear()
    finally:
        ser.close()


if __name__ == "__main__":
    main()
