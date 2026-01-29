import argparse
import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial

from vision_nav.visualization import LivePlot

MICOLINK_MSG_HEAD = 0xEF
MICOLINK_MAX_PAYLOAD_LEN = 64
MICOLINK_MSG_ID_RANGE_SENSOR = 0x51
MICOLINK_RANGE_PAYLOAD_LEN = 0x14


@dataclass
class RangeFlowData:
    time_ms: int
    distance_mm: int
    strength: int
    precision: int
    distance_status: int
    flow_vel_x: int
    flow_vel_y: int
    flow_quality: int
    flow_status: int


class MicoLinkDecoder:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.status = 0
        self.dev_id = 0
        self.sys_id = 0
        self.msg_id = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray()
        self.checksum = 0
        self.raw = bytearray()

    def checksum_ok(self, raw_bytes: bytearray) -> bool:
        return (sum(raw_bytes[:-1]) & 0xFF) == raw_bytes[-1]

    def parse_byte(self, byte: int) -> Optional[RangeFlowData]:
        if self.status == 0:
            if byte == MICOLINK_MSG_HEAD:
                self.raw = bytearray([byte])
                self.status = 1

        elif self.status == 1:
            self.dev_id = byte
            self.raw.append(byte)
            self.status = 2

        elif self.status == 2:
            self.sys_id = byte
            self.raw.append(byte)
            self.status = 3

        elif self.status == 3:
            self.msg_id = byte
            self.raw.append(byte)
            self.status = 4

        elif self.status == 4:
            self.seq = byte
            self.raw.append(byte)
            self.status = 5

        elif self.status == 5:
            self.length = byte
            self.raw.append(byte)

            if self.length == 0:
                self.status = 7
            elif self.length > MICOLINK_MAX_PAYLOAD_LEN:
                self.reset()
            else:
                self.payload = bytearray()
                self.status = 6

        elif self.status == 6:
            self.payload.append(byte)
            self.raw.append(byte)

            if len(self.payload) == self.length:
                self.status = 7

        elif self.status == 7:
            self.raw.append(byte)

            if self.checksum_ok(self.raw):
                result = self.decode_message()
                self.reset()
                return result
            self.reset()

        return None

    def decode_message(self) -> Optional[RangeFlowData]:
        if self.msg_id != MICOLINK_MSG_ID_RANGE_SENSOR:
            return None
        if self.length != MICOLINK_RANGE_PAYLOAD_LEN or len(self.payload) != MICOLINK_RANGE_PAYLOAD_LEN:
            return None

        fmt = "<I I B B B B h h B B H".replace(" ", "")
        unpacked = struct.unpack(fmt, self.payload)
        return RangeFlowData(
            time_ms=unpacked[0],
            distance_mm=unpacked[1],
            strength=unpacked[2],
            precision=unpacked[3],
            distance_status=unpacked[4],
            flow_vel_x=unpacked[6],
            flow_vel_y=unpacked[7],
            flow_quality=unpacked[8],
            flow_status=unpacked[9],
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Read Micolink frames from MTF-01")
    parser.add_argument("--port", default="COM11")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run, 0 for infinite")
    parser.add_argument("--window", type=int, default=300, help="Plot window size in samples")
    parser.add_argument("--no-plot", action="store_true", help="Disable live plotting")
    parser.add_argument("--print", action="store_true", help="Print decoded readings to the console")
    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
    print(f"Listening on {args.port} @ {args.baud}...")
    decoder = MicoLinkDecoder()
    plot = None if args.no_plot else LivePlot(args.window)
    start = time.time()

    try:
        while True:
            if args.duration and time.time() - start > args.duration:
                break
            data = ser.read(256)
            for byte in data:
                parsed = decoder.parse_byte(byte)
                if parsed is None:
                    continue
                if plot is not None:
                    plot.update(
                        float(parsed.distance_mm),
                        parsed.strength,
                        parsed.precision,
                        parsed.flow_vel_x,
                        parsed.flow_vel_y,
                        parsed.flow_quality,
                        dist_status=parsed.distance_status,
                        flow_status=parsed.flow_status,
                    )
                if args.print:
                    print(
                        "time_ms={:>8} distance_mm={:>5} strength={:>3} precision={:>3} "
                        "status={} flow_vel_x={:>4} flow_vel_y={:>4} flow_quality={:>3} flow_status={}".format(
                            parsed.time_ms,
                            parsed.distance_mm,
                            parsed.strength,
                            parsed.precision,
                            parsed.distance_status,
                            parsed.flow_vel_x,
                            parsed.flow_vel_y,
                            parsed.flow_quality,
                            parsed.flow_status,
                        )
                    )
    finally:
        ser.close()


if __name__ == "__main__":
    main()
