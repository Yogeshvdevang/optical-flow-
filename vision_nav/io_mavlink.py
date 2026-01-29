import time
from typing import Generator

from pymavlink import mavutil


def open_mavlink(
    port: str,
    baud: int,
    wait_heartbeat: bool = True,
    mavlink20: bool = True,
) -> mavutil.mavlink_connection:
    mav = mavutil.mavlink_connection(port, baud=baud, mavlink20=mavlink20)
    if wait_heartbeat:
        print("Waiting for heartbeat...")
        mav.wait_heartbeat()
        print("Connected!")
    else:
        print("Connected (heartbeat disabled).")
    return mav


def read_messages(
    mav: mavutil.mavlink_connection,
    blocking: bool = False,
    sleep_s: float = 0.01,
) -> Generator[object, None, None]:
    while True:
        msg = mav.recv_match(blocking=blocking)
        if msg is None:
            time.sleep(sleep_s)
            continue
        yield msg


def send_gps_input(mav: mavutil.mavlink_connection, msg: object) -> None:
    mav.mav.send(msg)
