"""Format checks for the /sensors/* outputs, in plain Python (no ROS imports), so they can be
unit-tested. check_sensors.py feeds them real messages; anything with the same attribute names
works (the tests use SimpleNamespace).

Each check returns a list of problems; an empty list means the message looks right.
"""
import struct
import zlib
from typing import List

EXPECTED_W, EXPECTED_H = 800, 600
EXPECTED_FX = 400.0  # 800 / (2 tan 45 deg)
LIDAR_FIELDS = ("x", "y", "z", "intensity")
FLOAT32 = 7          # sensor_msgs/PointField.FLOAT32


def check_image(msg, encoding: str = "rgb8") -> List[str]:
    problems = []
    if msg.encoding != encoding:
        problems.append(f"encoding is {msg.encoding!r}, expected {encoding!r}")
    if (msg.width, msg.height) != (EXPECTED_W, EXPECTED_H):
        problems.append(f"size is {msg.width}x{msg.height}, expected {EXPECTED_W}x{EXPECTED_H}")
    channels = {"rgb8": 3, "bgr8": 3, "bgra8": 4, "rgba8": 4}.get(msg.encoding)
    if channels and msg.step < msg.width * channels:
        problems.append(f"step {msg.step} is less than width x {channels}")
    if len(msg.data) != msg.height * msg.step:
        problems.append(f"data is {len(msg.data)} bytes, expected height x step = {msg.height * msg.step}")
    if not msg.header.frame_id:
        problems.append("header.frame_id is empty")
    return problems


def check_camera_info(msg) -> List[str]:
    problems = []
    if (msg.width, msg.height) != (EXPECTED_W, EXPECTED_H):
        problems.append(f"size is {msg.width}x{msg.height}, expected {EXPECTED_W}x{EXPECTED_H}")
    k = list(msg.k)
    if len(k) != 9 or abs(k[0] - EXPECTED_FX) > 1.0 or abs(k[4] - EXPECTED_FX) > 1.0:
        problems.append(f"K has fx={k[0] if k else None}, fy={k[4] if len(k) > 4 else None}; expected ~{EXPECTED_FX}")
    return problems


def check_lidar(msg) -> List[str]:
    problems = []
    fields = {f.name: f for f in msg.fields}
    for name in LIDAR_FIELDS:
        if name not in fields:
            problems.append(f"missing field {name!r}")
        elif fields[name].datatype != FLOAT32:
            problems.append(f"field {name!r} is not float32")
    if msg.width * msg.height == 0:
        problems.append("cloud is empty")
    if len(msg.data) != msg.row_step * msg.height or msg.row_step != msg.point_step * msg.width:
        problems.append("data size doesn't match point_step x width x height")
    if not msg.header.frame_id:
        problems.append("header.frame_id is empty")
    return problems


def check_rate(count: int, seconds: float, minimum_hz: float) -> List[str]:
    hz = count / seconds if seconds > 0 else 0.0
    return [] if hz >= minimum_hz else [f"{hz:.1f} Hz, expected at least {minimum_hz:.0f} Hz"]


def write_png(path: str, rgb: bytes, width: int, height: int) -> None:
    """Save rgb8 pixels as a PNG with no extra libraries."""
    rows = b"".join(b"\x00" + rgb[r * width * 3:(r + 1) * width * 3] for r in range(height))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(rows, 6))
                + chunk(b"IEND", b""))
