"""Sensor telemetry framing and payload decoding."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable

FRAME_HEADER = b"\xEB\x90"
CAN_SENSOR_ID = 0x621
MSG_SENSOR_STATE = 0x05
SENSOR_PAYLOAD_SIZE = 196
SENSOR_STRUCT = struct.Struct("<I26f4B42H")


@dataclass(frozen=True)
class ImuSample:
    acceleration: tuple[float, float, float]
    angular_rate: tuple[float, float, float]
    euler: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class FootSample:
    phase: int
    point_count: int
    total: int
    pressure: tuple[int, ...]


@dataclass(frozen=True)
class SensorSample:
    timestamp_ms: int
    imu1: ImuSample
    imu2: ImuSample
    left: FootSample
    right: FootSample


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_frame(message_id: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload is too large for the one-byte length field")
    body = FRAME_HEADER + bytes((message_id & 0xFF, len(payload))) + payload
    return body + struct.pack("<H", crc16_ccitt(body))


class StreamParser:
    """Reassemble application messages from consecutive CAN data fields."""

    def __init__(self, max_buffer: int = 4096):
        self.buffer = bytearray()
        self.max_buffer = max_buffer
        self.frames_ok = 0
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes | bytearray | Iterable[int]) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        messages: list[tuple[int, bytes]] = []

        while True:
            start = self.buffer.find(FRAME_HEADER)
            if start < 0:
                keep = 1 if self.buffer.endswith(FRAME_HEADER[:1]) else 0
                discard = len(self.buffer) - keep
                if discard > 0:
                    del self.buffer[:discard]
                    self.discarded_bytes += discard
                break
            if start > 0:
                del self.buffer[:start]
                self.discarded_bytes += start
            if len(self.buffer) < 4:
                break

            payload_length = self.buffer[3]
            frame_length = 4 + payload_length + 2
            if len(self.buffer) < frame_length:
                break

            candidate = bytes(self.buffer[:frame_length])
            received_crc = struct.unpack_from("<H", candidate, frame_length - 2)[0]
            calculated_crc = crc16_ccitt(candidate[:-2])
            if received_crc != calculated_crc:
                self.crc_errors += 1
                del self.buffer[0]
                self.discarded_bytes += 1
                continue

            message_id = candidate[2]
            payload = candidate[4:-2]
            del self.buffer[:frame_length]
            self.frames_ok += 1
            messages.append((message_id, payload))

        if len(self.buffer) > self.max_buffer:
            excess = len(self.buffer) - self.max_buffer
            del self.buffer[:excess]
            self.discarded_bytes += excess
        return messages


def decode_sensor_payload(payload: bytes) -> SensorSample:
    if len(payload) != SENSOR_PAYLOAD_SIZE:
        raise ValueError(f"sensor payload must be {SENSOR_PAYLOAD_SIZE} bytes, got {len(payload)}")

    value = SENSOR_STRUCT.unpack(payload)
    timestamp = value[0]
    f = value[1:27]
    flags = value[27:31]
    pressure = value[31:]

    imu1 = ImuSample(tuple(f[0:3]), tuple(f[3:6]), tuple(f[6:9]), tuple(f[9:13]))
    imu2 = ImuSample(tuple(f[13:16]), tuple(f[16:19]), tuple(f[19:22]), tuple(f[22:26]))
    left = FootSample(flags[0], flags[2], pressure[0], tuple(pressure[2:22]))
    right = FootSample(flags[1], flags[3], pressure[1], tuple(pressure[22:42]))
    return SensorSample(timestamp, imu1, imu2, left, right)


def encode_sensor_payload(sample: SensorSample) -> bytes:
    floats = (
        *sample.imu1.acceleration,
        *sample.imu1.angular_rate,
        *sample.imu1.euler,
        *sample.imu1.quaternion,
        *sample.imu2.acceleration,
        *sample.imu2.angular_rate,
        *sample.imu2.euler,
        *sample.imu2.quaternion,
    )
    return SENSOR_STRUCT.pack(
        sample.timestamp_ms,
        *floats,
        sample.left.phase,
        sample.right.phase,
        sample.left.point_count,
        sample.right.point_count,
        sample.left.total,
        sample.right.total,
        *sample.left.pressure,
        *sample.right.pressure,
    )
