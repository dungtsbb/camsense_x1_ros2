"""Giải mã giao thức UART của lidar Camsense X1.

Gói 36 byte, little-endian:
    55 AA 03 08 | speed(2) | góc đầu(2) | 8 x [khoảng cách int16 + quality u8] | góc cuối(2) | checksum(2)

- speed / 3840 = tốc độ quay (vòng/giây)
- góc (độ)   = (raw - 0xA000) / 64; raw nhỏ hơn 0xA000 là gói không hợp lệ
- khoảng cách: mm, int16 có dấu; quality = 0 nghĩa là không có phản xạ,
  khi đó trường khoảng cách không mang nghĩa (thường là 0x8000)
- checksum: fold 15 bit trên 17 word little-endian đầu gói (không phải CRC16)

Module này thuần Python, không phụ thuộc ROS — test ở test/test_protocol.py.
"""

import struct
from dataclasses import dataclass

HEADER = bytes([0x55, 0xAA, 0x03, 0x08])
PACKET_SIZE = 36
SAMPLES_PER_PACKET = 8
ANGLE_RAW_MIN = 0xA000


@dataclass
class Packet:
    speed_hz: float
    start_deg: float
    end_deg: float
    # list 8 phần tử (distance_mm, quality); điểm không phản xạ đã lọc thành None
    points: list


def checksum(buf: bytes) -> int:
    """Fold 15 bit trên 34 byte đầu của gói (17 word little-endian)."""
    chk = 0
    for i in range(0, PACKET_SIZE - 2, 2):
        chk = (chk << 1) + (buf[i] | (buf[i + 1] << 8))
    return ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF


def parse_packet(buf: bytes):
    """Giải mã 36 byte đã biết là bắt đầu bằng HEADER.

    Trả về Packet, hoặc None nếu checksum/góc không hợp lệ.
    """
    stored = struct.unpack_from('<H', buf, 34)[0]
    if checksum(buf) != stored:
        return None

    speed_raw, start_raw = struct.unpack_from('<HH', buf, 4)
    end_raw = struct.unpack_from('<H', buf, 32)[0]
    if start_raw < ANGLE_RAW_MIN or end_raw < ANGLE_RAW_MIN:
        return None

    points = []
    for i in range(SAMPLES_PER_PACKET):
        dist, quality = struct.unpack_from('<hB', buf, 8 + 3 * i)
        points.append((dist, quality) if quality > 0 and dist > 0 else None)

    return Packet(
        speed_hz=speed_raw / 3840.0,
        start_deg=(start_raw - ANGLE_RAW_MIN) / 64.0,
        end_deg=(end_raw - ANGLE_RAW_MIN) / 64.0,
        points=points,
    )


def feed(buffer: bytearray):
    """Tách các gói hợp lệ ra khỏi buffer byte thô từ serial.

    Trả về list Packet; cắt khỏi buffer phần đã xử lý, giữ lại phần đuôi
    chưa đủ một gói. Checksum sai thì trượt 1 byte (có thể là header giả).
    """
    packets = []
    i = 0
    n = len(buffer)
    while i + PACKET_SIZE <= n:
        if buffer[i:i + 4] != HEADER:
            i += 1
            continue
        pkt = parse_packet(bytes(buffer[i:i + PACKET_SIZE]))
        if pkt is None:
            i += 1
            continue
        packets.append(pkt)
        i += PACKET_SIZE
    del buffer[:i]
    return packets
