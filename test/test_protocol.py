"""Test giải mã giao thức với các gói bắt thực tế từ phần cứng.

Nguồn gói mẫu: github.com/thijses/camsense-X1 ("a few data packets.txt").
"""

from camsense_x1.protocol import PACKET_SIZE, checksum, feed, parse_packet

# 3 gói bắt thực tế: toàn điểm trống / lẫn lộn / đủ 8 điểm hợp lệ
PKT_EMPTY = bytes.fromhex(
    '55AA0308EF4DB1C200800000800000800000800000800000800000800000800033C4D133'
)
PKT_MIXED = bytes.fromhex(
    '55AA0308EC4D6CC40080000080000080000080000080008106147E06337D0633F6C5493C'
)
PKT_FULL = bytes.fromhex(
    '55AA0308EC4D2FC67A06357A06307B06357A0633790634780622080709ED0609BAC7DB39'
)
PACKETS = [PKT_EMPTY, PKT_MIXED, PKT_FULL]


def test_checksum_matches_captured_packets():
    for pkt in PACKETS:
        stored = pkt[34] | (pkt[35] << 8)
        assert checksum(pkt) == stored


def test_parse_angles_and_speed():
    p = parse_packet(PKT_EMPTY)
    assert p is not None
    assert abs(p.speed_hz - 5.19) < 0.05
    assert abs(p.start_deg - 138.77) < 0.02
    assert abs(p.end_deg - 144.80) < 0.02


def test_invalid_points_filtered():
    # cả 8 điểm là 0x8000 / quality 0 -> toàn None
    assert parse_packet(PKT_EMPTY).points == [None] * 8
    # đủ 8 điểm hợp lệ, khoảng cách dương tính bằng mm
    p = parse_packet(PKT_FULL)
    assert all(pt is not None for pt in p.points)
    assert all(0 < pt[0] < 8000 for pt in p.points)


def test_feed_resyncs_on_garbage():
    stream = bytearray(b'\x12\x34\x55\xAA' + PKT_EMPTY + b'\x55\xAA\x03' + PKT_FULL)
    packets = feed(stream)
    assert len(packets) == 2
    assert len(stream) < PACKET_SIZE  # phần đuôi rác ngắn hơn một gói


def test_corrupted_checksum_rejected():
    bad = bytearray(PKT_MIXED)
    bad[10] ^= 0xFF
    assert parse_packet(bytes(bad)) is None
