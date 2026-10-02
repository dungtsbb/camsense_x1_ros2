"""Node ROS2 đọc lidar Camsense X1 qua serial và publish sensor_msgs/LaserScan."""

import math

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from camsense_x1.protocol import feed

BAUD = 115200  # cố định với X1


class CamsenseX1Node(Node):
    def __init__(self):
        super().__init__('camsense_x1')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('bins', 400)      # X1 cho ~400 điểm/vòng
        self.declare_parameter('range_min', 0.08)
        self.declare_parameter('range_max', 8.0)
        # X1 quay theo chiều kim đồng hồ; True = đảo lại cho đúng REP-103 (CCW dương)
        self.declare_parameter('counterclockwise', True)

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.bins = int(self.get_parameter('bins').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.ccw = bool(self.get_parameter('counterclockwise').value)

        self.pub = self.create_publisher(LaserScan, 'scan', 10)

        self.ser = None
        self.buf = bytearray()
        self.speed_hz = 5.2
        self.prev_start_deg = None
        self._reset_scan()

        self.create_timer(0.02, self._poll)
        self.get_logger().info(f'Camsense X1: {self.port} @ {BAUD}, {self.bins} bins')

    def _reset_scan(self):
        self.dist_mm = [0] * self.bins
        self.intensity = [0] * self.bins

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0)
            self.get_logger().info(f'Đã mở {self.port}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Không mở được {self.port}: {e}', throttle_duration_sec=5.0)

    def _poll(self):
        if self.ser is None:
            self._open_serial()
            if self.ser is None:
                return
        try:
            data = self.ser.read(4096)
        except serial.SerialException as e:
            self.get_logger().warn(f'Mất kết nối serial: {e}')
            self.ser.close()
            self.ser = None
            return
        if not data:
            return

        self.buf += data
        for pkt in feed(self.buf):
            self._handle_packet(pkt)

    def _handle_packet(self, pkt):
        self.speed_hz = pkt.speed_hz

        # Lidar không có cờ hết vòng: góc đầu gói nhỏ hơn gói trước
        # nghĩa là kim vừa quay qua mốc 0 độ
        if self.prev_start_deg is not None and pkt.start_deg < self.prev_start_deg:
            self._publish_scan()
            self._reset_scan()
        self.prev_start_deg = pkt.start_deg

        span = pkt.end_deg - pkt.start_deg
        if span < 0.0:
            span += 360.0
        step = span / (len(pkt.points) - 1)

        for i, point in enumerate(pkt.points):
            if point is None:
                continue
            dist_mm, quality = point
            ang = (pkt.start_deg + step * i) % 360.0
            b = int(ang / 360.0 * self.bins + 0.5) % self.bins
            # nhiều điểm rơi cùng một ô: giữ điểm có tín hiệu mạnh hơn
            if self.dist_mm[b] == 0 or quality > self.intensity[b]:
                self.dist_mm[b] = dist_mm
                self.intensity[b] = quality

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi * (self.bins - 1) / self.bins
        msg.angle_increment = 2.0 * math.pi / self.bins
        msg.scan_time = 1.0 / self.speed_hz if self.speed_hz > 0 else 0.0
        msg.time_increment = msg.scan_time / self.bins
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = [math.inf] * self.bins
        intensities = [0.0] * self.bins
        for b in range(self.bins):
            if self.dist_mm[b] == 0:
                continue
            # góc thiết bị tăng theo chiều kim đồng hồ; đảo chỉ số cho CCW
            out = (self.bins - b) % self.bins if self.ccw else b
            r = self.dist_mm[b] / 1000.0
            if self.range_min <= r <= self.range_max:
                ranges[out] = r
                intensities[out] = float(self.intensity[b])

        msg.ranges = ranges
        msg.intensities = intensities
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CamsenseX1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
