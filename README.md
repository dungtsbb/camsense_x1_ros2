# camsense_x1 — ROS2 driver cho lidar Camsense X1

Node ROS2 (Python) đọc lidar Camsense X1 qua cổng serial và publish
`sensor_msgs/LaserScan` lên topic `scan`. Kiểm checksum từng gói, gom đủ một
vòng quay mới publish, xuất góc theo chuẩn REP-103 (ngược chiều kim đồng hồ).

Chi tiết giao thức 36 byte của X1:
[bài viết trên vinexus.dev](https://vinexus.dev/blog/doc-lidar-camsense-x1-esp32/).

## Yêu cầu

- ROS2 (test trên Humble và Jazzy)
- Python 3, `pyserial`
- Lidar nối qua mạch USB–UART: TX lidar → RX, cấp 5V và GND. Baud cố định 115200.

## Cài đặt

```bash
mkdir -p ~/lidar_ws/src && cd ~/lidar_ws/src
git clone https://github.com/dungtsbb/camsense_x1_ros2.git
cd ~/lidar_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/local_setup.bash
```

## Chạy

```bash
# cấp quyền cổng serial (một lần, hoặc thêm user vào group dialout)
sudo chmod 666 /dev/ttyUSB0

ros2 launch camsense_x1 camsense_x1.launch.py port:=/dev/ttyUSB0
```

Kiểm tra:

```bash
ros2 topic hz /scan        # ~5.2 Hz, đúng tốc độ quay của lidar
rviz2                      # add LaserScan, topic /scan, Fixed Frame = laser
```

## Tham số

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Cổng serial |
| `frame_id` | `laser` | Frame của LaserScan |
| `bins` | `400` | Số ô góc mỗi vòng (X1 cho ~400 điểm/vòng) |
| `range_min` | `0.08` | Khoảng cách nhỏ nhất (m), gần hơn bị loại |
| `range_max` | `8.0` | Khoảng cách lớn nhất (m), xa hơn bị loại |
| `counterclockwise` | `true` | Đảo chiều góc cho đúng REP-103 |

## Test

Phần giải mã giao thức (`camsense_x1/protocol.py`) thuần Python, được test
bằng các gói dữ liệu bắt thực tế từ phần cứng:

```bash
python3 -m pytest test/
```

## Nguồn giao thức

- [Vidicon/camsense-X1](https://github.com/Vidicon/camsense-X1) — cấu trúc gói, công thức góc/tốc độ
- [thijses/camsense-X1](https://github.com/thijses/camsense-X1) — gói dữ liệu mẫu dùng trong test
- [kaiaai/LDS](https://github.com/kaiaai/LDS) + [camsense/T2SDK](https://github.com/camsense/T2SDK) — thuật toán checksum

## License

MIT
