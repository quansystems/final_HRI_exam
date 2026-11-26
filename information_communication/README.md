# information_communication

Node ROS 2 `/information_receiver` kiểm tra và phát lại kết quả của camera bàn
cờ và webcam nhận diện biểu cảm, cử chỉ.

```text
camera_detection → /position_state → information_receiver → /information_state
detection_behaviour → /behaviour_state → information_receiver → /mutual_communication
```

Các topic dùng `std_msgs/msg/String` chứa JSON. Khi nhận bản tin bàn cờ, node
giải mã JSON và thực hiện các kiểm tra sau.

- `position_order` phải theo đúng thứ tự A1–C3 rồi D1–D8. `cells` phải đủ khóa của 17 vị trí.
- `d_occupancy` phải gồm 8 số nguyên `0` hoặc `1`, theo thứ tự D1–D8.
- Mỗi quân từ `red_1` đến `red_4` và `yellow_1` đến `yellow_4` có mảng `piece_locations` gồm 17 số `0` hoặc `1`, với tối đa một phần tử bằng `1`.
- `d_valid` phải có 8 cờ boolean. `piece_valid` phải có cờ boolean cho cả 8 quân.

Nếu kiểm tra thành công, node lưu bản tin trong `self.state` và phát lại toàn bộ
dữ liệu qua `/information_state`. Nếu JSON sai hoặc cấu trúc không hợp lệ, node
ghi cảnh báo, bỏ qua bản tin đó và giữ bản tin hợp lệ trước đó. Cờ `complete`,
`d_valid` và `piece_valid` được giữ nguyên. Ví dụ, khi `d_valid[0]` là `false`,
trạng thái D1 ở ảnh hiện tại chưa được camera xác nhận. Cả hai topic dùng QoS
`RELIABLE`, `TRANSIENT_LOCAL` và giữ bản tin gần nhất cho node kết nối sau.

Webcam phát khi trạng thái thay đổi và nhắc lại mỗi giây. Bản tin hợp lệ gồm
bốn cờ boolean.

| Trường | Ý nghĩa khi `true` |
| --- | --- |
| `available` | Có dữ liệu webcam mới |
| `two_open_palms` | Cả hai bàn tay đang cùng xòe |
| `happy` | Nhận diện biểu cảm vui vẻ |
| `angry` | Nhận diện biểu cảm tức giận |

Máy B nhận bốn cờ này qua `/mutual_communication`. `/information_behaviour`
vẫn mang cùng dữ liệu để hỗ trợ các node đã đăng ký từ trước. Khi có dữ liệu bàn cờ,
node đưa thêm đối tượng `behaviour` vào `/information_state`. Bản tin webcam
phải có đủ các cờ, và `happy` với `angry` không thể cùng `true`. Khi webcam dừng
hoặc sau khoảng 3 giây không có bản tin hợp lệ, cả bốn cờ trở về `false`.
Node gửi trạng thái ngay khi thay đổi và phát lại mỗi giây,
kể cả khi trạng thái giữ nguyên.
Các topic trạng thái webcam cũng giữ bản tin gần nhất với QoS `RELIABLE` và
`TRANSIENT_LOCAL`. Dữ liệu gửi qua ROS 2 chỉ gồm các cờ kết quả.

`board_sim.launch.py` tự bật `/information_receiver`. Node `/piece_command`
hiện nhận camera trực tiếp từ `/position_state`. Các node cần dùng bản tin đã
qua kiểm tra cấu trúc có thể đăng ký `/information_state`.

## Thử trên máy A trước

Build package từ một terminal đã source ROS 2 Humble.

```bash
cd ~/ur3_final_project
source /opt/ros/humble/setup.bash
colcon build --packages-select information_communication
```

Để thử cục bộ, đặt cấu hình sau trong mỗi terminal mở cho node, webcam và lệnh đọc.

```bash
source /opt/ros/humble/setup.bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Terminal node nhận. Nếu đang chạy launch tổng sau khi build, dùng node nhận
đã có trong launch đó.

```bash
source ~/ur3_final_project/install/setup.bash
ros2 run information_communication information_receiver_node.py
```

Terminal webcam.

```bash
cd ~/ur3_final_project/src/UniversalRobotandGripper/detection_behaviour
env -u PYTHONNOUSERSITE python3 detect.py --camera 0 --ros
```

Terminal đọc kết quả webcam.

```bash
ros2 topic echo /mutual_communication std_msgs/msg/String --no-daemon \
  --qos-reliability reliable --qos-durability transient_local --full-length
```

Khi có mô phỏng, có thể đọc bản tin tổng hợp của bàn cờ và webcam.

```bash
ros2 topic echo /information_state std_msgs/msg/String --no-daemon \
  --qos-reliability reliable --qos-durability transient_local --full-length
```

## Truyền giữa máy A và máy B

Dùng ROS 2 Humble trên cả hai máy. Lấy IP Wi-Fi hiện tại của máy A bằng
`ip -brief -4 address`. Ví dụ dưới đây dùng `10.11.103.168`.

Trên máy A, nếu cần khởi động Discovery Server, giữ lệnh này chạy trong một
terminal riêng.

```bash
source /opt/ros/humble/setup.bash
fastdds discovery -i 0 -l 10.11.103.168 -p 11811
```

Trước khi mở node, webcam, RViz hoặc lệnh đọc, đặt cùng cấu hình trong mỗi
terminal ROS trên cả hai máy. Nếu đã cấu hình Discovery Server cho RViz, dùng
đúng địa chỉ và domain đang dùng ở đó.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=10.11.103.168:11811
export ROS_SUPER_CLIENT=TRUE
```

Máy A chạy node chuyển tiếp bằng launch sau khi đã build. Nếu `board_sim.launch.py`
đã bật node nhận với cấu hình mạng chung, dùng node đó.

```bash
source ~/ur3_final_project/install/setup.bash
ros2 launch information_communication send_to_machine_b.launch.py \
  discovery_server:=10.11.103.168:11811 domain_id:=30
```

Máy A chạy webcam với cấu hình mạng trong một terminal riêng. Máy B chạy
lệnh sau để đọc cờ trạng thái. Máy B chỉ cần ROS 2 Humble để đọc bằng terminal.

```bash
ros2 topic echo /mutual_communication std_msgs/msg/String --no-daemon
```

Để thử truyền theo chiều B → A, dừng webcam phát trên A và giữ node nhận trên
A chạy. Trên B phát mẫu sau. Dùng lệnh đọc `/mutual_communication` trên A để
kiểm tra kết quả.

```bash
ros2 topic pub -r 1 /behaviour_state std_msgs/msg/String \
  '{data: "{\"available\":true,\"two_open_palms\":true,\"happy\":false,\"angry\":false}"}' \
  --qos-reliability reliable --qos-durability transient_local
```
