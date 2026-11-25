# Camera detection: A1–C3 và D1–D8

Camera RGB-D phát hiện màu và vị trí từ ảnh, depth và TF; không đọc pose quân từ Gazebo.
Package `camera_detection` chứa nhận diện, theo dõi và node `camera_detection_node.py`.

## Trạng thái

- `/position_state` và `/board_state`: JSON (`std_msgs/msg/String`) cho cả 17 vị trí.
- `/d_occupancy`: `std_msgs/msg/Int32MultiArray`, 8 phần tử 0/1 theo D1…D8.
- `/board_markers`: hiển thị kết quả trên RViz.
- `/observed_scene`: pose quân cho executor nếu đã cài các interface của `ur_task_planner`.

JSON gồm:

- `cells`: từng vị trí là `red`, `yellow`, `empty`, `unknown` hoặc `invalid`.
- `occupants`: từng vị trí chứa ID quân (ví dụ `red_1`), `null` nếu trống, hoặc trạng thái chưa rõ.
- `d_occupancy`: ban đầu `[1,1,1,1,1,1,1,1]`; 1 là có quân, 0 là ô trống đã được quan sát.
- `d_valid`: 8 cờ cho biết giá trị tương ứng có được camera xác nhận ở ảnh hiện tại không.
- `piece_locations`: 8 mảng one-hot dài 17, khóa `red_1`…`red_4`, `yellow_1`…`yellow_4`.
- `piece_valid`: cờ xác nhận vị trí từng quân trong ảnh hiện tại.
- `position_order`: `[A1,A2,A3,B1,B2,B3,C1,C2,C3,D1,D2,D3,D4,D5,D6,D7,D8]`.

Ví dụ `red_1` ban đầu có phần tử thứ 10 (index 9, D1) bằng 1. Sau khi chuyển
sang B2, phần tử index 4 bằng 1, các phần tử còn lại bằng 0; `d_occupancy[0]` về 0.
Khi che khuất/mất ảnh, giữ mảng vị trí cuối cùng nhưng đặt cờ valid về false;
không được coi những giá trị này là vị trí hiện tại đã xác nhận.

Bốn quân cùng màu không có mã nhận dạng riêng. ID được gán theo D lúc khởi tạo
và theo dõi bằng vị trí gần nhất; không bảo đảm danh tính sau che khuất/hoán đổi
đồng thời hoặc khởi động lại observer khi quân đã bị di chuyển.

## Hình học khởi tạo

Bàn cờ ban đầu trống. Các quân ở trên bàn làm việc, bên ngoài bàn cờ:

- D1…D8: X = `-0.170`, Y = `0.1925 - index * 0.055` m.
- D1–D4 đỏ; D5–D8 vàng.
- D song song trục Y; D1 đến D8 theo chiều giảm Y.
- D và bàn cờ có mặt đỡ Z = 0.806 m; tâm quân Z = 0.826 m.
- Các vị trí nằm trong giới hạn radial 0.56 m của cấu hình, tính cả bán kính quân.
  Kiểm tra hình học không thay cho kiểm chứng quỹ đạo và thao tác gắp thực tế.

## Chạy

```bash
cd ~/ur3_final_project
source /opt/ros/humble/setup.bash
colcon build --allow-overriding ur_description
source install/setup.bash
ros2 launch ur_task_planner board_sim.launch.py
```

Trong terminal có cùng cấu hình ROS:

```bash
ros2 topic echo /d_occupancy
ros2 topic echo /position_state --full-length
```

Có thể chạy riêng detector khi mô phỏng/camera đã chạy (tránh chạy hai observer):

```bash
ros2 run camera_detection camera_detection_node.py --ros-args \
  --params-file "$(ros2 pkg prefix ur_task_planner)/share/ur_task_planner/config/scene_depth.yaml" \
  -p use_sim_time:=true
```
