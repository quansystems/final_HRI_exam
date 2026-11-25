# infomation_communication

Node `information_receiver_node.py` nhận JSON `/position_state` từ camera_detection,
kiểm tra 17 vị trí, mảng D dài 8 và 8 mảng one-hot dài 17 cùng các cờ valid.
Lưu snapshot trong `self.state`, phát lại dữ liệu hợp lệ qua `/information_state`.
Không biến dữ liệu chưa xác nhận thành thông tin ô trống.

Launch `ros2 launch ur_task_planner board_sim.launch.py` tự khởi động node này.
Chỉ khi không dùng launch tổng mới chạy riêng:

```bash
ros2 run infomation_communication information_receiver_node.py
```

Xem dữ liệu node nhận bằng:

```bash
ros2 topic echo /information_state std_msgs/msg/String --full-length
```
