# UR3e + RG2: bàn cờ 3×3 và nhận diện RGB-D

Mô phỏng ROS 2 Humble / Gazebo Fortress, quan sát bàn cờ bằng camera trên cao.
Chạy mô phỏng và nhận diện không cần API key.

## Cấu trúc

- `ur_description`: mô hình, meshes và thông số robot UR.
- `ur_onrobot`: mô hình UR3e gắn kẹp RG2 và controller.
- `ur_simulation_gz`: bàn, bàn cờ, 8 quân, camera RGB-D và bridge Gazebo–ROS.
- `infomation_communication`: nhận `/position_state`, kiểm tra dữ liệu và phát `/information_state`.
- `camera_detection`: nhận diện 17 vị trí và theo dõi 8 quân từ RGB-D.
- `ur_task_planner`: nhận lệnh chuyển quân, thực thi qua MoveIt và phát thông báo.

## Bố cục bàn cờ

Nhìn từ camera: hàng **A, B, C** từ trên xuống; cột **1, 2, 3** từ trái sang phải.
Hàng A ở phía +Y; cột 1 ở phía −X của `gazebo_world`.

Bàn cờ A1–C3 ban đầu trống. Hàng D1–D8 chạy dọc theo trục Y bên trái bàn cờ,
trên mặt bàn làm việc nhưng bên ngoài bàn cờ:

| Vị trí | D1 | D2 | D3 | D4 | D5 | D6 | D7 | D8 |
|---|---|---|---|---|---|---|---|---|
| Quân | Đỏ 1 | Đỏ 2 | Đỏ 3 | Đỏ 4 | Vàng 1 | Vàng 2 | Vàng 3 | Vàng 4 |

Ô rộng 55 mm, quân 40 mm. D1 có tâm X=-0.170, Y=0.1925 m;
các vị trí tiếp theo giảm Y 55 mm, cùng X. Mặt đỡ Z=0.806 m.
Camera ở (-0.07, 0, 2.2) m, nhìn xuống và bao phủ đủ 17 vị trí. Thân camera
được khai báo là vật cản trong MoveIt.
`random_seed` được giữ để tương thích lệnh cũ; bố cục D hiện luôn cố định.

Package `camera_detection` lưu mảng hàng D và 8 mảng one-hot cho các quân.
Xem [cấu trúc trạng thái và giới hạn nhận dạng](camera_detection/README.md).

## Cài đặt và build

Cần Ubuntu 22.04 và ROS 2 Humble đã cài.

```bash
sudo apt update
sudo apt install -y build-essential cmake \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml \
  ros-humble-moveit ros-humble-ur-moveit-config ros-humble-ur-controllers \
  ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-controllers \
  ros-humble-rviz2 ros-humble-rmw-fastrtps-cpp
```

Chỉ chạy `sudo rosdep init` nếu máy chưa từng khởi tạo rosdep.

```bash
cd ~/ur3_final_project
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --allow-overriding ur_description --cmake-force-configure
source install/setup.bash
ros2 run ur_task_planner check_install.py
```

## Chạy mô phỏng và điều khiển

Dừng các launch robot/Joint State Publisher GUI cũ để không có hai nguồn TF hoặc
joint state cho cùng một robot.

```bash
cd ~/ur3_final_project
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
ros2 launch ur_task_planner board_sim.launch.py
```

Launch mở Gazebo, RViz, camera, MoveIt, bộ thực thi và node Notification.
Robot đợi lệnh qua dịch vụ `/move_piece`. Đóng cửa sổ Gazebo hoặc nhấn
`Ctrl+C` trong terminal launch sẽ dừng mô phỏng và các cầu nối.
Mỗi ROS domain dùng một phiên mô phỏng. Khi RViz báo đồng hồ nhảy lùi,
`ros2 topic info /clock` giúp kiểm tra có nhiều nguồn đồng hồ hay không.

Để chạy không mở giao diện:

```bash
ros2 launch ur_task_planner board_sim.launch.py gazebo_gui:=false launch_rviz:=false
```

Trong một terminal khác đã source ROS và workspace, mở luồng thông báo:

```bash
ros2 topic echo /notification std_msgs/msg/String --no-daemon \
  --qos-reliability reliable --qos-durability transient_local --full-length
```

Đợi log `Skill Executor Node ready.`, rồi gửi lệnh từ terminal đã cấu hình ROS:

```bash
ros2 service call /move_piece ur_task_planner/srv/MovePiece \
  "{color: red, source: D1, destination: A1}"
```

Dịch vụ nhận `red` / `đỏ` hoặc `yellow` / `vàng`. Phản hồi `accepted: true`
báo lệnh đã được nhận. Notification phát `WAITING`, `EXECUTING` cho từng
bước và `DONE` khi camera xác nhận quân đến đúng ô. Nếu nguồn và đích
trùng nhau, màu sai, nguồn trống, đích bị chiếm hoặc camera chưa xác nhận
được vị trí, node báo lỗi. Lệnh mới trong khi robot đang xử lý sẽ nhận
`BUSY` và bị từ chối.

## Xem ô nào có quân màu nào

Terminal khác:

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_final_project/install/setup.bash
ros2 topic echo /board_state --full-length
```

`/board_state` và `/position_state` là JSON cho cả 17 vị trí. Ban đầu A1–C3
là `empty`, D1–D4 là `red`, D5–D8 là `yellow`.

```bash
ros2 topic echo /d_occupancy
ros2 topic echo /position_state --full-length
```

`d_occupancy` có 8 phần tử 0/1. `piece_locations` có 8 mảng one-hot dài 17;
`position_order` nêu thứ tự các vị trí. Các cờ `d_valid` và `piece_valid`
phải được kiểm tra khi có che khuất/mất camera.
RViz hiển thị ảnh camera và markers cho cả bàn cờ lẫn hàng D.

## Xem trên máy B qua Discovery Server

Máy B cần cùng các package mô hình và bản build dự án để mở cấu hình RViz mới.
Dùng IP Wi-Fi hiện tại của máy A; ví dụ đã kiểm tra kết nối: `10.11.84.48`.

Trên A, một terminal riêng:

```bash
source /opt/ros/humble/setup.bash
fastdds discovery -i 0 -l 10.11.84.48 -p 11811
```

Trong **mỗi terminal ROS trên cả A và B**, trước khi chạy launch hoặc RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_final_project/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=10.11.84.48:11811
export ROS_SUPER_CLIENT=TRUE
ros2 daemon stop
```

A chạy `ros2 launch ur_task_planner board_sim.launch.py` như trên.
B mở RViz, không chạy thêm mô phỏng hay observer:

```bash
LC_ALL=C.UTF-8 ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix ur_task_planner)/share/ur_task_planner/rviz/board.rviz" \
  --ros-args -p use_sim_time:=true
```

## Kiểm tra

```bash
cd ~/ur3_final_project
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
colcon test --packages-select ur_simulation_gz ur_task_planner
colcon test-result --verbose
```

Bộ kiểm tra bao gồm hình học bàn cờ, khả năng đặt một quân mỗi ô, nhiều quân cùng
màu, depth không hợp lệ, ô bị che và việc từ chối đặt vào ô đã có quân.
