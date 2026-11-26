# Nhận diện biểu cảm và hai bàn tay xòe

`detect.py` dùng webcam máy tính và chạy suy luận trên CPU. Màn hình chỉ hiện
`Vui ve`, `Tuc gian` hoặc `Hai tay xoe` khi đủ bằng chứng. Ứng dụng có thể báo
biểu cảm và cử chỉ cùng lúc. Chạy riêng với camera RGB-D của bàn cờ.

## Chạy webcam

Trên máy đã có Python 3.10 và các thư viện trong `requirements.txt`:

```bash
cd ~/ur3_final_project/src/UniversalRobotandGripper/detection_behaviour
python3 detect.py --camera 0
```

Nếu cần cài thư viện, dùng môi trường Python riêng để giữ nguyên ROS đang có:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python detect.py --camera 0
```

Mô hình được tải và kiểm SHA-256 ở lần chạy đầu. Webcam mặc định xử lý ở tối đa
5 khung hình/giây, chiều rộng 640 px và hai luồng OpenCV. Có thể dùng `--fps 3`
để giảm tải. Thoát bằng `q`, `Esc` hoặc `Ctrl+C`.

`--headless` tắt cửa sổ và ghi một dòng JSON khi kết quả thay đổi. Ví dụ
`{"expression":"happy","two_open_palms":true}`. Khi bằng chứng biến mất, giá trị
trở về `null` hoặc `false`. Khung hình webcam được xử lý trong bộ nhớ.

Để phát kết quả cho `information_communication`, source ROS 2 Humble và thêm
`--ros` khi chạy `detect.py`. Tham số này gửi các cờ trạng thái lên
`/behaviour_state`. [Hướng dẫn thử cục bộ và truyền qua Wi-Fi](../information_communication/README.md)

Khuôn mặt gần camera nhất được chọn theo kích thước trong ảnh. Để thử cử chỉ,
xòe các ngón và đưa cả hai bàn tay vào khung hình cùng lúc. Hệ thống xác nhận
cả hai tay ở hai khung hình liên tiếp. Xòe từng tay luân phiên sẽ không kích hoạt.

## Chạy lại 5 lượt tinh chỉnh

```bash
python3 benchmark.py --cache ~/ur3_final_project/log/behaviour_benchmark
```

Kết quả từng cấu hình được lưu trong `report.json`. Benchmark chọn tham số trên
tập tinh chỉnh và báo độ chính xác cùng macro-F1 trên tập kiểm tra riêng.
Mẫu biểu cảm dùng đồng thuận của người chấm video CREMA-D, chia tập theo diễn
viên. Mẫu cử chỉ ghép ảnh tay có nhãn từ HaGRID thành chuỗi cùng khung hình và
chia tập theo ảnh nguồn. Tập ghép kiểm tra điều kiện đồng thời, tay đơn, tay nắm
và hai tay luân phiên. Cần video thật để đo độ chính xác cử chỉ trong môi trường dùng.

[OpenCV Zoo — biểu cảm](https://github.com/opencv/opencv_zoo/tree/main/models/facial_expression_recognition) ·
[HSEmotion ONNX](https://github.com/av-savchenko/hsemotion-onnx) ·
[MediaPipe Gesture Recognizer](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer) ·
[CREMA-D](https://github.com/CheyneyComputerScience/CREMA-D) ·
[HaGRID](https://github.com/hukenovs/hagrid)

| Lượt | Thay đổi | Macro-F1 trung bình khi tinh chỉnh | Kiểm tra biểu cảm | Kiểm tra cử chỉ |
| --- | --- | ---: | ---: | ---: |
| 1 | MobileFaceNet và suy luận toàn khung hình | 58,9% | 61,1% | 80,6% |
| 2 | Đổi sang HSEmotion | 59,6% | 65,3% | 80,6% |
| 3 | Trung bình 5 khung hình và hiệu chỉnh điểm tức giận | 68,4% | 76,4% | 80,6% |
| 4 | Soi hai vùng ảnh khi mô hình chưa thấy đủ hai tay | 79,3% | 76,4% | 91,0% |
| 5 | Thử hỗ trợ bằng hình học ngón và xác nhận hai khung hình | 80,4% | 76,4% | 87,5% |

Cấu hình mặc định giữ mô hình và cách soi vùng của lượt 4 kèm xác nhận hai
khung hình. Cấu hình này đạt **55/72** clip biểu cảm và **131/144** chuỗi cử chỉ
trên tập kiểm tra. Trong đó có **22/24** clip tức giận, **16/24** clip vui vẻ,
**23/36** chuỗi hai tay xòe và **108/108** chuỗi âm. Bước hỗ trợ bằng hình học
ở lượt 5 làm tăng báo nhầm trên tập kiểm tra, nên được giữ trong benchmark để
đối chiếu. Thời gian suy luận cả hai tác vụ trên i5-11400H có trung vị khoảng
142 ms và phân vị 95 khoảng 189 ms mỗi khung hình.
