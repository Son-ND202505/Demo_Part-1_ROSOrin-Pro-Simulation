# Cập nhật định vị và chọn đích — 17/10/2026

## Vấn đề và thay đổi

Script cũ luôn đặt initial pose từ JSON, kể cả khi robot đã di chuyển. Script
mới mặc định kiểm tra AMCL hiện tại, không đọc JSON pose cũ. Nếu kiểm tra không
đạt, nó yêu cầu AMCL tìm lại vị trí toàn cục và thử khởi tạo bằng so khớp LiDAR
với bản đồ. Không sử dụng tọa độ thật từ Gazebo và không phát vận tốc trong
giai đoạn định vị. Các service AMCL sử dụng là `reinitialize_global_localization`
và `request_nomotion_update`, có trong
[mã nguồn Nav2 Humble](https://api.nav2.org/nav2-humble/html/amcl__node_8cpp_source.html).

`global_scan_match.py` tìm trên lưới vị trí/hướng, tinh chỉnh ứng viên bằng
Powell, đánh giá khoảng cách điểm cuối tia đến vật cản và vùng trống dọc tia.
Ứng viên điểm thấp hoặc có nghiệm cạnh tranh gần bằng nhau bị loại. Sau đó
`localization_guard.py` yêu cầu 5 cập nhật AMCL liên tiếp đạt: độ lệch chuẩn
vị trí tối đa 0,35 m, góc 0,35 rad, ít nhất 65% điểm cuối quét cách vật cản
không quá 0,20 m; dữ liệu scan, pose và TF phải còn mới. Đây là tiêu chí kinh
nghiệm, không phải chứng minh vị trí đúng theo ground truth.

Trước khi gửi đích, kiểm tra đích nằm trong ô trống của bản đồ và cách vật cản
đã biết tối thiểu 0,16 m. Nav2 vẫn phải tìm được đường đi phù hợp. Một khóa tiến
trình ngăn hai script trials/localize trong dự án chạy đồng thời; người dùng
vẫn cần dừng teleop và action gửi từ RViz trước khi khởi tạo lại định vị.

## Cách dùng

```bash
# Từ gốc repository, với Gazebo và navigation đang chạy, SLAM đã dừng:
bash part1_rosorin/scripts/run.sh trials --task single --goal 0.8 1.0 -90
bash part1_rosorin/scripts/run.sh trials --task waypoints
```

Chờ lệnh trước hoàn thành rồi mới chạy lệnh sau. Đổi danh sách trong
[navigation_goals.yaml](../config/navigation_goals.yaml), dùng mét và yaw_deg.
`--task both` chạy hai bài. `--goals PATH` chọn YAML riêng. `--localization global`
buộc tìm lại vị trí sau teleport; `--localization current` giữ AMCL và không
reset toàn cục. `run.sh localize` chỉ định vị để chuẩn bị dùng RViz. Pose JSON
chỉ được sử dụng nếu truyền `--initial-pose PATH` tường minh.

## Kết quả kiểm chứng

Sửa thêm lỗi khởi động từ phiên mới: chỉ chờ AMCL active trước khi định vị;
chờ bt_navigator active sau khi định vị đạt. Trước đó, script chờ bt_navigator
quá sớm trong khi planner/global costmap lại chờ TF map từ AMCL, tạo vòng chờ.
Kiểm tra hồi quy xác nhận phải cập nhật/kiểm tra định vị trước khi chờ Nav2.

- 6 kiểm tra tự động đạt: giữ pose hiện tại không reset, đổi đích/đổi độ sang
  radian, từ chối dữ liệu không hợp lệ, xử lý gốc bản đồ quay, kiểm tra ô đích,
  tìm lại pose trong phòng bất đối xứng tổng hợp từ quét ray-cast.
- Lần tìm toàn cục đầu chưa hội tụ, timeout và không gửi đích:
  [JSON](../results/runtime/navigation_trials_20261017_122827_666138.json).
- Một lần tìm toàn cục sau đó đạt kiểm tra, không có pose nhập trước:
  [JSON](../results/runtime/navigation_trials_20261017_123014_738999.json).
  Lần này AMCL hội tụ; trường scan_seed là null, không tuyên bố ứng viên từ
  thuật toán so khớp đã được dùng trong phép thử thực tế này.
- Một lần khác timeout ở service bt_navigator/get_state, không gửi đích:
  [JSON](../results/runtime/navigation_trials_20261017_123057_568123.json).
- Điều hướng một điểm từ pose đang được AMCL theo dõi: SUCCEEDED, 14,03 giây
  thực, sai số cuối 0,196 m, global_resets = 0, initial_pose_source = null:
  [JSON](../results/runtime/navigation_trials_20261017_123227_865323.json).
- Tiếp tục ba waypoint từ vị trí vừa đạt: SUCCEEDED, 20,33 giây thực, không
  bỏ sót waypoint, sai số cuối 0,256 m, global_resets = 0:
  [JSON](../results/runtime/navigation_trials_20261017_123308_443228.json).

Sai số đo theo AMCL/TF; không có đánh giá độc lập bằng ground truth. Không
thử teleport đến mọi vị trí và không bảo đảm hội tụ trong phòng đối xứng hoặc
vùng chưa được lập bản đồ. Kiểm tra chất lượng chạy trước mỗi bài gọi script,
không giám sát/tái định vị toàn cục liên tục giữa một action đang thực thi.
Các JSON mới có tên thời gian để không ghi đè minh chứng của lần khác. Không
tự cập nhật CSV/biểu đồ lịch sử vì các script tổng hợp hiện đọc tên tệp cố định.

Để chạy lại kiểm tra không cần Gazebo:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 part1_rosorin/scripts/test_navigation_inputs.py
```
