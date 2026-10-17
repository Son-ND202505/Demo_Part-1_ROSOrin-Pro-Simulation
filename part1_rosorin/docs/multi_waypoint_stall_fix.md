# Lỗi robot không đi tiếp khi điều hướng nhiều điểm

Ngày: 2026-10-17. Mọi số liệu dưới đây đo từ Gazebo/Nav2 thật; lần kiểm chứng sau
khi sửa chạy trong một mô phỏng headless tách biệt (ROS_DOMAIN_ID và IGN_PARTITION
riêng), không dùng lại kết quả của phiên cũ.

## Hiện tượng

Chạy nhiều waypoint bằng bảng Navigation 2 trong RViz (đích 1 `(4.58, 3.16)`, đích 2
`(-2.99, 2.45)`, đích 3 `(-3.97, -2.27)`), robot đứng gần `(2.5, -1.3)` rồi quay
qua lại. Đường đi màu xanh trong RViz rõ ràng **không sát tường**, nhưng Nav2 báo
`Failed to make progress` mỗi ~10 s, chạy hồi phục (spin, wait, backup) mà vẫn không
đi được; bảng điều khiển tăng dần "Number of recoveries". Trong 160 s robot chỉ
dịch ~1 m.

## Nguyên nhân (ba nguyên nhân độc lập)

### 1. Plugin MecanumDrive của Gazebo sai hệ số động học (nguyên nhân gốc)

Khối plugin của hãng khai báo `wheel_separation 0.34`, `wheelbase 0.30`,
`wheel_radius 0.05`, nhưng các khớp bánh mà nó điều khiển nằm tại `(±0.08, ±0.09)`
với bánh bán kính 0.04. Đo lệnh hở qua `/controller/cmd_vel`
([số liệu](../results/runtime/stall_fix/drive_calibration.txt)):

| Lệnh | Thực tế trước sửa | Thực tế sau sửa |
| --- | --- | --- |
| tiến 0.20 m/s | 0.160 m/s (0.8×) | 0.200 m/s |
| quay 1.0 rad/s | 1.61 rad/s (1.6×) | 0.95 rad/s |

Khi Nav2 chạy, tốc độ quay đo được lên tới 1.4–2.06 rad/s trong khi bộ điều khiển
giới hạn 1.0 rad/s. Mọi lần quay đều quá đà, bộ điều khiển đảo chiều, rồi lại quá
đà. Bài một điểm gần đường thẳng nên không lộ lỗi này.

### 2. DWB bị kẹt ở đường đi hình chữ U vòng quanh vách mỏng

Phòng có một vách mỏng nằm giữa phòng; cửa vòng qua vách rộng ~1.05 m ở `x ≈ 1.0`.
Từ `(2.3, -1.5)` đến đích 1, đường đi của planner đi về phía tây qua cửa, lên phía
bắc rồi quay lại phía đông, tức là chữ U quanh vách. DWB chỉ nhìn đoạn đường nằm
trong costmap cục bộ 4×4 m; điểm cuối đoạn đó nằm ngay phía bắc robot, **bên kia
vách**. Hai critic `GoalAlign` và `GoalDist` (24 + 24) kéo robot thẳng về điểm đó
(khoảng cách Manhattan, bỏ qua vật cản), `BaseObstacle` chặn hướng ấy, còn
`PathAlign`/`PathDist` kéo về phía đường đi. Ghi `/evaluation` cho thấy lệnh DWB
chọn là `(0, 0, ±1.0)` gần như liên tục: có lúc tới 315 trong 399 quỹ đạo đi tới bị
`BaseObstacle` loại, và có lúc 200 bị `Oscillation` loại. Đây là lý do robot không
đi dù đường đi xa tường. Chỉ sửa nguyên nhân 1 là chưa đủ: sau khi sửa hệ số động
học, robot vẫn quay tại chỗ như cũ.

### 3. Đích 2 không thể tới

`(-2.99, 2.45)` nằm trong một đám ô chiếm dụng rải rác (khoảng 3×2 m) trong bản
đồ, cách ô bị chiếm gần nhất 0.15 m. Sau khi trừ bán kính robot 0.16 m gần như cả
vùng bị chặn nên planner báo `failed to create plan`. Script `navigation_trials.py`
từ chối đích như vậy ("too close to a mapped obstacle"); RViz thì không kiểm tra.

AMCL cũng làm tình hình xấu hơn: khi robot quay quá đà, `map→odom` dao động
0.22–0.55 m theo trục y và tới 8° (odom của Gazebo là pose thật của mô hình).

## Cách sửa

| Tệp | Thay đổi |
| --- | --- |
| `launch/room.launch.py` | `_fit_mecanum_drive` suy ra `wheel_separation`, `wheelbase`, `wheel_radius` từ chính các khớp bánh trong URDF khi tạo mô tả robot. Mã nguồn hãng vẫn giữ nguyên. |
| `scripts/prepare_nav_config.py` (sinh `config/nav2.yaml`) | Thay DWB bằng Regulated Pure Pursuit, bám đường đi bằng điểm carrot thay vì kéo về điểm cuối cục bộ. `xy_goal_tolerance` 0.25 → 0.15 m. AMCL `alpha1..5` 0.2 → 0.05. |
| `scripts/test_navigation_inputs.py` | Test cho phép suy hình học MecanumDrive từ URDF. |

Sau khi pull thay đổi cần **khởi động lại cả mô phỏng và Nav2**, vì tham số được nạp
lúc khởi động. `run.sh navigation` chỉ tạo lại `nav2.yaml` khi tệp chưa tồn tại,
nên nếu có bản cũ hãy chạy `python3 part1_rosorin/scripts/prepare_nav_config.py`.

## Kiểm chứng sau khi sửa

| Bài kiểm tra | Kết quả |
| --- | --- |
| Từ vị trí kẹt `(2.4, -1.3)` đến đích 1 `(4.58, 3.16)` | `SUCCEEDED`, ~40 s cho đoạn đi qua cửa và vòng vách |
| Ba đích của bạn | Đích 1 và 3 đạt; đích 2 bị bỏ qua (`missed: [1]`), đúng như nguyên nhân 3 |
| Chuỗi 5 đoạn dài hai chiều qua cửa | `SUCCEEDED`, `missed: []`, 2/2 lần chạy; log Nav2 không có `Failed to make progress` hay hồi phục |
| Bộ mặc định `--task both` ([JSON](../results/runtime/stall_fix/trials_default_after_fix.json)) | PASS; một điểm 0.127 m, nhiều điểm 0.140 m, không bỏ sót |
| `test_navigation_inputs.py` | 7 test đạt |

Giới hạn: đây là vài lần chạy chức năng trong một mô phỏng, không phải đánh giá
độ tin cậy trên mẫu lớn. Một lần chạy khác bị loại vì định vị toàn cục không hội tụ
khi robot đứng ở góc phòng `(4.9, 3.4)` (script báo FAIL, không gửi đích); hãy khởi
tạo AMCL ở nơi có nhiều đặc trưng (gần vị trí xuất phát) hoặc bằng 2D Pose Estimate. Sai số vị trí đo bằng AMCL/TF, không phải ground truth độc
lập. Chưa kiểm tra riêng RPP với hệ số động học cũ nên không tách được phần đóng
góp của từng thay đổi ngoài thí nghiệm "chỉ sửa động học vẫn kẹt". Báo cáo PDF và
các JSON cũ (sai số cuối 0.217–0.248 m) được đo với DWB và cấu hình trước khi sửa,
nên không bị ghi đè.

## Chọn đích để không gặp lỗi tương tự

- Chọn đích trong vùng trống, cách vật cản đã vẽ trên bản đồ ít nhất 0.3 m, tránh
  các đám chấm rải rác trên bản đồ (ví dụ quanh `(-3, 2.5)`).
- Nếu planner báo `failed to create plan`, đích nằm trong vùng không tới được. Đổi
  đích thay vì chờ recovery.
- LiDAR chỉ có góc nhìn 156° phía trước (±78°), nên robot không thấy phía sau; hãy
  để Nav2 tự quay đầu thay vì lùi sát tường bằng tay.
