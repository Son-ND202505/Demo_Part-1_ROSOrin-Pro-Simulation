# Part 1 — Hướng dẫn thực hành ROSOrin Pro

Tài liệu này hướng dẫn chạy robot trong phòng mô phỏng, xem LiDAR, tạo bản đồ
bằng SLAM, lưu bản đồ, sau đó dùng AMCL/Nav2 để robot tự đi đến một điểm và ba
điểm liên tiếp. Các lệnh khớp với script đang có trong dự án.

**Lần đầu nên làm theo thứ tự bước 0 → 7.** Nếu dùng chính máy đã cài và build
dự án này, có thể bắt đầu ở bước 0 rồi chuyển đến [bước 3](#buoc-3).
Không cần cài lại các gói đã có. Phần tạo bản đồ → chuyển sang điều hướng được
thực hiện trong cùng một phiên Gazebo để giữ đúng vị trí robot.

**Tài liệu liên quan:** [phân tích và sửa lỗi điều hướng nhiều điểm](part1_rosorin/docs/multi_waypoint_stall_fix.md) ·
[tự định vị](part1_rosorin/docs/automatic_localization.md) ·
[kết quả SLAM/điều hướng](part1_rosorin/docs/phase_d_e_f_results.md) ·
[video demo](demos/README.md).

## Mục lục

- [0. Hiểu cách dùng terminal và các thành phần](#buoc-0)
- [1. Chuẩn bị hệ điều hành, ROS và dữ liệu](#buoc-1)
- [2. Build và kiểm tra gói](#buoc-2)
- [3. Mở mô phỏng và kiểm tra cảm biến](#buoc-3)
- [4. Tạo, lưu bản đồ và ghi lại vị trí robot](#buoc-4)
- [5. Chuyển từ SLAM sang điều hướng](#buoc-5)
- [6. Đọc kết quả và thao tác bằng chuột](#buoc-6)
- [7. Tắt chương trình và chạy lại](#buoc-7)
- [8. Lưu kết quả, quản lý GitHub và tạo báo cáo](#buoc-8)
- [9. Xử lý lỗi thường gặp](#buoc-9)

<a id="buoc-0"></a>
## 0. Quy ước trước khi thao tác

| Thành phần | Hiểu đơn giản | Khi nào sử dụng |
| --- | --- | --- |
| Gazebo | Mô phỏng căn phòng, robot và cảm biến | Chạy suốt buổi thực hành |
| RViz | Hiển thị dữ liệu ROS: robot, tia quét, bản đồ, đường đi | Quan sát và chọn đích |
| SLAM Toolbox | Dùng LiDAR và chuyển động để tạo bản đồ | Giai đoạn tạo bản đồ |
| AMCL | Ước lượng vị trí robot trên bản đồ đã lưu | Giai đoạn điều hướng |
| Nav2 | Lập đường đi và điều khiển robot đến đích | Giai đoạn điều hướng |
| TF | Quan hệ giữa các hệ tọa độ, ví dụ `map`, `odom`, `base_footprint` | Cần cho hiển thị và điều hướng |

Mở terminal bằng **Ctrl+Alt+T**, thêm tab bằng **Ctrl+Shift+T**. Dán lệnh bằng
**Ctrl+Shift+V**. Chỉ sao chép nội dung trong khối lệnh; không gõ dấu nhắc `$`.

Trong **mỗi terminal mới**, chuyển vào thư mục dự án:

```bash
cd /home/sonnd/ros2_ws/EyeCode-Robot-Navigation
pwd
```

Trên máy khác, thay đường dẫn trong lệnh `cd` bằng nơi bạn lưu dự án. Các đường
dẫn phía sau đều tính từ thư mục gốc này, không phải từ `part1_rosorin/`.

Nếu dấu nhắc đang có tên môi trường Conda, ví dụ `(unitree_rl_mjlab)` hoặc
`(base)`, chạy `conda deactivate`. Nếu còn môi trường đang kích hoạt, chạy lại
cho đến khi thoát hết. Nếu đang dùng Python venv, chạy `deactivate`.
Chỉ thực hiện trong terminal dành cho ROS; môi trường training vẫn giữ nguyên.
Part 1 không cần PyTorch. Lỗi `No module named torch` trong Python hệ thống
không chứng minh GPU hoặc driver có vấn đề.

Lệnh `bash part1_rosorin/scripts/run.sh ...` tự nạp môi trường ROS trong tiến
trình của nó. Khi chạy trực tiếp lệnh `ros2` hoặc script Python có import ROS,
cần nạp trong terminal hiện tại trước:

```bash
source /opt/ros/humble/setup.bash
source part1_rosorin/ws/install/local_setup.bash
```

Các lệnh `simulation`, `slam`, `map-view`, `navigation` chạy liên tục: terminal
không trở lại dấu nhắc là bình thường. Giữ terminal đó và dùng tab khác để nhập
lệnh tiếp theo. **Ctrl+C** dùng để dừng chương trình đang chạy ở terminal đó.

## Sơ đồ sử dụng terminal

| Terminal | Bước 3: kiểm tra | Bước 4: tạo bản đồ | Bước 5: điều hướng |
| --- | --- | --- | --- |
| T1 | `simulation` | Giữ nguyên | Giữ nguyên |
| T2 | `sensors` | Dừng `sensors`, chạy `slam` | Dừng `slam`, chạy `navigation` |
| T3 | `verify` rồi để trống | `map-view` | Giữ nguyên |
| T4 | Chưa cần | `sweep` hoặc teleop chậm → `save-map` → `capture-pose` | `localize` → `trials` |

Không chạy hai phiên `simulation`. Không chạy SLAM và AMCL cùng lúc. Trong lúc
script hoặc Nav2 điều khiển robot, dừng teleop và không dùng nút điều khiển của
Gazebo để tránh nhiều nguồn cùng gửi lệnh vận tốc.

<a id="buoc-1"></a>
## 1. Chuẩn bị môi trường và dữ liệu — chỉ cần làm lần đầu

### 1.1. Kiểm tra nền tảng

Quy trình của dự án đã được kiểm chứng trên **Ubuntu 22.04, ROS 2 Humble và
Ignition Gazebo Fortress (Gazebo 6)**, có phiên desktop để mở Gazebo/RViz.
Gazebo Classic 11 không thay thế được Ignition mà launch này sử dụng.

```bash
lsb_release -ds
test -f /opt/ros/humble/setup.bash && echo "Da co ROS 2 Humble"
df -h .
```

Nếu chưa có `/opt/ros/humble/setup.bash`, cần cài ROS 2 Humble và cấu hình nguồn
APT của ROS trước. Đây là điều kiện đầu vào của hướng dẫn; các lệnh cài thêm
dưới đây không tự cấu hình một máy Ubuntu trắng. Xem
[hướng dẫn cài Humble chính thức](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### 1.2. Cài các gói dùng trong quy trình

Bỏ qua nếu máy hiện tại đã cài đủ và bước 2 kiểm tra thành công. Trên máy có
ROS Humble và nguồn APT phù hợp, chạy:

```bash
sudo apt-get update
sudo apt-get install --no-install-recommends \
  python3-colcon-common-extensions python3-yaml python3-numpy python3-scipy unzip \
  ros-humble-ros-ign-gazebo ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2 \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-nav2-rviz-plugins
```

Nhập mật khẩu Ubuntu nếu được hỏi; terminal không hiện ký tự khi gõ mật khẩu.
Đợi cài xong và kiểm tra không có dòng `Unable to fetch` hoặc lỗi package.

Dung lượng tham khảo từ lần chuẩn bị đã ghi nhận: `simulations.zip` khoảng
7,63 MB; tệp tải về cùng mã nguồn giải nén khoảng 38 MB. Riêng đợt bổ sung bốn
gói mô phỏng trên máy cũ cần khoảng 47,4 MB tải về và 309,2 MB sau cài đặt.
**Đây không phải tổng dung lượng cài trên máy mới**; danh sách ở trên còn có
ROS navigation/SLAM và phụ thuộc. APT sẽ báo dung lượng thực tế trước khi xác
nhận cài. Chừa thêm chỗ cho build, log và video.

### 1.3. Đặt đúng dữ liệu của hãng

Kiểm tra trước để tránh tải lại:

```bash
ls -la third_party/rosorin_pro/original/
ls third_party/rosorin_pro/upstream/src/simulations/
```

Cấu trúc cần có:

```text
third_party/rosorin_pro/
├── original/
│   ├── simulations.zip
│   └── .typerc
└── upstream/src/simulations/
    ├── robot_gazebo/
    └── rosorin_description/
```

Nếu mới clone dự án, hai thư mục `original/` và `upstream/` không có trong Git.
Tải `simulations.zip` và `.typerc` từ liên kết Feature Package được ghi tại
[nguồn dữ liệu của hãng](third_party/rosorin_pro/README.md), rồi đặt vào
`third_party/rosorin_pro/original/`. Dùng Ctrl+H trong trình quản lý tệp để thấy
`.typerc` vì tên bắt đầu bằng dấu chấm.

```bash
mkdir -p third_party/rosorin_pro/original
mkdir -p third_party/rosorin_pro/upstream/src
sha256sum third_party/rosorin_pro/original/simulations.zip
sha256sum third_party/rosorin_pro/original/.typerc
unzip -t third_party/rosorin_pro/original/simulations.zip
```

So sánh hai mã SHA-256 với bảng trong tài liệu nguồn dữ liệu. Nếu khớp và kiểm
tra ZIP không lỗi, **chỉ giải nén khi chưa có mã nguồn**:

```bash
unzip third_party/rosorin_pro/original/simulations.zip \
  -d third_party/rosorin_pro/upstream/src
```

Không cần `source .typerc` hoặc chép nội dung của nó vào `.bashrc`; launch của
dự án đã đặt lựa chọn camera cần dùng. `.typerc` được giữ để tham chiếu cấu
hình hãng, không phải điều kiện chạy của wrapper này.

<a id="buoc-2"></a>
## 2. Build và kiểm tra gói

Tại thư mục gốc dự án:

```bash
bash part1_rosorin/scripts/build.sh
```

Đợi đến cuối và tìm dòng tổng kết **`2 packages finished`**, không có package
failed. Build tạo `build/`, `install/`, `log/` trong `part1_rosorin/ws/`.

```bash
bash part1_rosorin/scripts/check_runtime.sh
```

Kết quả mong đợi: các dòng `FOUND` và `Package lookup passed`. Script này kiểm
tra gói mô phỏng; kiểm tra thêm phần SLAM/Nav2 bằng:

```bash
source /opt/ros/humble/setup.bash
source part1_rosorin/ws/install/local_setup.bash
ros2 pkg prefix robot_gazebo
ros2 pkg prefix rosorin_description
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_bringup
ros2 pkg prefix nav2_map_server
ros2 pkg prefix nav2_simple_commander
ros2 pkg prefix nav2_rviz_plugins
```

Mỗi lệnh phải trả về đường dẫn. Hai gói của hãng phải trỏ về thư mục install
của dự án này. Nếu có `Package not found`, xử lý trước khi mở mô phỏng.
Việc tìm thấy gói chưa chứng minh mô phỏng chạy được; bước tiếp theo kiểm tra thực tế.

<a id="buoc-3"></a>
## 3. Chạy mô phỏng và xem cảm biến

### 3.1. Sao lưu kết quả cũ trước khi chạy lại

Các lệnh mặc định sẽ ghi đè bản đồ `room_slam` và một số JSON cùng tên trong
`results/runtime/`. Nếu muốn giữ lần đo trước, chạy một lần trong T1:

```bash
PART1_BACKUP="$HOME/rosorin_backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$PART1_BACKUP"
cp -a part1_rosorin/maps part1_rosorin/results "$PART1_BACKUP/"
echo "$PART1_BACKUP"
```

Lệnh tạo bản sao ngoài dự án, không xóa bản gốc. Thư mục results có thể chứa
nhiều log; kiểm tra dung lượng trống nếu bạn đã chạy nhiều lần.

### 3.2. T1 — mở Gazebo

Đảm bảo phiên mô phỏng trước đã dừng, rồi chạy:

```bash
bash part1_rosorin/scripts/run.sh simulation
```

Đợi cửa sổ Gazebo hiện căn phòng và robot. Đồng hồ mô phỏng phải tăng; nếu đang
Pause, bấm Play. Giữ T1 hoạt động. Dùng launch trên, không dùng trực tiếp
`ros2 launch robot_gazebo room_worlds.launch.py` vì bản hãng có lỗi phụ thuộc
`holonomic_sim` đã được xử lý trong wrapper của dự án.

### 3.3. T2 — mở RViz cảm biến

Mở terminal mới, `cd` về gốc dự án như bước 0, rồi chạy:

```bash
bash part1_rosorin/scripts/run.sh sensors
```

Trong RViz cần thấy robot và các điểm LiDAR. Fixed Frame của cấu hình này là
`odom`; chưa cần `map` vì SLAM chưa chạy. Nếu chưa thấy ngay, đợi robot spawn
và dữ liệu cảm biến xuất hiện.

### 3.4. T3 — kiểm tra tự động tiến, lùi, quay và LiDAR

Mở T3, `cd` về gốc dự án, chạy khi robot vừa spawn và xung quanh còn trống:

```bash
bash part1_rosorin/scripts/run.sh verify
```

Robot sẽ tiến, lùi, quay trái, quay phải với vận tốc thấp rồi dừng. Đây là
quay hướng, không phải trượt ngang. Mỗi hướng nhận lệnh trong 1,5 giây thời
gian mô phỏng; thời gian thực có thể dài hơn nếu máy chạy chậm.

Kết quả cần thấy: **`Verification PASS`**. Mở dữ liệu bằng:

```bash
python3 -m json.tool part1_rosorin/results/runtime/control_sensors.json
```

Trường `status` phải là `PASS`. Dữ liệu ghi nhận chuyển động, khoảng đo LiDAR
hợp lệ và các phép biến đổi TF. Nếu FAIL, xem `errors` và [mục lỗi](#buoc-9)
trước khi tiếp tục.

**Điều khiển tay, tùy chọn:** trong terminal trống chạy
`bash part1_rosorin/scripts/run.sh teleop`, nhấp chuột vào chính terminal đó
rồi dùng `w` tiến, `s` lùi, `a` quay trái, `d` quay phải. Nhả phím để dừng;
Ctrl+C kết thúc. Script hãng đặt tốc độ 1 m/s và 1,5 rad/s, nên bấm ngắn và
quan sát để tránh va vào đồ vật. **Không dùng teleop này để quét bản đồ**
(quá nhanh, bản đồ dễ bị nghiêng); xem mục 4.3b. Dừng teleop trước khi dùng `verify`, `sweep`
hoặc Nav2. Lần đầu nên theo bài kiểm tra tự động trước.

<a id="buoc-4"></a>
## 4. Tạo bản đồ, lưu bản đồ và vị trí robot

### 4.1. T2 — thay cửa sổ cảm biến bằng SLAM

Trong T2 đang chạy `sensors`, nhấn Ctrl+C để dừng RViz đó. Khi dấu nhắc trở
lại, chạy:

```bash
bash part1_rosorin/scripts/run.sh slam
```

Giữ T1 và T2 hoạt động.

### 4.2. T3 — mở RViz bản đồ

Sau khi `verify` đã kết thúc, chạy trong T3:

```bash
bash part1_rosorin/scripts/run.sh map-view
```

Fixed Frame lúc này là `map`. Bạn sẽ thấy vùng bản đồ quanh robot: vùng trống
thường sáng, vật cản tối, vùng chưa quan sát có màu xám. Thông báo chưa có TF
`map` lúc vừa khởi động có thể xuất hiện; nó phải hết khi SLAM nhận đủ dữ liệu.

### 4.3. T4 — quét quanh robot

Mở T4, `cd` về gốc dự án, chạy:

```bash
bash part1_rosorin/scripts/run.sh sweep
```

Robot quay chậm hơn một vòng, bản đồ cập nhật theo tia quét. Đợi script tự
kết thúc và in `"status": "PASS"`. Đừng chạy lệnh tiếp theo khi robot vẫn
đang quay. Sweep chỉ quan sát khu vực trung tâm, chưa khám phá toàn bộ phòng.
Muốn mở rộng bản đồ, có thể dùng teleop sau sweep; phải dừng teleop và robot
trước khi lưu ở bước kế tiếp.

### 4.3b. T4 — lái tay quét toàn bộ nhà (để có bản đồ đầy đủ)

`sweep` chỉ quét khu vực trung tâm. Muốn bản đồ đủ các phòng, lái tay chậm bằng
`teleop_twist_keyboard` (không dùng teleop của hãng, vì 1,5 rad/s làm SLAM khớp
sai và tường bị nghiêng hoặc chồng đôi):

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.2 -p turn:=0.4 -r cmd_vel:=/controller/cmd_vel
```

Bấm chuột vào chính terminal đó rồi dùng `i` tiến, `,` lùi, `j` quay trái, `l`
quay phải, `k` dừng. Giữ `speed` 0,2 m/s và `turn` 0,4 rad/s; không bấm `q`
(tăng tốc). Bảng Teleop trong Gazebo cũng dùng được nếu đặt Forward 0,2, Vertical
0, Yaw 0,3; chỉ dùng một nguồn điều khiển tại một thời điểm.

- Quay tại chỗ một vòng chậm trong mỗi phòng: LiDAR chỉ nhìn ±78° phía trước.
- Dừng 1–2 giây sau mỗi lần quay để SLAM bắt kịp; qua cửa (rộng khoảng 1 m) thật chậm.
- Đi vòng khép kín và quay về chỗ xuất phát để SLAM tự hiệu chỉnh.
- Quan sát RViz: tường phải là một nét mảnh, góc vuông. Nếu thấy tường chồng đôi
  hoặc cả khu bị xoay, dừng lại, tắt Gazebo và SLAM rồi quét lại từ đầu; bản đồ đã
  lệch không sửa được.

Xong thì Ctrl+C teleop, đợi robot đứng yên rồi sang mục 4.4.

### 4.4. T4 — lưu bản đồ rồi lưu vị trí

Chạy lần lượt, đợi mỗi lệnh kết thúc thành công mới chạy lệnh tiếp theo:

```bash
bash part1_rosorin/scripts/run.sh save-map
```

```bash
bash part1_rosorin/scripts/run.sh capture-pose
```

Kiểm tra:

```bash
ls -lh part1_rosorin/maps/room_slam.yaml part1_rosorin/maps/room_slam.pgm
cat part1_rosorin/maps/room_slam.yaml
python3 -m json.tool part1_rosorin/results/runtime/localization_initial_pose.json
```

`room_slam.yaml` chứa thông số bản đồ và đường dẫn ảnh; `room_slam.pgm` là ảnh
bản đồ. Luôn giữ hai tệp cùng nhau. JSON vị trí có các trường `x`, `y`, `qz`,
`qw` để đặt vị trí ban đầu cho AMCL trong chính bản đồ vừa tạo.

`capture-pose` là tùy chọn để lưu minh chứng hoặc dùng chế độ pose tường minh.
Luồng điều hướng mới không tự đọc tệp này. Chỉ khi dùng `--initial-pose`, cần
đảm bảo robot vẫn ở đúng vị trí đã capture; không dùng pose cũ sau reset/di chuyển.

<a id="buoc-5"></a>
## 5. Chuyển sang AMCL/Nav2 và chạy điều hướng

### 5.1. T2 — dừng SLAM, giữ mô phỏng

Nhấn Ctrl+C ở **T2 đang chạy `slam`** và đợi tiến trình kết thúc. Giữ T1 Gazebo
và T3 RViz đang chạy. SLAM và AMCL đều có thể cung cấp `map → odom`, vì vậy
cần kết thúc SLAM trước khi mở AMCL.

Trong T2, tạo cấu hình và mở điều hướng:

```bash
/usr/bin/python3 part1_rosorin/scripts/prepare_nav_config.py
bash part1_rosorin/scripts/run.sh navigation
```

Lệnh đầu tạo lại `config/nav2.yaml` từ cấu hình hãng cùng các chỉnh sửa của dự án
(bộ điều khiển Regulated Pure Pursuit, `xy_goal_tolerance` 0,15 m, nhiễu AMCL thấp;
xem [phân tích lỗi](part1_rosorin/docs/multi_waypoint_stall_fix.md)); nếu đã tự chỉnh
tệp này, hãy sao lưu trước vì nó bị ghi đè. Lệnh sau tải `maps/room_slam.yaml`, mở AMCL
và các node Nav2. Giữ T2 chạy. Bước 5.2 tự kiểm tra định vị hiện tại; nếu chưa
đạt thì yêu cầu AMCL tìm vị trí trên toàn bản đồ, không tự gửi pose lưu cũ.

### 5.2. T4 — khởi tạo vị trí và chạy bài kiểm tra

Ngay sau `navigation`, T2 in các dòng `AMCL cannot publish a pose` và
`Timed out waiting for transform ... map`: đây là bình thường, Nav2 đang chờ vị trí
ban đầu. Panel Navigation 2 trong RViz hiện `unknown` cho đến khi định vị xong.
Chạy ngay lệnh định vị (robot đứng yên, đợi `"status": "PASS"`):

```bash
bash part1_rosorin/scripts/run.sh localize
```

Sau đó T2 in `Managed nodes are active` và panel chuyển sang `active`. Mở RViz
(`map-view`) sau khi `navigation` đã chạy để RViz khỏi đơ; nếu RViz báo
"not responding", chọn Wait. Nếu T2 thật sự đứng ở "Configuring ..." quá ~30 giây
(lỗi khởi động Fast-DDS đôi khi gặp), Ctrl+C rồi chạy lại `navigation`.


```bash
bash part1_rosorin/scripts/run.sh trials
```

Script dùng chế độ `--localization auto`: giữ AMCL hiện tại nếu kiểm tra đạt,
nếu không thì tìm lại vị trí trên toàn bản đồ bằng LiDAR. Bước so khớp quét 2D
thử các vị trí/hướng trên vùng trống, tinh chỉnh ứng viên và chỉ dùng ứng viên
đủ khớp, khác biệt với nghiệm cạnh tranh để khởi tạo AMCL. AMCL vẫn phải vượt
qua kiểm tra sau đó. Cần `python3-numpy` và `python3-scipy` (nếu thiếu, cài bằng
`sudo apt-get install python3-numpy python3-scipy`). Robot đứng yên trong
giai đoạn tìm vị trí; chỉ gửi đích sau khi có các cập nhật định vị liên tiếp đạt
kiểm tra covariance và độ khớp LiDAR/bản đồ. Không dùng ground truth của Gazebo.
Các đích đọc từ [navigation_goals.yaml](part1_rosorin/config/navigation_goals.yaml), mặc định:

| Bài kiểm tra | Đích trong hệ tọa độ `map` (x, y, góc yaw) |
| --- | --- |
| Một điểm | `(0.8 m, 1.0 m, -π/2 rad)` |
| Ba điểm liên tiếp | `(0.5, 0.5, -π/2)` → `(0.2, -0.4, -π/2)` → `(1.0, 0.5, 0)` |

Quan sát đường đi trong RViz và chuyển động trong Gazebo. Các đích này được
chọn cho phòng và vùng trống trung tâm đã kiểm chứng; không mặc định phù hợp
với một world khác hoặc bản đồ có vật cản tại đó.

Định vị có timeout mặc định 90 giây; mỗi bài điều hướng có giới hạn 150 giây.
Nếu định vị không hội tụ, script ghi FAIL và không gửi đích. Bản đồ thiếu dữ
liệu, phòng đối xứng hoặc vật cản thay đổi có thể khiến vị trí không xác định
được từ một chỗ đứng. Không thể bảo đảm tự định vị đúng ở mọi nơi chỉ bằng
LiDAR; các kiểm tra chất lượng cũng không thay thế ground truth độc lập.

### 5.3. Đổi đích và chạy lại từ vị trí hiện tại

Chi tiết thay đổi, tiêu chí định vị và kết quả thử mới:
[ghi chú kiểm chứng tự định vị](part1_rosorin/docs/automatic_localization.md).

Chạy một đích với tọa độ x, y (m), hướng cuối (độ):

```bash
bash part1_rosorin/scripts/run.sh trials --task single --goal 0.8 1.0 -90
```

Hoặc sửa `single_goal` và `waypoints` trong `config/navigation_goals.yaml`:

```yaml
single_goal: {x: 0.8, y: 1.0, yaw_deg: -90}
waypoints:
  - {x: 0.5, y: 0.5, yaw_deg: -90}
  - {x: 0.2, y: -0.4, yaw_deg: -90}
  - {x: 1.0, y: 0.5, yaw_deg: 0}
```

```bash
bash part1_rosorin/scripts/run.sh trials --task single
bash part1_rosorin/scripts/run.sh trials --task waypoints
```

Chạy lần lượt, chờ lệnh trước xong. `--task both` (mặc định) chạy cả hai bài.
Tọa độ thuộc bản đồ đang tải; yaw 0° hướng +x, 90° hướng +y. Chọn vùng trống
và có đường đi; kiểm tra ô đích hợp lệ chưa đảm bảo có đường từ vị trí robot.
Không cần build lại sau khi sửa YAML. Dùng `--goals PATH` nếu muốn tệp riêng.

Nếu vừa kéo/teleport robot trong Gazebo, buộc tìm lại vị trí:

```bash
bash part1_rosorin/scripts/run.sh trials --localization global --task single --goal 0.8 1.0 -90
```

Nếu chỉ muốn tự định vị trước khi dùng nút Nav2 Goal trong RViz:

```bash
bash part1_rosorin/scripts/run.sh localize
```

`--localization current` chỉ kiểm tra và giữ AMCL, không reset toàn cục; phù hợp
khi đã định vị bằng RViz. Dừng teleop và action đang chạy trước khi dùng các
lệnh trên. Tự tìm vị trí diễn ra khi gọi `trials`/`localize`, không phải một
dịch vụ giám sát mất định vị liên tục trong lúc robot đang đi.

<a id="buoc-6"></a>
## 6. Kiểm tra kết quả và chọn đích bằng chuột

### 6.1. Đọc kết quả thật của lần chạy

Sau khi `trials` kết thúc, xem dòng `Result saved:` để lấy đường dẫn JSON mới.
Tên mặc định có thời gian `navigation_trials_YYYYMMDD_HHMMSS_ffffff.json`,
không ghi đè minh chứng cũ. Để chủ động đặt tên và đọc kết quả:

```bash
bash part1_rosorin/scripts/run.sh trials --output part1_rosorin/results/runtime/my_run.json
python3 -m json.tool part1_rosorin/results/runtime/my_run.json
```

Một lần đạt yêu cầu có:

- `status: "PASS"` ở cấp ngoài cùng.
- Với `--task both`, hai mục `single_goal`, `multi_point` đều có `action_result: "SUCCEEDED"`
  và `pass: true`.
- `missed_waypoints: []` trong bài nhiều điểm.
- `final_position_error_m` không quá 0,35 m theo điều kiện kiểm tra của script.

`wall_seconds` là thời gian thực; `simulation_seconds` là thời gian trong mô
phỏng. Sai số vị trí được tính từ AMCL/TF so với đích, không phải ground truth
độc lập. Hai phiên đã ghi nhận trước đó có bốn bài đạt, sai số cuối khoảng
0,217–0,248 m (đo trước bản sửa động học/bộ điều khiển ngày 2026-10-17); lần chạy
mới phải đọc JSON mới, không mặc định có cùng kết quả.

### 6.2. Chọn một đích trong RViz, tùy chọn

Thực hiện sau khi bài kiểm tra đã kết thúc và Nav2 vẫn chạy:

1. Kiểm tra bảng **Navigation 2** cho thấy localization/navigation đã active.
2. Nếu vừa chạy `trials`, vị trí đã được khởi tạo. Không đặt lại vị trí nếu
   robot đang khớp bản đồ.
3. Bấm **Nav2 Goal** trên thanh công cụ. Trong vùng trống đã biết của bản đồ,
   nhấn giữ chuột trái tại đích, kéo theo hướng robot cần quay khi tới nơi,
   rồi thả chuột.
4. Quan sát đường đi và chờ robot đến đích trước khi chọn đích khác.

**Nhiều điểm bằng chuột:** bấm **Waypoint / Nav Through Poses Mode**, bấm **Nav2 Goal**
và đặt lần lượt từng điểm, rồi bấm **Start Waypoint Following** (không bấm
*Start Nav Through Poses*). **Cancel** để hủy. RViz không kiểm tra đích: chọn vùng
trống, cách vật cản từ 0,3 m trở lên; đích nằm trong đám chấm rải rác trên bản đồ
sẽ bị Nav2 bỏ qua (`failed to create plan`).

Nếu khởi tạo thủ công thay cho `trials`, dùng **2D Pose Estimate** trước:
đặt mũi tên tại vị trí thực của robot trên bản đồ, kéo đúng hướng robot đang
quay. Các tia LiDAR cần khớp tương đối với tường. Chọn sai vị trí/hướng có
thể làm robot định vị sai. Bài ba điểm ở bước 5 dùng action FollowWaypoints
tự động, thuận tiện hơn việc phụ thuộc vào bố cục nút của từng phiên bản RViz.

<a id="buoc-7"></a>
## 7. Tắt chương trình và chạy lại

Khi bài kiểm tra đã xong, robot đã dừng:

1. Dừng teleop nếu còn mở.
2. Ctrl+C ở T2 để dừng Nav2 (hoặc SLAM nếu chưa chuyển giai đoạn).
3. Ctrl+C ở T3 để đóng RViz.
4. Ctrl+C ở T1 để dừng Gazebo. Đợi tiến trình kết thúc trước khi mở lại.

Nếu cần dừng giữa một bài điều hướng, dùng **Cancel** trong bảng Navigation 2
nếu có action đang chạy, dừng script `trials`, rồi dừng Nav2. Khi có bất thường,
có thể Pause Gazebo để tạm ngừng chuyển động mô phỏng trong lúc xử lý.

**Lần đầu chạy lại:** làm từ bước 3 đến 6, tạo lại bản đồ và capture vị trí mới.
Đây là trình tự dễ kiểm soát nhất. Bản đồ và JSON cùng tên sẽ bị ghi đè, nên
thực hiện sao lưu nếu cần giữ dữ liệu trước.

### Dùng bản đồ có sẵn sau khi spawn mới — cách nâng cao

Chỉ dùng khi world giữ nguyên và bạn biết vị trí spawn nằm gần `(0, 0, 0)`
trong hệ tọa độ của bản đồ đã lưu. Với bản đồ tạo từ vị trí/hướng khác, cần
ước lượng lại pose; `start_pose.json` không tự căn chỉnh hai hệ tọa độ.

Mở `simulation`, `map-view`, `navigation` ở ba terminal riêng, bỏ qua SLAM.
Trong terminal thứ tư, khi robot vẫn ở vị trí spawn và chưa chạy verify/teleop:

```bash
source /opt/ros/humble/setup.bash
source part1_rosorin/ws/install/local_setup.bash
/usr/bin/python3 part1_rosorin/scripts/navigation_trials.py \
  --initial-pose part1_rosorin/config/start_pose.json \
  --output part1_rosorin/results/runtime/navigation_trials_fresh_spawn.json
```

Đây là cách gọi hỗ trợ pose đầu vào khác, không phải kết quả đã được kiểm
chứng cho mọi bản đồ. Nếu không chắc hệ tọa độ khớp, ưu tiên tạo lại bản đồ
theo quy trình chính hoặc dùng chế độ tự định vị toàn cục ở mục 5.3. Luồng
`trials` mới không ghi đè pose thủ công trừ khi bạn truyền `--initial-pose`;
dùng `--localization current` nếu muốn cấm cả việc reset AMCL toàn cục.

<a id="buoc-8"></a>
## 8. Tệp kết quả, báo cáo và quản lý GitHub

Báo cáo Part 1 được nộp chung với Part 2 nên không nằm trong repo này. Số liệu
nguồn của báo cáo nằm trong `part1_rosorin/results/` và `part1_rosorin/docs/`.

| Đường dẫn tính từ gốc dự án | Nội dung |
| --- | --- |
| `part1_rosorin/launch/` | Launch tích hợp mô phỏng, SLAM, Nav2 |
| `part1_rosorin/config/` | Cấu hình RViz, Nav2 và pose spawn tham khảo |
| `part1_rosorin/scripts/` | Script build, chạy, kiểm tra và tổng hợp kết quả |
| `part1_rosorin/maps/room_slam.yaml` và `.pgm` | Bản đồ vừa lưu |
| `part1_rosorin/results/runtime/control_sensors.json` | Kiểm tra chuyển động, LiDAR, TF |
| `part1_rosorin/results/runtime/mapping_sweep.json` | Kết quả quay quét |
| `part1_rosorin/results/runtime/localization_initial_pose.json` | Vị trí bàn giao SLAM → AMCL |
| `part1_rosorin/results/runtime/navigation_trials_*.json` | Kết quả mới, mỗi lần một tệp có thời gian |
| `part1_rosorin/results/runtime/ros_logs/` | Log ROS để chẩn đoán |
| `part1_rosorin/results/runtime/stall_fix/` | Hiệu chuẩn bánh xe và kết quả sau khi sửa lỗi |
| `demos/` | Video demo (`.webm`) và ghi chú, xem [demos/README.md](demos/README.md) |

`run.sh trials` và `run.sh localize` chuyển tiếp các tham số thêm. Các chế độ
khác giữ cách gọi cũ. Ví dụ muốn đổi tên kết quả và dùng pose biết chính xác,
có thể gọi trực tiếp Python sau khi source môi trường:

```bash
source /opt/ros/humble/setup.bash
source part1_rosorin/ws/install/local_setup.bash
/usr/bin/python3 part1_rosorin/scripts/navigation_trials.py \
  --initial-pose part1_rosorin/results/runtime/localization_initial_pose.json \
  --output part1_rosorin/results/runtime/navigation_trials_run03.json
```

Ví dụ trên dành cho thời điểm bàn giao ngay sau capture-pose, khi robot còn
đứng tại pose đó. Nếu robot đã chạy đến đích, không dùng lại pose ban đầu để
khởi tạo AMCL. Các script `capture_pose.py`, `mapping_sweep.py` và
`verify_control_sensors.py` cũng có `--output`; thư mục cha nên được tạo sẵn.

Nếu cần tên bản đồ khác, khi SLAM còn chạy:

```bash
source /opt/ros/humble/setup.bash
source part1_rosorin/ws/install/local_setup.bash
ros2 run nav2_map_server map_saver_cli -f part1_rosorin/maps/room_run03 \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
```

Sau khi đã capture pose tương ứng và dừng SLAM, dùng lệnh sau **thay cho**
`run.sh navigation` để tải đúng bản đồ:

```bash
ros2 launch part1_rosorin/launch/navigation.launch.py \
  map:="$PWD/part1_rosorin/maps/room_run03.yaml"
```

### Tạo lại biểu đồ tổng hợp

Đây là bước tùy chọn, sau khi có đủ dữ liệu. Script đọc bộ tệp chuẩn của hai lần
chạy đã ghi nhận, bao gồm tên `*_demo`; nó không tự tìm mọi JSON mới như
`navigation_trials_run03.json`. Kiểm tra đầu vào trước khi tổng hợp để không trộn
bản đồ/vị trí/kết quả của các phiên khác nhau.

Nếu thiếu thư viện tổng hợp kết quả:

```bash
sudo apt-get install python3-yaml python3-numpy python3-pil \
  python3-matplotlib
```

```bash
/usr/bin/python3 part1_rosorin/scripts/summarize_results.py
```

Lệnh trên ghi lại CSV và hình tổng hợp theo dữ liệu đầu vào, không tự chạy
lại mô phỏng. Hai video demo và ghi chú nằm tại [thư mục demos](demos/README.md).

### Những gì nên đưa lên GitHub

Giữ source, launch, config, README, cặp bản đồ YAML/PGM, kết quả JSON/CSV chọn
lọc và tài liệu minh chứng. `.gitignore` đã loại trừ bản tải hãng, mã nguồn
hãng giải nén, build/install/log, log ROS nội bộ, video MP4 và các JSON kết quả chạy thử. Người clone mới
cần tải dữ liệu hãng lại theo manifest ở bước 1.3.

Hai video `.webm` trong `demos/` (tổng ~62 MB) đã nằm trong repository. Trước khi tự
commit, kiểm tra danh sách tệp thay đổi để không vô tình đưa bản sao lưu hoặc log
lớn vào repository.

<a id="buoc-9"></a>
## 9. Xử lý lỗi thường gặp

Chạy các lệnh chẩn đoán trong một terminal trống ở gốc dự án, sau khi source
hai tệp môi trường như bước 0. Lệnh nghe topic/TF có thể chạy liên tục; Ctrl+C
để thoát khi đã quan sát đủ.

| Hiện tượng | Kiểm tra và cách xử lý |
| --- | --- |
| `No such file or directory` khi chạy script | Dùng `pwd`, chuyển về gốc dự án bằng `cd`; không đứng trong `part1_rosorin/`. |
| Thiếu `ws/install/local_setup.bash` | Build ở bước 2 chưa thành công hoặc chưa chạy. Xem `part1_rosorin/results/audit/build.log`. |
| `ros2: command not found` | Source `/opt/ros/humble/setup.bash` trong chính terminal đang dùng. Nếu tệp không tồn tại, cần cài ROS trước. |
| `Package ... not found` | Source overlay, chạy lại check và kiểm tra gói ở bước 2. Thiếu gói nào thì cài đúng gói đó. |
| APT báo `Unable to fetch some archives` | Chạy `sudo apt-get update`, kiểm tra mạng và lỗi nguồn APT, rồi chạy lại lệnh cài. Tải được một phần chưa đồng nghĩa đã cài đủ. |
| `python: command not found` hoặc import ROS thất bại | Thoát Conda/venv trong terminal ROS, dùng `/usr/bin/python3` và source môi trường. Không cài torch để sửa lỗi ROS. |
| `holonomic_sim not found` / `no ros2_control tag` | Kiểm tra đang dùng `run.sh simulation`, không phải launch gốc của hãng hoặc một phiên cũ. |
| Gazebo mở nhưng robot không chạy | Kiểm tra Play/Pause, `/clock`, `/odom`; dừng nguồn điều khiển khác trước khi thử. |
| RViz báo thiếu `map` ở bước cảm biến | Dùng `run.sh sensors` với frame `odom`. `map-view` cần SLAM hoặc AMCL đã định vị. |
| Không thấy tia LiDAR | Kiểm tra `/scan`, bật LaserScan trong RViz và QoS Best Effort; cấu hình có sẵn đã đặt như vậy. |
| `capture-pose` báo không có TF | Giữ SLAM chạy và đợi có bản đồ; không dừng SLAM trước bước capture. |
| Nav2 cứ đợi active/AMCL | Kiểm tra log T2, bản đồ có đủ YAML/PGM, `/clock` tăng, `/scan` có dữ liệu, SLAM đã dừng. Không mở thêm Nav2/trials. |
| Robot lệch tường trên RViz | Pose ban đầu có thể sai; kiểm tra có reset/di chuyển sau capture không. Đặt lại pose đúng bằng RViz hoặc làm lại quy trình chính. |
| Robot quay tại chỗ, Nav2 báo `Failed to make progress` và tăng "Number of recoveries" dù đường đi xanh không sát tường | Đây là lỗi đã sửa ngày 2026-10-17 (hệ số bánh MecanumDrive sai và DWB kẹt ở đường chữ U quanh vách). Dùng `run.sh simulation` mới và `nav2.yaml` do `prepare_nav_config.py` sinh, rồi **khởi động lại cả Gazebo và Nav2**. Chi tiết: [multi_waypoint_stall_fix.md](part1_rosorin/docs/multi_waypoint_stall_fix.md). |
| Planner báo `failed to create plan` cho một đích | Đích nằm trong vùng không tới được (sát vật cản hoặc trong đám chấm rải rác trên bản đồ). RViz không kiểm tra đích; `trials` có kiểm tra. Đổi đích sang vùng trống cách vật cản ≥ 0,3 m. |
| `Localization did not converge` | Robot đang ở nơi ít đặc trưng (ví dụ góc phòng). Không gửi đích; đưa robot về gần vị trí xuất phát hoặc dùng 2D Pose Estimate rồi chạy lại. |
| Bản đồ SLAM bị nghiêng, tường chồng đôi | Thường do quay quá nhanh khi lái tay (teleop hãng 1,5 rad/s). Tắt Gazebo và SLAM, quét lại bằng teleop chậm ở mục 4.3b. Khôi phục bản đồ tốt từ bản sao lưu ở mục 3.1 nếu cần. |
| RViz báo `rviz2 is not responding` | Chọn Wait; Nav2 không phụ thuộc RViz. Nếu vẫn đơ, đóng RViz và mở lại `map-view` sau khi Nav2 đã chạy. |
| Action FAIL/timeout | Đọc JSON và log Nav2, kiểm tra đích có nằm trong vùng trống, pose khớp bản đồ và máy có chạy mô phỏng đủ nhanh. Không sửa JSON thành PASS. |
| Cảnh báo EGL/đồ họa | Trên máy đã thử, từng có cảnh báo EGL nhưng vẫn hiển thị và nhận LiDAR. Nếu thực sự không có hình/dữ liệu, cần kiểm tra riêng; cảnh báo đơn lẻ không kết luận GPU training hỏng. |
| Lỗi trùng node hoặc dữ liệu bất thường | Kiểm tra có phiên Gazebo/SLAM/Nav2 cũ còn chạy, dừng bằng Ctrl+C ở đúng terminal rồi khởi động một phiên. |

Các lệnh quan sát hữu ích:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic hz /scan
```

`ros2 topic hz /scan` chạy liên tục, nên dừng bằng Ctrl+C trước lệnh khác.
Tốc độ theo thời gian thực có thể thấp hơn khi mô phỏng chậm.

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Lệnh trên phải hiện phép biến đổi khi mô phỏng đang chạy. Trong giai đoạn
SLAM hoặc AMCL đã khởi tạo, kiểm tra thêm:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Nếu cần nhờ hỗ trợ, gửi tên bước đang làm, lệnh vừa chạy, phần lỗi cuối terminal
và JSON của lần lỗi nếu có. Điều đó giúp phân biệt lỗi cài đặt, cảm biến, TF,
định vị và điều hướng.
