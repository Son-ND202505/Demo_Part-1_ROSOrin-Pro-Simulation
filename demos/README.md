# Part 1 demo

Hai video quay trực tiếp cửa sổ Gazebo, RViz và terminal trong lần chạy ngày
2026-10-17 (không dựng hình, không có thuyết minh):

| Video | Thời lượng | Nội dung |
| --- | --- | --- |
| [`Demo_part1_1.webm`](Demo_part1_1.webm) | ~3 phút 32 giây | SLAM Toolbox tạo bản đồ: lái tay chậm bằng `teleop_twist_keyboard`, bản đồ hiện dần trong RViz. |
| [`demo_part1.webm`](demo_part1.webm) | ~7 phút 57 giây | Điều hướng tự động bằng AMCL/Nav2 trên bản đồ đã lưu: định vị, điều hướng một điểm và nhiều điểm bằng lệnh `run.sh trials` (kết quả `PASS`), quan sát đường đi trong RViz. |

Các lệnh tương ứng nằm trong [README](../README.md) (mục 3–6). Lỗi điều hướng nhiều
điểm gặp trong quá trình quay và cách sửa được ghi trong
[phân tích lỗi](../part1_rosorin/docs/multi_waypoint_stall_fix.md).

Các tệp `.mp4` cũ (bản ghi ngày 2026-10-13) chỉ lưu cục bộ và bị `.gitignore` loại trừ.
