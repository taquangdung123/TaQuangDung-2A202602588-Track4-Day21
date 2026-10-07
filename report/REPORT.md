# Báo cáo Day 6: Ngưỡng RANSAC và vật cản thấp sát đất

> Thay **mọi** ô có chữ ĐIỀN nằm trong ngoặc vuông bằng nội dung của bạn, xoá luôn cả dấu ngoặc vuông. Lệnh `python tools/check_submission.py` sẽ báo FAIL nếu còn sót bất kỳ chỗ nào.

- **Họ tên:** Tạ Quang Dũng
- **MSSV:** 2A202602588
- **Lớp:** AI20K-T4
- **Link repo:** https://github.com/taquangdung123/TaQuangDung-2A202602588-Track4-Day21.git
- **Topic:** D — Robot/drone obstacle
- **Dataset:** data/kitti_mini
- **Các frame đã dùng:** 000011 (nhiều người đi bộ), 000019 (vật rất gần < 6 m), 000004 (xe xa > 50 m)

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

Một câu khẳng định kỹ thuật có thể kiểm chứng. Ví dụ: *"Lệch yaw 1° làm 12% điểm LiDAR rơi ra khỏi vật thể ở 30 m, phát hiện được bằng edge-alignment score với ngưỡng X."*

**Claim (nháp, CP1):** Trên 3 frame KITTI 000011, 000019, 000004 (voxel_size = 0.1 m, DBSCAN eps = 0.5 m), tăng `distance_threshold` của RANSAC ground removal từ 0.1 m lên 0.3 m làm người đi bộ và cyclist trong label mất hơn 30% số điểm sau bước tách mặt đất, và làm tỉ lệ vật được phát hiện thành cluster (tâm cluster cách tâm GT box < 1 m) giảm hơn 20 điểm phần trăm, trong khi với xe con chỉ giảm dưới 5 điểm phần trăm.

## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

| Cấu hình / mức perturb | Metric 1 | Metric 2 | Ghi chú |
|---|---|---|---|
| [ĐIỀN] | | | |

![demo](../results/figures/[ĐIỀN].png)

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch.

```bash
python -m venv venv && venv\Scriptsctivate      # Windows; Linux/macOS: source venv/bin/activate
set PYTHONUTF8=1                                    # PowerShell: $env:PYTHONUTF8="1" (requirements.txt có tiếng Việt)
pip install -r requirements.txt "open3d>=0.18"

# CP2: kiểm tra phép chiếu + overlay
python -m src.test_projection
python -m starter.projection --data-root data/kitti_mini --frame 000011

# CP2: demo topic D — voxel downsample + tách mặt đất RANSAC (BEV trước/sau)
python -m src.obstacle_demo --frame 000011
python -m src.obstacle_demo --frame 000019
python -m src.obstacle_demo --frame 000004
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
