"""Topic D: quét distance_threshold (RANSAC) và eps (DBSCAN), đo vật cản còn giữ được theo class.

Pipeline: voxel downsample -> RANSAC tách mặt đất (numpy, seed cố định) -> DBSCAN (open3d).
Mỗi lần chỉ đổi 1 tham số, tham số còn lại giữ ở mức mốc (dist_thr=0.1 m, eps=0.5 m).

Chạy từ gốc repo:
    python -m src.exp_obstacle_sweep
    python -m src.exp_obstacle_sweep --latency-reps 20      # thêm đo latency p50/p95 (ghi file riêng)
"""
from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from starter.datasets import load_frame
from starter.projection import velo_to_cam

CLASS_GROUP = {"Pedestrian": "Pedestrian", "Person_sitting": "Pedestrian", "Cyclist": "Cyclist",
               "Car": "Car", "Van": "Car", "Truck": "Car"}
ROI = dict(x=(0.0, 40.0), y=(-20.0, 20.0))   # vùng quan tâm phía trước xe (velodyne frame, mét)
LOW_H = 0.5                                   # lát thấp 0–0.5 m trên mặt đất (vật thấp sát đất)
MIN_PTS = 10                                  # số điểm tối thiểu để tính là "còn thấy vật" / cluster hợp lệ


def points_in_box(points_cam: np.ndarray, obj) -> np.ndarray:
    """Mask (N,) các điểm (camera frame) nằm trong 3D box của label (giống script mẫu topic A)."""
    h, w, l = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    local = (points_cam - obj.location) @ R
    return ((np.abs(local[:, 0]) <= l / 2) & (local[:, 1] <= 0) & (local[:, 1] >= -h)
            & (np.abs(local[:, 2]) <= w / 2))


def voxel_down(xyz: np.ndarray, voxel: float) -> np.ndarray:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    return np.asarray(pcd.voxel_down_sample(voxel).points)


def ransac_ground(xyz: np.ndarray, dist_thr: float, iters: int = 500, seed: int = 0):
    """RANSAC mặt phẳng gần nằm ngang (|n_z| > 0.9), seed cố định.

    Trả về (mask ground (N,), độ cao có dấu của mỗi điểm so với mặt phẳng (N,), hướng lên trên).

    Không dùng open3d.segment_plane vì chạy đa luồng nên mỗi lần ra kết quả hơi khác (thấy ở CP2).
    """
    rng = np.random.default_rng(seed)
    tri = xyz[rng.integers(0, len(xyz), size=(iters, 3))]               # (iters, 3, 3) bộ 3 điểm ngẫu nhiên
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    norm = np.linalg.norm(n, axis=1)
    ok = norm > 1e-9
    n[ok] /= norm[ok, None]
    ok &= np.abs(n[:, 2]) > 0.9
    d = -(n * tri[:, 0]).sum(axis=1)
    sub = xyz[rng.choice(len(xyz), size=min(8000, len(xyz)), replace=False)]
    score = (np.abs(sub @ n.T + d) < dist_thr).sum(axis=0)              # số inlier trên tập con
    score[~ok] = -1
    best = int(np.argmax(score))
    sign = 1.0 if n[best, 2] > 0 else -1.0
    height = sign * (xyz @ n[best] + d[best])
    return np.abs(height) < dist_thr, height


def dbscan(xyz: np.ndarray, eps: float, min_points: int = 5) -> np.ndarray:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    return np.asarray(pcd.cluster_dbscan(eps=eps, min_points=min_points))


def in_roi(xyz: np.ndarray) -> np.ndarray:
    return ((xyz[:, 0] >= ROI["x"][0]) & (xyz[:, 0] <= ROI["x"][1])
            & (xyz[:, 1] >= ROI["y"][0]) & (xyz[:, 1] <= ROI["y"][1]))


def pipeline(xyz: np.ndarray, voxel: float, dist_thr: float, eps: float):
    down = voxel_down(xyz, voxel)
    ground, height = ransac_ground(down, dist_thr)
    keep = ~ground & in_roi(down)
    labels = dbscan(down[keep], eps) if keep.any() else np.zeros(0, int)
    return down, ground, height, keep, labels


def run_one(fr: dict, voxel: float, dist_thr: float, eps: float) -> tuple[dict, list[dict]]:
    xyz = fr["points"][:, :3]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    down, ground, height, keep, labels = pipeline(xyz, voxel, dist_thr, eps)
    obst = down[keep]

    # Thống kê cluster: chỉ tính cluster >= MIN_PTS điểm
    ids, counts = np.unique(labels[labels >= 0], return_counts=True)
    big = ids[counts >= MIN_PTS]
    near = min((np.linalg.norm(obst[labels == c, :2], axis=1).min() for c in big), default=np.nan)

    # Đối chiếu với label: điểm trong 3D box trước/sau tách mặt đất, và cluster lớn nhất chứa chúng
    cam_down = velo_to_cam(down, fr["calib"])
    cluster_of = np.full(len(down), -1)
    cluster_of[keep] = labels
    objs = []
    for i, obj in enumerate(fr["labels"]):
        grp = CLASS_GROUP.get(obj.type)
        # location là camera frame: z_cam = phía trước (x velo), -x_cam = bên trái (y velo)
        if grp is None or not in_roi(np.array([[obj.location[2], -obj.location[0], 0.0]]))[0]:
            continue
        inbox = points_in_box(cam_down, obj)
        before = int(inbox.sum())
        if before < MIN_PTS:            # vật quá ít điểm ngay từ đầu: không đánh giá
            continue
        kept = inbox & ~ground
        _, cnt = np.unique(cluster_of[kept & (cluster_of >= 0)], return_counts=True)
        best = int(cnt.max()) if len(cnt) else 0
        # "Lát thấp": điểm của vật cao 0–LOW_H m trên mặt đất, đại diện cho vật thấp (pallet, người ngồi)
        low = inbox & (height >= 0) & (height <= LOW_H)
        n_low = int(low.sum())
        objs.append({"object_id": i, "type": obj.type, "class_group": grp,
                     "distance_m": round(float(np.hypot(obj.location[0], obj.location[2])), 1),
                     "height_m": round(float(obj.dimensions[0]), 2),
                     "pts_before": before, "pts_after_ground": int(kept.sum()),
                     "kept_ratio": round(int(kept.sum()) / before, 4),
                     "low_pts_before": n_low,
                     "low_kept_ratio": round(int((low & ~ground).sum()) / n_low, 4) if n_low else float("nan"),
                     "best_cluster_pts": best, "detected": int(best >= MIN_PTS)})
    summary = {"n_down": len(down), "n_ground": int(ground.sum()), "n_obstacle_roi": len(obst),
               "n_clusters": len(big), "nearest_cluster_m": round(float(near), 2)}
    return summary, objs


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"-> {path} ({len(rows)} dòng)")


def configs(args) -> list[tuple[str, float, float]]:
    """(tham số đang quét, dist_thr, eps): mỗi lần chỉ đổi 1 tham số so với mốc (base_dist, base_eps)."""
    out = [("dist_thr", t, args.base_eps) for t in args.dist_levels]
    out += [("eps", args.base_dist, e) for e in args.eps_levels if e != args.base_eps]
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--data-root", default="data/kitti_mini")
    ap.add_argument("--frames", nargs="+", default=["000011", "000015", "000019", "000004"])
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument("--dist-levels", nargs="+", type=float, default=[0.1, 0.2, 0.3, 0.4])
    ap.add_argument("--eps-levels", nargs="+", type=float, default=[0.3, 0.5, 0.8])
    ap.add_argument("--base-dist", type=float, default=0.1)
    ap.add_argument("--base-eps", type=float, default=0.5)
    ap.add_argument("--latency-reps", type=int, default=0, help=">0: đo latency, bỏ lần đầu, báo p50/p95")
    ap.add_argument("--out", default="results/obstacle_sweep.csv")
    ap.add_argument("--out-objects", default="results/obstacle_sweep_objects.csv")
    ap.add_argument("--out-latency", default="results/obstacle_latency.csv")
    args = ap.parse_args()

    rows, obj_rows, lat_rows = [], [], []
    for frame in args.frames:
        fr = load_frame(args.data_root, frame)
        xyz = fr["points"][:, :3]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        for sweep, dist_thr, eps in configs(args):
            cfg = {"dataset": Path(args.data_root).name, "frame": frame, "sweep": sweep,
                   "voxel": args.voxel, "dist_thr": dist_thr, "eps": eps}
            summary, objs = run_one(fr, args.voxel, dist_thr, eps)
            n = len(objs)
            row = {**cfg, **summary, "n_objects": n,
                   "detected_ratio": round(sum(o["detected"] for o in objs) / n, 4) if n else float("nan"),
                   "mean_kept_ratio": round(float(np.mean([o["kept_ratio"] for o in objs])), 4) if n else float("nan"),
                   "mean_low_kept_ratio": round(float(np.nanmean([o["low_kept_ratio"] for o in objs])), 4) if n else float("nan")}
            rows.append(row)
            obj_rows += [{**cfg, **o} for o in objs]
            print(row)
            if args.latency_reps > 0:
                ts = []
                for _ in range(args.latency_reps + 1):
                    t0 = time.perf_counter()
                    pipeline(xyz, args.voxel, dist_thr, eps)
                    ts.append(time.perf_counter() - t0)
                ts = np.array(ts[1:]) * 1000                      # bỏ lần chạy đầu (warm-up), đổi ra ms
                lat_rows.append({**cfg, "reps": args.latency_reps,
                                 "p50_ms": round(float(np.percentile(ts, 50)), 1),
                                 "p95_ms": round(float(np.percentile(ts, 95)), 1)})
                print("   latency p50 =", lat_rows[-1]["p50_ms"], "ms")

    write_csv(Path(args.out), rows)
    write_csv(Path(args.out_objects), obj_rows)
    if lat_rows:
        write_csv(Path(args.out_latency), lat_rows)


if __name__ == "__main__":
    main()
