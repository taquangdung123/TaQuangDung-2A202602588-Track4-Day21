"""Demo CP2 topic D: voxel downsample -> tách mặt đất (RANSAC), vẽ BEV trước/sau từng bước.

Chạy từ gốc repo:
    python -m src.obstacle_demo --frame 000011
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from starter.datasets import load_frame


def to_pcd(xyz: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd


def downsample_and_remove_ground(points: np.ndarray, voxel_size: float, dist_thr: float, seed: int = 0):
    """points (N, >=3) velodyne -> (xyz_down, ground_mask trên xyz_down, plane_model)."""
    xyz = points[:, :3]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    down = np.asarray(to_pcd(xyz).voxel_down_sample(voxel_size).points)
    o3d.utility.random.seed(seed)
    plane, inliers = to_pcd(down).segment_plane(distance_threshold=dist_thr, ransac_n=3, num_iterations=1000)
    ground = np.zeros(len(down), dtype=bool)
    ground[inliers] = True
    return xyz, down, ground, plane


def bev(ax, xyz, c, title, s=0.2):
    cmap = dict(cmap="turbo", vmin=-2.0, vmax=1.0) if isinstance(c, np.ndarray) else {}  # tô màu theo độ cao z
    ax.scatter(xyz[:, 0], xyz[:, 1], s=s, c=c, linewidths=0, **cmap)
    ax.set(xlim=(0, 60), ylim=(-30, 30), aspect="equal", title=title, xlabel="x (m)", ylabel="y (m)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="data/kitti_mini")
    ap.add_argument("--frame", default="000011")
    ap.add_argument("--voxel-size", type=float, default=0.1)
    ap.add_argument("--dist-thr", type=float, default=0.2)
    ap.add_argument("--out-dir", default="results/figures")
    a = ap.parse_args()

    fr = load_frame(a.data_root, a.frame)
    raw, down, ground, plane = downsample_and_remove_ground(fr["points"], a.voxel_size, a.dist_thr)
    obst = down[~ground]

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    bev(axes[0], raw, raw[:, 2], f"Gốc: {len(raw)} điểm")
    bev(axes[1], down, down[:, 2], f"Voxel {a.voxel_size} m: {len(down)} điểm")
    bev(axes[2], down[ground], "lightgray", "", s=0.1)
    bev(axes[2], obst, "red", f"RANSAC thr={a.dist_thr} m: ground={ground.sum()}, còn lại={len(obst)}")
    fig.suptitle(f"{a.frame}  plane: {np.round(plane, 3).tolist()}")
    fig.tight_layout()
    out = Path(a.out_dir) / f"d_ground_{a.frame}_v{a.voxel_size}_t{a.dist_thr}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"frame={a.frame} raw={len(raw)} down={len(down)} ground={ground.sum()} "
          f"non_ground={len(obst)} plane={np.round(plane, 3).tolist()} -> {out}")


if __name__ == "__main__":
    main()
