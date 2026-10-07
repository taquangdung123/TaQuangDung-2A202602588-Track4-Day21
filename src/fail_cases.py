"""Topic D, CP4: vẽ 2 failure case của pipeline RANSAC + DBSCAN.

fail_01: frame 000004, dist_thr = 0.1 m -> mặt đường thấp hơn mặt phẳng RANSAC bị coi là vật cản gần nhất (4.02 m).
fail_02: frame 000011, người đi bộ bị che ở 14.5 m: dist_thr 0.1 -> 0.2 m thì không còn thành cluster.

Chạy từ gốc repo: python -m src.fail_cases
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.exp_obstacle_sweep import MIN_PTS, pipeline, points_in_box
from starter.datasets import load_frame
from starter.projection import project_velo_to_image, velo_to_cam

OUT = Path("results/figures")
VOXEL, EPS = 0.1, 0.5


def big_clusters(keep, labels):
    """Mảng (N_down,) id cluster (>= MIN_PTS điểm), -1 nếu không thuộc cluster nào."""
    ids, cnt = np.unique(labels[labels >= 0], return_counts=True)
    cl = np.full(len(keep), -1)
    good = np.isin(labels, ids[cnt >= MIN_PTS])
    idx = np.flatnonzero(keep)
    cl[idx[good]] = labels[good]
    return cl


def nearest_cluster(down, cl):
    best = (np.inf, -1)
    for c in np.unique(cl[cl >= 0]):
        best = min(best, (float(np.linalg.norm(down[cl == c, :2], axis=1).min()), int(c)))
    return best


def fail_01() -> None:
    fr = load_frame("data/kitti_mini", "000004")
    xyz = fr["points"][:, :3]
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), gridspec_kw={"height_ratios": [1.2, 1]})
    for col, thr in enumerate([0.1, 0.2]):
        down, ground, height, keep, labels = pipeline(xyz, VOXEL, thr, EPS)
        cl = big_clusters(keep, labels)
        dist, cid = nearest_cluster(down, cl)
        near = cl == cid
        p = down[near]
        print(f"fail_01 thr={thr}: nearest cluster {dist:.2f} m, {near.sum()} điểm, "
              f"kích thước {np.round(p.max(0) - p.min(0), 2).tolist()} m, "
              f"cao so với mặt phẳng {height[near].min():.2f}..{height[near].max():.2f} m")

        # Hàng trên: chiếu lên camera, xám = vật cản khác, đỏ = cluster gần nhất
        ax = axes[0, col]
        ax.imshow(fr["image"][..., ::-1])
        for m, color, s in [(keep & ~near, "lightgray", 0.3), (near, "red", 1.5)]:
            uv, _, _ = project_velo_to_image(down[m], fr["calib"], fr["image"].shape)
            ax.scatter(uv[:, 0], uv[:, 1], s=s, c=color, linewidths=0)
        ax.set_title(f"dist_thr = {thr} m: vật cản gần nhất = {dist:.2f} m (đỏ, {near.sum()} điểm)")
        ax.axis("off")

        # Hàng dưới: BEV vùng gần xe, tô theo độ cao so với mặt phẳng RANSAC
        ax = axes[1, col]
        roi = (down[:, 0] < 15) & (np.abs(down[:, 1]) < 10)
        ax.scatter(down[roi & ground, 0], down[roi & ground, 1], s=0.5, c="lightgray", linewidths=0,
                   label="ground (RANSAC)")
        sc = ax.scatter(down[roi & keep, 0], down[roi & keep, 1], s=1.5, c=height[roi & keep],
                        cmap="turbo", vmin=-0.2, vmax=1.0, linewidths=0)
        ax.scatter(p[:, 0], p[:, 1], s=6, facecolors="none", edgecolors="red", linewidths=0.4,
                   label=f"cluster gần nhất: cao {height[near].min():.2f}..{height[near].max():.2f} m")
        ax.plot(0, 0, "k^", ms=8)                       # vị trí LiDAR
        ax.set(xlim=(0, 15), ylim=(-10, 10), aspect="equal", xlabel="x (m)", ylabel="y (m)")
        ax.legend(loc="upper right", fontsize=8, markerscale=4)
        fig.colorbar(sc, ax=ax, label="độ cao so với mặt phẳng RANSAC (m)", shrink=0.8)
    fig.suptitle("fail_01 — frame 000004: với dist_thr = 0.1 m, mặt đường thấp hơn mặt phẳng 0.10–0.18 m "
                 "bị báo là vật cản gần nhất (4.02 m)")
    fig.tight_layout()
    fig.savefig(OUT / "fail_01_ground_false_obstacle.png", dpi=110)
    plt.close(fig)


def fail_02() -> None:
    fr = load_frame("data/kitti_mini", "000011")
    obj = fr["labels"][1]                    # Pedestrian 14.5 m, occluded = 2
    x1, y1, x2, y2 = obj.bbox
    pad = 60
    fig, axes = plt.subplots(1, 3, figsize=(15, 6))
    for col, thr in enumerate([0.1, 0.2, 0.3]):
        down, ground, height, keep, labels = pipeline(fr["points"][:, :3], VOXEL, thr, EPS)
        cl = big_clusters(keep, labels)
        inbox = points_in_box(velo_to_cam(down, fr["calib"]), obj)
        ids, cnt = np.unique(labels[np.isin(np.flatnonzero(keep), np.flatnonzero(inbox))], return_counts=True)
        sizes = {int(i): int(c) for i, c in zip(ids, cnt) if i >= 0}
        best = max(sizes.values(), default=0)
        print(f"fail_02 thr={thr}: {inbox.sum()} điểm trong box, ground ăn {(inbox & ground).sum()}, "
              f"còn {(inbox & ~ground).sum()}, cluster lớn nhất {best} điểm (cần >= {MIN_PTS})")

        ax = axes[col]
        ax.imshow(fr["image"][..., ::-1])
        for m, color, name in [(inbox & ground, "gray", "bị coi là ground"),
                               (inbox & ~ground & (cl >= 0), "lime", f"thuộc cluster >= {MIN_PTS} điểm"),
                               (inbox & ~ground & (cl < 0), "red", "không thành cluster")]:
            uv, _, _ = project_velo_to_image(down[m], fr["calib"], fr["image"].shape)
            ax.scatter(uv[:, 0], uv[:, 1], s=40, c=color, edgecolors="black", linewidths=0.5,
                       label=f"{name}: {m.sum()}")
        ax.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, ec="yellow", lw=1.5))
        ax.set(xlim=(x1 - pad, x2 + pad), ylim=(y2 + pad, y1 - pad))
        verdict = "PHÁT HIỆN" if best >= MIN_PTS else "BỎ SÓT"
        ax.set_title(f"dist_thr = {thr} m -> {verdict} (cluster lớn nhất {best} điểm)")
        ax.legend(loc="lower left", fontsize=8)
        ax.axis("off")
    fig.suptitle("fail_02 — frame 000011, người đi bộ bị che (occluded = 2) ở 14.5 m, chỉ có 24 điểm LiDAR")
    fig.tight_layout()
    fig.savefig(OUT / "fail_02_occluded_pedestrian_lost.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    fail_01()
    fail_02()
    print(f"-> {OUT}/fail_01_ground_false_obstacle.png, {OUT}/fail_02_occluded_pedestrian_lost.png")
