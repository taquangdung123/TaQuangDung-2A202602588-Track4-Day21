"""Vẽ kết quả quét topic D. Chạy từ gốc repo (sau src.exp_obstacle_sweep): python -m src.plot_obstacle_sweep"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

obj = pd.read_csv("results/obstacle_sweep_objects.csv", dtype={"frame": str})
run = pd.read_csv("results/obstacle_sweep.csv", dtype={"frame": str})

fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

# (a) dist_thr: % điểm của vật còn lại sau tách mặt đất, toàn vật và lát thấp 0–0.5 m
ax = axes[0]
d = obj[obj["sweep"] == "dist_thr"]
for (grp, color) in [("Pedestrian", "tab:orange"), ("Car", "tab:blue")]:
    g = d[d["class_group"] == grp].groupby("dist_thr")
    n = d[d["class_group"] == grp]["object_id"].groupby(d["frame"]).nunique().sum()
    ax.plot(g["kept_ratio"].mean().index, 100 * g["kept_ratio"].mean(), marker="o", color=color,
            label=f"{grp}: toàn vật (n={n})")
    ax.plot(g["low_kept_ratio"].mean().index, 100 * g["low_kept_ratio"].mean(), marker="s", ls="--",
            color=color, label=f"{grp}: lát thấp 0–0.5 m")
ax.set_xlabel("distance_threshold của RANSAC (m)")
ax.set_ylabel("% điểm của vật còn lại sau tách mặt đất")
ax.set_ylim(0, 105)
ax.set_xticks(sorted(d["dist_thr"].unique()))
ax.set_title("(a) eps = 0.5 m, voxel = 0.1 m, 4 frame KITTI")
ax.grid(alpha=0.3)
ax.legend(fontsize=8)

# (b) eps: số cluster (>= 10 điểm) theo từng frame
ax = axes[1]
e = run[run["dist_thr"] == 0.1]
for frame, g in e.groupby("frame"):
    g = g.sort_values("eps")
    ax.plot(g["eps"], g["n_clusters"], marker="o", label=f"frame {frame}")
ax.set_xlabel("eps của DBSCAN (m)")
ax.set_ylabel("Số cluster (>= 10 điểm) trong ROI 40 m × 40 m")
ax.set_ylim(0, None)
ax.set_xticks(sorted(e["eps"].unique()))
ax.set_title("(b) distance_threshold = 0.1 m, voxel = 0.1 m")
ax.grid(alpha=0.3)
ax.legend(fontsize=8)

fig.tight_layout()
out = Path("results/figures/obstacle_sweep.png")
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, dpi=150)
print(f"-> {out}")
