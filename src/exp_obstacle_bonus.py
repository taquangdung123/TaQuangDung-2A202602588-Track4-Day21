"""Topic D, bonus: so sánh cách tách mặt đất (B1), stress test suy giảm dữ liệu (B2), đo latency từng bước (B3).

Chạy từ gốc repo:
    python -m src.exp_obstacle_bonus compare      # B1: RANSAC so với cắt độ cao cố định, trên cả 20 frame KITTI
    python -m src.exp_obstacle_bonus stress       # B2: nhiễu Gauss và random dropout, mỗi loại 4 mức
    python -m src.exp_obstacle_bonus latency      # B3: latency từng bước, mỗi dòng CSV là 1 lần chạy
    python -m src.exp_obstacle_bonus <lệnh> --help
"""
from __future__ import annotations

import argparse
import platform
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from src.exp_obstacle_sweep import dbscan, height_ground, in_roi, ransac_ground, run_one, voxel_down, write_csv
from starter.datasets import load_frame
from starter.perturb import gaussian_noise, random_dropout

ALL_KITTI = ["000001", "000004", "000007", "000008", "000009", "000010", "000011", "000012", "000015", "000016",
             "000019", "000021", "000023", "000025", "000031", "000032", "000043", "000048", "000049", "000061"]
FIG = Path("results/figures")


def summarize(rows: list[dict], objs: list[dict], keys: list[str]) -> pd.DataFrame:
    """Gộp theo cấu hình: % điểm còn lại / lát thấp / tỉ lệ phát hiện theo class, và thống kê cluster."""
    o = pd.DataFrame(objs)
    r = pd.DataFrame(rows)
    per_cls = o.groupby(keys + ["class_group"]).agg(
        n=("detected", "size"), kept=("kept_ratio", "mean"), low_kept=("low_kept_ratio", "mean"),
        detected=("detected", "mean")).unstack("class_group")
    per_cls.columns = [f"{m}_{c}" for m, c in per_cls.columns]
    per_run = r.groupby(keys).agg(clusters_per_frame=("n_clusters", "mean"),
                                  flat_clusters_total=("n_flat_clusters", "sum"),
                                  nearest_m_median=("nearest_cluster_m", "median"))
    return per_cls.join(per_run).round(3).reset_index()


# ---------------------------------------------------------------- B1
def cmd_compare(a) -> None:
    methods = [("ransac", {"ground_method": "ransac", "dist_thr": a.dist_thr}),
               ("height_cut", {"ground_method": "height", "z_cut": a.z_cut})]
    rows, objs = [], []
    for frame in a.frames:
        fr = load_frame(a.data_root, frame)
        for name, kw in methods:
            s, ob = run_one(fr, a.voxel, kw.get("dist_thr", a.dist_thr), a.eps,
                            ground_method=kw["ground_method"], z_cut=kw.get("z_cut", a.z_cut))
            cfg = {"frame": frame, "method": name}
            rows.append({**cfg, **s})
            objs += [{**cfg, **o} for o in ob]
    write_csv(Path(a.out), rows)
    write_csv(Path(a.out).with_name(Path(a.out).stem + "_objects.csv"), objs)
    summ = summarize(rows, objs, ["method"])
    print(summ.T.to_string())
    r = pd.DataFrame(rows)
    piv = r.pivot(index="frame", columns="method", values=["n_flat_clusters", "nearest_cluster_m"])
    print(piv[(piv["n_flat_clusters"] > 0).any(axis=1)].to_string())


# ---------------------------------------------------------------- B2
def cmd_stress(a) -> None:
    perturbs = [("gaussian_noise", "sigma_m", a.noise_levels,
                 lambda p, v: gaussian_noise(p, sigma_xyz_m=v, seed=a.seed) if v > 0 else p),
                ("random_dropout", "keep_ratio", a.keep_levels,
                 lambda p, v: random_dropout(p, keep_ratio=v, seed=a.seed) if v < 1 else p)]
    rows, objs = [], []
    for frame in a.frames:
        fr = load_frame(a.data_root, frame)
        for kind, pname, levels, fn in perturbs:
            for v in levels:
                frp = {**fr, "points": fn(fr["points"], v)}
                s, ob = run_one(frp, a.voxel, a.dist_thr, a.eps)
                cfg = {"frame": frame, "perturb": kind, "level": v}
                rows.append({**cfg, **s})
                if v == levels[0]:                     # mức gốc: tập vật cần đánh giá
                    base = {o["object_id"]: o for o in ob}
                seen = {o["object_id"] for o in ob}
                # Vật có ở mức gốc nhưng sau suy giảm còn < MIN_PTS điểm: tính là bỏ sót, không được bỏ qua
                ob += [{**base[i], "pts_before": 0, "pts_after_ground": 0, "kept_ratio": 0.0, "low_pts_before": 0,
                        "low_kept_ratio": 0.0, "best_cluster_pts": 0, "detected": 0} for i in base if i not in seen]
                objs += [{**cfg, **o} for o in ob]
    write_csv(Path(a.out), rows)
    write_csv(Path(a.out).with_name(Path(a.out).stem + "_objects.csv"), objs)
    summ = summarize(rows, objs, ["perturb", "level"])
    print(summ.to_string())

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for ax, (kind, pname, levels, _) in zip(axes, perturbs):
        g = summ[summ["perturb"] == kind].sort_values("level")
        for col, lab, c, ls in [("detected_Pedestrian", "Người đi bộ: % thành cluster", "tab:orange", "-"),
                                ("detected_Car", "Xe: % thành cluster", "tab:blue", "-"),
                                ("low_kept_Pedestrian", "Người đi bộ: % điểm lát thấp còn lại", "tab:orange", "--")]:
            ax.plot(g["level"], 100 * g[col], marker="o", color=c, ls=ls, label=lab)
        ax2 = ax.twinx()
        ax2.plot(g["level"], g["clusters_per_frame"], marker="s", color="gray", ls=":", label="Số cluster TB/frame")
        ax2.set_ylabel("Số cluster (>= 10 điểm) TB/frame", color="gray")
        ax2.set_ylim(0, None)
        ax.set(xlabel=f"{kind}: {pname}", ylabel="%", ylim=(0, 105), xticks=levels,
               title=f"{kind} (RANSAC {a.dist_thr} m, eps {a.eps} m, seed {a.seed})")
        if kind == "random_dropout":
            ax.invert_xaxis()
        ax.grid(alpha=0.3)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="lower left")
    fig.tight_layout()
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / "bonus_b2_stress.png", dpi=150)
    print(f"-> {FIG / 'bonus_b2_stress.png'}")


# ---------------------------------------------------------------- B3
def hardware() -> dict:
    cpu, ram = platform.processor(), ""
    try:
        import subprocess
        if platform.system() == "Windows":
            q = lambda c: subprocess.run(["powershell", "-NoProfile", "-c", c], capture_output=True, text=True).stdout.strip()
            cpu = q("(Get-CimInstance Win32_Processor).Name") or cpu
            ram = q("[math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory/1GB,1)")
    except Exception:
        pass
    return {"cpu": cpu, "ram_gb": ram, "os": platform.platform(), "python": platform.python_version()}


def cmd_latency(a) -> None:
    hw = hardware()
    print(hw)
    rows = []
    for frame in a.frames:
        fr = load_frame(a.data_root, frame)
        xyz = fr["points"][:, :3]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        for method in ["ransac", "height_cut"]:
            for rep in range(a.reps + 1):              # rep 0 = warm-up, ghi lại nhưng bỏ khi tính p50/p95
                t = [time.perf_counter()]
                down = voxel_down(xyz, a.voxel); t.append(time.perf_counter())
                ground, _ = ransac_ground(down, a.dist_thr) if method == "ransac" else height_ground(down, a.z_cut)
                t.append(time.perf_counter())
                keep = ~ground & in_roi(down)
                dbscan(down[keep], a.eps); t.append(time.perf_counter())
                ms = np.diff(t) * 1000
                rows.append({"frame": frame, "method": method, "rep": rep, "warmup": int(rep == 0),
                             "n_points": len(xyz), "n_down": len(down), "n_obstacle": int(keep.sum()),
                             "voxel_ms": round(ms[0], 2), "ground_ms": round(ms[1], 2), "dbscan_ms": round(ms[2], 2),
                             "total_ms": round(ms.sum(), 2), **hw})
    write_csv(Path(a.out), rows)
    df = pd.DataFrame(rows)
    df = df[df["warmup"] == 0]
    q = lambda p: (lambda s: np.percentile(s, p))
    stats = df.groupby("method")[["voxel_ms", "ground_ms", "dbscan_ms", "total_ms"]].agg([q(50), q(95)])
    stats.columns = [f"{c}_{'p50' if i % 2 == 0 else 'p95'}" for i, (c, _) in enumerate(stats.columns)]
    print(stats.round(1).T.to_string())
    print("total_ms của lần warm-up (đã bỏ):", pd.DataFrame(rows).query("warmup == 1")["total_ms"].round(0).tolist())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p, frames, out):
        p.add_argument("--data-root", default="data/kitti_mini", help="thư mục dataset dạng KITTI")
        p.add_argument("--frames", nargs="+", default=frames, help="danh sách frame id")
        p.add_argument("--voxel", type=float, default=0.1, help="kích thước voxel downsample (m)")
        p.add_argument("--dist-thr", type=float, default=0.2, help="distance_threshold của RANSAC (m)")
        p.add_argument("--z-cut", type=float, default=-1.5,
                       help="ngưỡng cắt độ cao (velodyne z, m): z < z_cut là ground (LiDAR KITTI cao 1.73 m)")
        p.add_argument("--eps", type=float, default=0.5, help="eps của DBSCAN (m)")
        p.add_argument("--out", default=out, help="đường dẫn CSV kết quả")

    fmt = argparse.ArgumentDefaultsHelpFormatter
    p = sub.add_parser("compare", formatter_class=fmt, help="B1: RANSAC so với cắt độ cao cố định")
    common(p, ALL_KITTI, "results/bonus_b1_ground_compare.csv")
    p.set_defaults(fn=cmd_compare)

    p = sub.add_parser("stress", formatter_class=fmt, help="B2: nhiễu Gauss và random dropout")
    common(p, ["000011", "000015", "000019", "000004"], "results/bonus_b2_stress.csv")
    p.add_argument("--noise-levels", nargs="+", type=float, default=[0.0, 0.02, 0.05, 0.1],
                   help="các mức sigma nhiễu Gauss xyz (m), 0 = dữ liệu gốc")
    p.add_argument("--keep-levels", nargs="+", type=float, default=[1.0, 0.7, 0.5, 0.3],
                   help="các mức tỉ lệ giữ lại điểm, 1.0 = dữ liệu gốc")
    p.add_argument("--seed", type=int, default=0, help="seed cho phép suy giảm ngẫu nhiên")
    p.set_defaults(fn=cmd_stress)

    p = sub.add_parser("latency", formatter_class=fmt, help="B3: latency từng bước, mỗi dòng 1 lần chạy")
    common(p, ["000011", "000015", "000019", "000004"], "results/bonus_b3_latency_runs.csv")
    p.add_argument("--reps", type=int, default=20, help="số lần đo (không tính 1 lần warm-up)")
    p.set_defaults(fn=cmd_latency)

    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
