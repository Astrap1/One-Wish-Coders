#!/usr/bin/env python3
"""plot_sensors.py — turn the sensor_test snapshots into images:
results/plots/phase2_lidar_topdown.png and phase2_front_camera.png."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
R = ROOT / "results" / "sensor_test"
P = ROOT / "results" / "plots"
P.mkdir(parents=True, exist_ok=True)
import json
_LF = json.loads((ROOT / "autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/models/"
                  "hovercraft/link_frames.json").read_text())
LIDAR_X, _, LIDAR_Z = _LF["sensors"]["lidar_3d"]["xyz"]      # ground-mode position

rows = list(csv.DictReader(open(R / "lidar_scan.csv")))
az = np.array([float(r["azimuth"]) for r in rows])
el = np.array([float(r["elevation"]) for r in rows])
rg = np.array([float(r["range"]) for r in rows])
x = rg * np.cos(el) * np.cos(az) + LIDAR_X    # LiDAR sits ahead of the hull centre
y = rg * np.cos(el) * np.sin(az)
z = rg * np.sin(el) + LIDAR_Z
self_hit = (np.abs(x) < 0.75) & (np.abs(y) < 0.55)   # returns from the vehicle's own body
ok = np.isfinite(rg) & ~self_hit
# Self-occlusion report: returns that hit the vehicle itself, per 30 deg sector
fin = np.isfinite(rg)
deg = np.degrees(az)
rear = (np.abs(deg) > 150)
print(f"self-hits: {100 * (self_hit & fin).mean():.1f}% of all beams; "
      f"rear +/-30 deg sector: {100 * (self_hit & fin)[rear].mean():.1f}%")
(R / "occlusion.txt").write_text(
    f"self_hit_all_pct {100 * (self_hit & fin).mean():.2f}\n"
    f"self_hit_rear60_pct {100 * (self_hit & fin)[rear].mean():.2f}\n")

fig, ax = plt.subplots(figsize=(7, 6))
sc = ax.scatter(x[ok], y[ok], c=z[ok], s=3, cmap="viridis", vmin=0, vmax=2)
fig.colorbar(sc, ax=ax, label="point height (m)", shrink=0.8)
ax.add_patch(plt.Rectangle((-0.6, -0.35), 1.2, 0.7, fill=False, lw=1.5, ec="#e07b00"))
ax.annotate("hovercraft", (0, 0.45), ha="center", fontsize=9, color="#e07b00")
for name, (px, py) in {"trunk": (3.0, 1.2), "trunk ": (4.5, -2.0), "trunk  ": (-2.5, 2.8),
                       "fallen log": (2.2, -1.0), "rock": (-2.8, -2.2), "bank": (8.5, 4.5)}.items():
    ax.annotate(name.strip(), (px, py), xytext=(px + 0.3, py + 0.4), fontsize=8, color="#444",
                arrowprops=dict(arrowstyle="-", color="#999", lw=0.6))
ax.set_aspect("equal"); ax.set_xlim(-6, 10); ax.set_ylim(-6, 6)
ax.set_title("One 16-channel LiDAR scan, top-down",
             loc="left", fontsize=11, fontweight="bold")
ax.set_xlabel("x forward (m)"); ax.set_ylabel("y left (m)"); ax.grid(alpha=0.2)
fig.tight_layout(); fig.savefig(P / "phase2_lidar_topdown.png", dpi=150)
Image.open(R / "front_camera.ppm").save(P / "phase2_front_camera.png")
print("points kept", ok.sum(), "of", len(rg))
