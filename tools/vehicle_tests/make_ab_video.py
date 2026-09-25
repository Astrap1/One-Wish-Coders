#!/usr/bin/env python3
"""
make_ab_video.py — turn the Gazebo camera frames of mud_ab_test into the
Checkpoint 2 clip: wide shot + close-up picture-in-picture + live telemetry
read from the AirCushion CSV logs (mode, terrain, speed, cushion gap).
"""
import argparse
import bisect
import csv
import re
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser()
ap.add_argument("--frames", required=True)      # /tmp/gz_clip
ap.add_argument("--logs", required=True)        # results/mud_ab_test
ap.add_argument("--out", required=True)
ap.add_argument("--fps", type=float, default=12)
ap.add_argument("--sim-seconds", type=float, default=22.0)
a = ap.parse_args()

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
f_title, f_big, f_txt, f_small = (ImageFont.truetype(FONT_B, 26), ImageFont.truetype(FONT_B, 20),
                                  ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 15))
C_GROUND, C_HOVER = (181, 101, 29), (31, 111, 178)


def frames(cam):
    d = Path(a.frames) / cam
    fs = list(d.glob("*.png"))
    fs.sort(key=lambda p: int(re.findall(r"_(\d+)\.png$", p.name)[0]))
    return fs


def load(p):
    rows = list(csv.DictReader(open(p)))
    t = [float(r["t"]) for r in rows]
    return t, rows


def at(log, t):
    ts, rows = log
    i = min(max(bisect.bisect_left(ts, t), 0), len(rows) - 1)
    return rows[i]


wide, close = frames("cam_wide"), frames("cam_close")
n = len(wide)
gl = load(Path(a.logs) / "hover_hc_ground.csv")
hl = load(Path(a.logs) / "hover_hc_hover.csv")
tmp = Path(a.out).parent / "_frames"
tmp.mkdir(parents=True, exist_ok=True)
for f in tmp.glob("*.png"):
    f.unlink()

MODE = {"OFF": "GROUND mode", "SPIN_UP": "switching: lift fan spin-up",
        "FAN_ON_NO_SUPPORT": "switching: folding legs", "HOVER": "HOVER mode",
        "SPIN_DOWN": "switching: spin-down"}


def panel(d, xy, color, title, lines):
    x, y = xy
    w, h = 470, 34 + 24 * len(lines)
    d.rounded_rectangle((x, y, x + w, y + h), 10, fill=(255, 255, 255, 215))
    d.rectangle((x, y + 8, x + 6, y + h - 8), fill=color)
    d.text((x + 18, y + 6), title, font=f_big, fill=color)
    for i, ln in enumerate(lines):
        d.text((x + 18, y + 34 + 24 * i), ln, font=f_txt, fill=(30, 30, 30))


for i, fw in enumerate(wide):
    t = a.sim_seconds - (n - 1 - i) / a.fps          # frames are 1/fps apart, last at the end
    im = Image.open(fw).convert("RGBA")
    W, H = im.size
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    d.rectangle((0, 0, W, 48), fill=(20, 24, 28, 225))
    d.text((16, 9), "Checkpoint 2 — mud A/B test: same vehicle, same mud, two modes",
           font=f_title, fill=(255, 255, 255))
    d.text((W - 190, 13), f"sim t = {max(t, 0):5.1f} s", font=f_big, fill=(255, 210, 90))

    g, h = at(gl, t), at(hl, t)
    panel(d, (16, 62), C_GROUND, "A (far lane) · wheels only",
          [f"{MODE.get(g['hover_state'], g['hover_state'])} · on {g['terrain']}",
           f"wheel command 1.0 m/s · actual {float(g['speed']):.2f} m/s",
           f"x = {float(g['x']):5.2f} m"])
    st = h["hover_state"]
    panel(d, (16, H - 150), C_HOVER, "B (near lane) · air cushion + fans",
          [f"{MODE.get(st, st)} · over {h['terrain']}",
           f"cushion gap {float(h['gap_mean']) * 100:4.1f} cm · speed {float(h['speed']):.2f} m/s",
           f"fan thrust {abs(float(h['thrust_left'])) if abs(float(h['thrust_left'])) < 0.05 else float(h['thrust_left']):4.1f} N × 2 · x = {float(h['x']):5.2f} m"])
    d.text((W - 470, H - 26), "Gazebo Harmonic · 1 ms physics · simplified air-cushion model",
           font=f_small, fill=(255, 255, 255))
    im = Image.alpha_composite(im, ov)
    if close:
        j = min(len(close) - 1, max(0, len(close) - (n - i)))
        c = Image.open(close[j]).convert("RGBA").resize((448, 252))
        x0, y0 = W - 448 - 18, 62
        frame = Image.new("RGBA", (452, 282), (255, 255, 255, 230))
        im.alpha_composite(frame, (x0 - 2, y0 - 2))
        im.alpha_composite(c, (x0, y0))
        ImageDraw.Draw(im).text((x0 + 8, y0 + 254), "close-up: lane A wheels spinning in the mud",
                                font=f_small, fill=(40, 40, 40))
    im.convert("RGB").save(tmp / f"{i:04d}.png")

# hold the last frame for 2 s so the outcome is readable
last = tmp / f"{n - 1:04d}.png"
for k in range(int(2 * a.fps)):
    (tmp / f"{n + k:04d}.png").write_bytes(last.read_bytes())

subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(a.fps), "-i",
                str(tmp / "%04d.png"), "-r", "24", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                "-crf", "20", a.out], check=True)
print(f"wrote {a.out} ({n} Gazebo frames + hold)")
