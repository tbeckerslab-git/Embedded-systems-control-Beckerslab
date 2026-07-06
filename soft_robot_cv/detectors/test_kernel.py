"""Test script for rod_video_to_state_kernel.py.

Runs the full pipeline on video_0.avi (cosserat_rod_data/videos/),
saves .mat / .npz datasets, and generates diagnostic figures + centreline animation.

Outputs go to:
  Processed_data/kernel_test_*
  Processed_videos/kernel_test_centreline.mp4
  Figures/kernel_test_*.png
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
VIDEO_PATH  = os.path.join(SCRIPT_DIR, "..", "..", "cosserat_rod_data", "videos", "video_0-2.avi")
OUT_DATA    = os.path.join(SCRIPT_DIR, "Processed_data")
OUT_FIGS    = os.path.join(SCRIPT_DIR, "Figures")
OUT_VIDS    = os.path.join(SCRIPT_DIR, "Processed_videos")
OUT_PREFIX  = os.path.join(OUT_DATA, "kernel_test")

os.makedirs(OUT_DATA, exist_ok=True)
os.makedirs(OUT_FIGS, exist_ok=True)
os.makedirs(OUT_VIDS, exist_ok=True)

sys.path.insert(0, SCRIPT_DIR)
from rod_video_to_state_kernel import process_rod_video, extract_centerlines, find_fixed_base

# ── rod parameters (current rod, 2026-06-22) ─────────────────────────────────
L0   = 0.5    # m
b    = 15e-3    # m  (thickness, weak-axis bending direction)
h    = 20e-3    # m  (width)
mass = 0.149    # kg
N    = 25
THR  = 135
WIN  = 15       # Savitzky-Golay window (frames)

print("=" * 60)
print("rod_video_to_state_kernel  —  pipeline test")
print(f"Video : {VIDEO_PATH}")
print(f"L0={L0} m  b={b*1e3:.0f}mm  h={h*1e3:.0f}mm  mass={mass*1e3:.0f}g  N={N}")
print("=" * 60)

# ── 1. Run the full pipeline ──────────────────────────────────────────────────
print("\n[1/5] Running process_rod_video ...")
TARGET_FPS = 10000   # Hz — matches Verlet integrator dt=1e-4 s

result = process_rod_video(
    VIDEO_PATH,
    L0=L0, b=b, h=h, mass=mass, N=N,
    thr=THR, win=WIN,
    out_prefix=OUT_PREFIX,
    write_files=True,
    target_fps=TARGET_FPS,
)

state    = result["state"]
z        = state["z"]
t        = state["t"]                    # original camera time axis (194 Hz)
t_out    = result["t_query"]             # dense output time axis (10000 Hz)
z_smooth = result["z_smooth"]           # (Nq, 154) at t_out
z_dot    = result["z_dot"]              # (Nq, 154) at t_out
Xpx      = result["Xpx"]
Ypx      = result["Ypx"]
ppm      = result["ppm"]
base     = result["base"]
xb, yb   = base["xb"], base["yb"]
Nt, D    = z.shape
fps_vid  = base["fps"]

print(f"  Frames extracted : {Nt}  (camera, {fps_vid:.0f} Hz)")
print(f"  State dim D      : {D}   (expected {4*(N+1)+2*N})")
print(f"  Duration         : {t[-1]:.3f} s")
print(f"  Output steps     : {len(t_out)}  (dt={t_out[1]-t_out[0]:.2e} s, {TARGET_FPS} Hz)")
print(f"  ppm              : {ppm:.2f} px/m")
print(f"  Recon error      : {state['recon_mm']:.3f} mm")
print(f"  GP lengthscale   : {result['gp_lengthscale']:.4f} s")
print(f"  GP noise         : {result['gp_noise']:.4f}")
print(f"  Files written    : {result['files']}")

NN = N + 1
x_m   = state["x"]
y_m   = state["y"]
theta = state["theta"]
px    = state["px"]
py    = state["py"]

# ── 2. Figure: clamp detection + first frame centreline ──────────────────────
print("\n[2/5] Generating figures ...")

import cv2
cap = cv2.VideoCapture(VIDEO_PATH)
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
_, frame0 = cap.read()
cap.release()
frame0_rgb = cv2.cvtColor(frame0, cv2.COLOR_BGR2RGB)

fig, ax = plt.subplots(1, 1, figsize=(8, 6))
ax.imshow(frame0_rgb)
ax.plot(Xpx[0], Ypx[0], "c.-", lw=1.5, ms=6, label="nodes frame 0")
ax.plot(xb, yb, "r*", ms=14, label=f"clamp ({xb:.1f},{yb:.1f})")
ax.set_title("Frame 0: detected centreline & clamp")
ax.legend(fontsize=9)
ax.axis("off")
fig.tight_layout()
p = os.path.join(OUT_FIGS, "kernel_test_frame0_overlay.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# ── 3. Figure: tip trajectory ─────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
axes[0].plot(t, x_m[:, -1], "b", lw=1, label=f"SavGol ({fps_vid:.0f} Hz)")
axes[0].plot(t_out, z_smooth[:, N], "r--", lw=1.5, label=f"GP smooth ({TARGET_FPS} Hz)")
axes[0].set_ylabel("x tip [m]"); axes[0].legend(fontsize=8); axes[0].grid(True)
axes[1].plot(t, y_m[:, -1], "b", lw=1, label=f"SavGol ({fps_vid:.0f} Hz)")
axes[1].plot(t_out, z_smooth[:, NN + N], "r--", lw=1.5, label=f"GP smooth ({TARGET_FPS} Hz)")
axes[1].set_ylabel("y tip [m]"); axes[1].set_xlabel("time [s]")
axes[1].legend(fontsize=8); axes[1].grid(True)
fig.suptitle("Tip trajectory (raw vs GP-smooth)", fontsize=11)
fig.tight_layout()
p = os.path.join(OUT_FIGS, "kernel_test_tip_trajectory.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# ── 4. Figure: state z and zdot overview ─────────────────────────────────────
# Layout: 6 rows (one per state group) × 2 cols (left=z, right=zdot)
# State layout: x(N+1) | y(N+1) | θ(N) | px(N+1) | py(N+1) | pθ(N)
labels_z  = ["x tip [m]", "y tip [m]", "θ mid [rad]",
             "px mid [kg/s]", "py mid [kg/s]", "pθ mid [kg·m/s]"]
labels_zd = ["ẋ tip [m/s]", "ẏ tip [m/s]", "θ̇ mid [rad/s]",
             "p̊x mid", "p̊y mid", "p̊θ mid"]
col_picks = [N,               # x tip        col 25
             NN + N,          # y tip        col 51
             2*NN + N//2,     # θ mid seg    col 64
             2*NN + N + N//2, # px mid node  col 89
             3*NN + N + N//2, # py mid node  col 115
             4*NN + N + N//2] # pθ mid seg   col 141

fig, axes = plt.subplots(6, 2, figsize=(14, 18), sharex=True)
fig.suptitle("State z (left) and ż (right) — SavGol vs GP-smooth", fontsize=12)

for row, (lbl, lbd, ci) in enumerate(zip(labels_z, labels_zd, col_picks)):
    ax_z  = axes[row, 0]
    ax_zd = axes[row, 1]

    # left: SavGol (raw z) vs GP-smooth
    ax_z.plot(t,     z[:, ci],        "b",  lw=1.0, alpha=0.7, label="SavGol")
    ax_z.plot(t_out, z_smooth[:, ci], "r",  lw=1.0, label="GP smooth")
    ax_z.set_ylabel(lbl, fontsize=8)
    ax_z.legend(fontsize=7, loc="upper right")
    ax_z.grid(True)

    # right: zdot only (at dense t_out)
    ax_zd.plot(t_out, z_dot[:, ci], "g", lw=0.8)
    ax_zd.set_ylabel(lbd, fontsize=8)
    ax_zd.grid(True)
    ax_zd.axhline(0, color="k", lw=0.5, ls="--")

axes[-1, 0].set_xlabel("time [s]", fontsize=9)
axes[-1, 1].set_xlabel("time [s]", fontsize=9)
fig.tight_layout()
p = os.path.join(OUT_FIGS, "kernel_test_state_overview.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# ── 5. Figure: arc-length per frame ──────────────────────────────────────────
arc_lens = np.array([
    np.hypot(*np.diff(
        np.column_stack([Xpx[i], Ypx[i]]), axis=0
    ).T).sum() / ppm
    for i in range(Nt)
])
fig, ax = plt.subplots(figsize=(10, 3))
ax.plot(t, arc_lens, lw=1)
ax.axhline(L0, color="r", ls="--", label=f"L0 = {L0} m")
ax.axhline(0.98 * L0, color="orange", ls=":", label="0.98·L0")
ax.set_xlabel("time [s]"); ax.set_ylabel("measured arc [m]")
ax.set_title("Arc-length per frame"); ax.legend(); ax.grid(True)
fig.tight_layout()
p = os.path.join(OUT_FIGS, "kernel_test_arc_length.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# ── 6. Animation: centreline overlaid on video ────────────────────────────────
print("\n[3/5] Building centreline animation ...")
ANIM_FPS  = min(30, fps_vid)          # animation playback fps
STEP      = max(1, int(fps_vid / ANIM_FPS))   # stride to downsample frames
OUT_ANIM  = os.path.join(OUT_VIDS, "kernel_test_centreline.mp4")

cap = cv2.VideoCapture(VIDEO_PATH)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
frame_idxs   = list(range(0, min(Nt, total_frames), STEP))

fig_a, ax_a = plt.subplots(figsize=(7, 5.25))
ax_a.axis("off")
fig_a.subplots_adjust(left=0, right=1, top=1, bottom=0)

cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
_, fr0 = cap.read()
im_h  = ax_a.imshow(cv2.cvtColor(fr0, cv2.COLOR_BGR2RGB), animated=True)
line, = ax_a.plot([], [], "c.-", lw=1.5, ms=5)
dot,  = ax_a.plot([], [], "r*", ms=12)
ttl   = ax_a.set_title("", fontsize=9)

def _init():
    line.set_data([], [])
    dot.set_data([], [])
    return im_h, line, dot, ttl

def _update(idx):
    fi = frame_idxs[idx]
    cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
    ok, fr = cap.read()
    if ok:
        im_h.set_data(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB))
    line.set_data(Xpx[fi], Ypx[fi])
    dot.set_data([xb], [yb])
    ttl.set_text(f"frame {fi}  t={t[fi]:.3f}s")
    return im_h, line, dot, ttl

ani = animation.FuncAnimation(
    fig_a, _update, frames=len(frame_idxs),
    init_func=_init, blit=True, interval=1000/ANIM_FPS,
)
writer = animation.FFMpegWriter(fps=ANIM_FPS, bitrate=2000)
ani.save(OUT_ANIM, writer=writer)
cap.release()
plt.close(fig_a)
print(f"  Saved: {OUT_ANIM}")

# ── 7. Summary ────────────────────────────────────────────────────────────────
print("\n[4/5] Summary")
print(f"  z shape       : {z.shape}")
print(f"  z_smooth shape: {z_smooth.shape}")
print(f"  z_dot shape   : {z_dot.shape}")
print(f"  x tip range   : [{x_m[:,-1].min():.4f}, {x_m[:,-1].max():.4f}] m")
print(f"  y tip range   : [{y_m[:,-1].min():.4f}, {y_m[:,-1].max():.4f}] m")
print(f"  arc mean/std  : {arc_lens.mean():.4f} ± {arc_lens.std():.4f} m  (L0={L0})")
rho = state["rho"]; A = state["A"]
print(f"  rho={rho:.2f} kg/m³  A={A*1e6:.2f} mm²")

print("\n[5/5] Done.")
print(f"  Data  -> {OUT_DATA}/kernel_test_*")
print(f"  Figs  -> {OUT_FIGS}/kernel_test_*.png")
print(f"  Anim  -> {OUT_ANIM}")
