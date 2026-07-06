"""Test script for extract_rod_joint2d.py.

Runs the full joint-2D spline pipeline on video_0.avi and generates the same
diagnostic figures and output files as test_kernel.py, with prefix joint2d_test_*.
Results can be compared directly against kernel_test_* outputs.

Outputs:
  Processed_data/joint2d_test_state_dataset.{mat,npz}
  Processed_data/joint2d_test_zdot_dataset.{mat,npz}
  Figures/joint2d_test_frame0_overlay.png
  Figures/joint2d_test_tip_trajectory.png
  Figures/joint2d_test_state_overview.png
  Figures/joint2d_test_arc_length.png
  Processed_videos/joint2d_test_centreline.mp4
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.io import savemat

# ── paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VIDEO_PATH = os.path.join(SCRIPT_DIR, "..", "..", "cosserat_rod_data", "videos", "video_0-2.avi")
OUT_DATA   = os.path.join(SCRIPT_DIR, "Processed_data")
OUT_FIGS   = os.path.join(SCRIPT_DIR, "Figures")
OUT_VIDS   = os.path.join(SCRIPT_DIR, "Processed_videos")
OUT_PREFIX = os.path.join(OUT_DATA, "joint2d_test")

os.makedirs(OUT_DATA, exist_ok=True)
os.makedirs(OUT_FIGS, exist_ok=True)
os.makedirs(OUT_VIDS, exist_ok=True)

sys.path.insert(0, SCRIPT_DIR)
from extract_rod_joint2d import process_rod_video_joint2d
from rod_video_to_state_kernel import find_fixed_base, extract_centerlines

# ── rod parameters (current rod, 2026-06-22) ──────────────────────────────────
L0   = 0.5      # m
b    = 15e-3    # m  (thickness, weak-axis bending)
h    = 20e-3    # m  (width)
mass = 0.149    # kg
N    = 25
THR  = 135
UPSAMPLE_DT = 1e-4   # s — matches Verlet integrator

print("=" * 60)
print("extract_rod_joint2d  —  pipeline test")
print(f"Video : {VIDEO_PATH}")
print(f"L0={L0} m  b={b*1e3:.0f}mm  h={h*1e3:.0f}mm  mass={mass*1e3:.0f}g  N={N}")
print("=" * 60)

# ── 1. Run the full pipeline ──────────────────────────────────────────────────
print("\n[1/5] Running process_rod_video_joint2d ...")

import cv2
cap_tmp = cv2.VideoCapture(VIDEO_PATH)
fps_vid  = float(cap_tmp.get(cv2.CAP_PROP_FPS))
duration = int(cap_tmp.get(cv2.CAP_PROP_FRAME_COUNT)) / fps_vid
cap_tmp.release()

r = process_rod_video_joint2d(
    VIDEO_PATH,
    L0=L0, b=b, h=h, mass=mass, N=N,
    thr=THR,
    upsample_dt=UPSAMPLE_DT,
    upsample_window=(0.0, duration),   # upsample full recording
)

z_nat   = r["z"]          # (Nt, D) native camera fps
zdot_nat= r["zdot"]       # (Nt, D) native camera fps
t_nat   = r["t"]          # original camera timestamps
z_up    = r["z_up"]       # (Nq, D) at UPSAMPLE_DT
zdot_up = r["zdot_up"]    # (Nq, D) at UPSAMPLE_DT
t_up    = r["t_up"]       # dense time axis
Nt, D   = z_nat.shape
Nq      = len(t_up)
NN      = N + 1
xb, yb  = r["xb"], r["yb"]
ppm     = r["ppm"]

print(f"  Frames extracted : {Nt}  (camera, {fps_vid:.0f} Hz)")
print(f"  State dim D      : {D}   (expected {4*(N+1)+2*N})")
print(f"  Duration         : {t_nat[-1]:.3f} s")
print(f"  Upsampled steps  : {Nq}  (dt={UPSAMPLE_DT:.2e} s)")
print(f"  ppm              : {ppm:.2f} px/m")
print(f"  Recon error      : {r['recon_mm']:.3f} mm")
print(f"  Surf residual    : {r['surf_info']['resid_m']*1e3:.3f} mm")
print(f"  sigX             : {r['surf_info']['sigX']*1e3:.3f} mm")
print(f"  sigY             : {r['surf_info']['sigY']*1e3:.3f} mm")

# ── 2. Save datasets (.mat + .npz) ────────────────────────────────────────────
print("\n[2/5] Saving datasets ...")
A     = b * h
rho   = mass / (A * L0)
Iarea = h * b**3 / 12.0
ds    = L0 / N
s_node = np.linspace(0, L0, NN).reshape(-1, 1)
s_seg  = ((s_node[:-1] + s_node[1:]) / 2)

common = dict(N=float(N), L0=L0, A=A, I=Iarea, rho=rho, mass=mass,
              fps=fps_vid, dt=1.0/fps_vid, dt_out=UPSAMPLE_DT,
              s_node=s_node, s_seg=s_seg, ppm=ppm, xb=xb, yb=yb,
              thb=float(r["thb"]))

sf = f"{OUT_PREFIX}_state_dataset.mat"
zf = f"{OUT_PREFIX}_zdot_dataset.mat"

savemat(sf, dict(common,
                 z=z_nat, t=t_nat.reshape(-1, 1), D=float(D),
                 state_layout="z=[x(N+1),y(N+1),theta(N),px(N+1),py(N+1),"
                              "ptheta(N)], y-up, rows=time cols=state"))
savemat(zf, dict(common,
                 z_dot=zdot_up, z_smooth=z_up, t=t_up.reshape(-1, 1), D=float(D),
                 state_layout="z_dot rows=time cols=state, y-up, dt=1e-4"))
np.savez(f"{OUT_PREFIX}_state_dataset.npz",
         z=z_nat, t=t_nat, s_node=s_node.ravel(), s_seg=s_seg.ravel())
np.savez(f"{OUT_PREFIX}_zdot_dataset.npz",
         z_dot=zdot_up, z_smooth=z_up, t=t_up, dt=UPSAMPLE_DT)
print(f"  Saved: {sf}")
print(f"  Saved: {zf}")

# ── 3. Figures ────────────────────────────────────────────────────────────────
print("\n[3/5] Generating figures ...")

# re-extract pixel centrelines for overlay and arc-length plots
info = find_fixed_base(VIDEO_PATH, thr=THR)
Xpx, Ypx, _ = extract_centerlines(VIDEO_PATH, xb, yb, info["W"],
                                   nframes=info["nframes"],
                                   thr=THR, npts=N+1, L0=L0)

# Figure 1: frame 0 overlay
cap = cv2.VideoCapture(VIDEO_PATH)
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
_, frame0 = cap.read()
cap.release()
frame0_rgb = cv2.cvtColor(frame0, cv2.COLOR_BGR2RGB)

fig, ax = plt.subplots(figsize=(8, 6))
ax.imshow(frame0_rgb)
ax.plot(Xpx[0], Ypx[0], "c.-", lw=1.5, ms=6, label="nodes frame 0")
ax.plot(xb, yb, "r*", ms=14, label=f"clamp ({xb:.1f},{yb:.1f})")
ax.set_title("Frame 0: detected centreline & clamp (joint2d)")
ax.legend(fontsize=9)
ax.axis("off")
fig.tight_layout()
p = os.path.join(OUT_FIGS, "joint2d_test_frame0_overlay.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# Figure 2: tip trajectory — native vs upsampled
# State layout: x(N+1) | y(N+1) | theta(N) | px(N+1) | py(N+1) | ptheta(N)
# x tip  = col N (0-based)
# y tip  = col NN+N (0-based)
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
axes[0].plot(t_nat, z_nat[:, N],         "b",  lw=1,   label=f"native ({fps_vid:.0f} Hz)")
axes[0].plot(t_up,  z_up[:, N],          "r--", lw=1.5, label=f"upsampled ({int(1/UPSAMPLE_DT)} Hz)")
axes[0].set_ylabel("x tip [m]"); axes[0].legend(fontsize=8); axes[0].grid(True)

axes[1].plot(t_nat, z_nat[:, NN + N],    "b",  lw=1,   label=f"native ({fps_vid:.0f} Hz)")
axes[1].plot(t_up,  z_up[:, NN + N],    "r--", lw=1.5, label=f"upsampled ({int(1/UPSAMPLE_DT)} Hz)")
axes[1].set_ylabel("y tip [m]"); axes[1].set_xlabel("time [s]")
axes[1].legend(fontsize=8); axes[1].grid(True)
fig.suptitle("Tip trajectory — joint-2D spline (native vs upsampled)", fontsize=11)
fig.tight_layout()
p = os.path.join(OUT_FIGS, "joint2d_test_tip_trajectory.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# Figure 3: state z and zdot overview (6x2 grid)
# col_picks mirrors test_kernel.py exactly
col_picks = [N,               # x tip        col 25
             NN + N,          # y tip        col 51
             2*NN + N//2,     # θ mid seg    col 64
             2*NN + N + N//2, # px mid node  col 89
             3*NN + N + N//2, # py mid node  col 115
             4*NN + N + N//2] # pθ mid seg   col 141
labels_z  = ["x tip [m]", "y tip [m]", "θ mid [rad]",
              "px mid [kg/s]", "py mid [kg/s]", "pθ mid [kg·m/s]"]
labels_zd = ["ẋ tip [m/s]", "ẏ tip [m/s]", "θ̇ mid [rad/s]",
              "ṗx mid", "ṗy mid", "ṗθ mid"]

fig, axes = plt.subplots(6, 2, figsize=(14, 18), sharex=True)
fig.suptitle("State z (left) and ż (right) — joint-2D spline\n"
             "blue=native camera fps  red=upsampled 10 kHz", fontsize=12)

for row, (lbl, lbd, ci) in enumerate(zip(labels_z, labels_zd, col_picks)):
    ax_z  = axes[row, 0]
    ax_zd = axes[row, 1]

    ax_z.plot(t_nat, z_nat[:, ci], "b",  lw=1.0, alpha=0.7, label="native")
    ax_z.plot(t_up,  z_up[:, ci],  "r",  lw=0.8, label="upsampled")
    ax_z.set_ylabel(lbl, fontsize=8)
    ax_z.legend(fontsize=7, loc="upper right")
    ax_z.grid(True)

    ax_zd.plot(t_up, zdot_up[:, ci], "g", lw=0.8)
    ax_zd.set_ylabel(lbd, fontsize=8)
    ax_zd.grid(True)
    ax_zd.axhline(0, color="k", lw=0.5, ls="--")

axes[-1, 0].set_xlabel("time [s]", fontsize=9)
axes[-1, 1].set_xlabel("time [s]", fontsize=9)
fig.tight_layout()
p = os.path.join(OUT_FIGS, "joint2d_test_state_overview.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# Figure 4: arc-length per frame
arc_lens = np.array([
    np.hypot(*np.diff(np.column_stack([Xpx[i], Ypx[i]]), axis=0).T).sum() / ppm
    for i in range(Nt)
])
fig, ax = plt.subplots(figsize=(10, 3))
ax.plot(t_nat, arc_lens, lw=1)
ax.axhline(L0,        color="r",      ls="--", label=f"L0 = {L0} m")
ax.axhline(0.98 * L0, color="orange", ls=":",  label="0.98·L0")
ax.set_xlabel("time [s]"); ax.set_ylabel("measured arc [m]")
ax.set_title("Arc-length per frame (joint2d)"); ax.legend(); ax.grid(True)
fig.tight_layout()
p = os.path.join(OUT_FIGS, "joint2d_test_arc_length.png")
fig.savefig(p, dpi=120); plt.close(fig)
print(f"  Saved: {p}")

# ── 4. Animation ──────────────────────────────────────────────────────────────
print("\n[4/5] Building centreline animation ...")
ANIM_FPS = min(30, fps_vid)
STEP     = max(1, int(fps_vid / ANIM_FPS))
OUT_ANIM = os.path.join(OUT_VIDS, "joint2d_test_centreline.mp4")

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
dot,  = ax_a.plot([], [], "r*",  ms=12)
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
    ttl.set_text(f"frame {fi}  t={t_nat[fi]:.3f}s")
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

# ── 5. Summary ────────────────────────────────────────────────────────────────
print("\n[5/5] Summary")
print(f"  z_nat shape    : {z_nat.shape}   (native {fps_vid:.0f} Hz)")
print(f"  z_up shape     : {z_up.shape}   (upsampled {int(1/UPSAMPLE_DT)} Hz)")
print(f"  zdot_up shape  : {zdot_up.shape}")
print(f"  x tip range    : [{z_nat[:, N].min():.4f}, {z_nat[:, N].max():.4f}] m")
print(f"  y tip range    : [{z_nat[:, NN+N].min():.4f}, {z_nat[:, NN+N].max():.4f}] m")
print(f"  arc mean/std   : {arc_lens.mean():.4f} ± {arc_lens.std():.4f} m  (L0={L0})")
print(f"  recon error    : {r['recon_mm']:.3f} mm")
print(f"  surf residual  : {r['surf_info']['resid_m']*1e3:.3f} mm")
print(f"  rho={rho:.2f} kg/m³  A={A*1e6:.2f} mm²")
print(f"\nDone.")
print(f"  Data  -> {OUT_DATA}/joint2d_test_*")
print(f"  Figs  -> {OUT_FIGS}/joint2d_test_*.png")
print(f"  Anim  -> {OUT_ANIM}")
