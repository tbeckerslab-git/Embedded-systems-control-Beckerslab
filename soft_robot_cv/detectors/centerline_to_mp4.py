#!/usr/bin/env python3
"""
Convert centerline_data_*.txt files to MP4 animations.

File format per row:
  frame_id  x_1 x_2 ... x_N  y_1 y_2 ... y_N  [frame_counter | timestamp]

The last column is always a trailing scalar (timestamp or counter), ignored for
rendering. The number of x/y points is inferred from (num_cols - 2) / 2.
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def load_centerline(path):
    data = np.loadtxt(path)
    n_cols = data.shape[1]

    # Format: [x_1 .. x_N, y_1 .. y_N, frame_counter, (unix_timestamp)?]
    # The last column is a unix timestamp if > 1e9, otherwise it is the frame counter.
    # The second-to-last column is the frame counter when a timestamp is present.
    last_col = data[:, -1]
    if last_col[-1] > 1e9:
        # last col = unix timestamp, second-to-last = frame counter
        t = last_col - last_col[0]
        fps = 1.0 / np.median(np.diff(t))
        N = (n_cols - 2) // 2
    else:
        fps = 30.0
        N = (n_cols - 1) // 2

    x = data[:, 0:N]
    y = data[:, N:2*N]
    return x, y, fps, N


def make_video(txt_path, out_path, title=""):
    x, y, fps, N = load_centerline(txt_path)
    n_frames = x.shape[0]

    # canvas bounds with a small margin
    pad = 20
    x_min, x_max = x.min() - pad, x.max() + pad
    y_min, y_max = y.min() - pad, y.max() + pad

    fig, ax = plt.subplots(figsize=(8, 6), facecolor="black")
    ax.set_facecolor("black")
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_max, y_min)   # flip so pixel-origin is top-left
    ax.axis("off")

    line, = ax.plot([], [], "o-", color="cyan", lw=1.5, ms=3, markerfacecolor="white")
    frame_text = ax.text(
        0.02, 0.97, "", transform=ax.transAxes,
        color="white", fontsize=9, va="top", family="monospace"
    )
    if title:
        ax.set_title(title, color="white", fontsize=10, pad=4)

    def init():
        line.set_data([], [])
        frame_text.set_text("")
        return line, frame_text

    def update(i):
        line.set_data(x[i], y[i])
        frame_text.set_text(f"frame {i+1}/{n_frames}")
        return line, frame_text

    interval_ms = 1000.0 / fps
    ani = animation.FuncAnimation(
        fig, update, frames=n_frames, init_func=init,
        interval=interval_ms, blit=True
    )

    writer = animation.FFMpegWriter(fps=fps, codec="libx264",
                                    extra_args=["-pix_fmt", "yuv420p"])
    ani.save(out_path, writer=writer, dpi=120)
    plt.close(fig)
    print(f"Saved: {out_path}  ({n_frames} frames @ {fps:.1f} fps, {N} pts)")


if __name__ == "__main__":
    base = os.path.dirname(os.path.abspath(__file__))

    files = [
        ("centerline_data_25pts.txt",  "centerline_25pts.mp4",  "Centerline – 25 pts"),
        ("centerline_data_new.txt",    "centerline_new.mp4",    "Centerline – 100 pts"),
    ]

    targets = sys.argv[1:] if len(sys.argv) > 1 else [f[0] for f in files]

    for fname, outname, title in files:
        if fname not in targets and os.path.splitext(fname)[0] not in targets:
            continue
        src = os.path.join(base, fname)
        dst = os.path.join(base, outname)
        if not os.path.exists(src):
            print(f"Not found, skipping: {src}")
            continue
        print(f"Processing {fname} …")
        make_video(src, dst, title)
