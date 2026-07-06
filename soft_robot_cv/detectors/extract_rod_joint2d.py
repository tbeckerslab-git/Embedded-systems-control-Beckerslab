"""
extract_rod_joint2d.py
======================

Video (.avi) of a clamped soft rod swinging under gravity  ->  Cosserat-rod
port-Hamiltonian state z and its time-derivative zdot, in the CR-GPR paper
format, using a SINGLE joint 2-D (space+time) tensor-product spline surface
for smoothing.

Why joint 2-D smoothing
-----------------------
The state layout mixes fields that need SPATIAL smoothness (theta, curvature,
computed from d/ds of the centerline) with fields that need TEMPORAL smoothness
(velocities, momenta, computed from d/dt). Smoothing the two axes sequentially
(spatial spline per frame, then a per-node temporal filter) leaves an ordering
ambiguity: a per-node temporal filter ignores spatial coherence and can inject
small across-segment kinks, and theta computed from time-smoothed positions is
not guaranteed consistent with theta-dot computed separately.

The fix here: fit ONE cubic x cubic tensor-product spline surface
    X(t, s),  Y(t, s)
over the whole (time x normalized-arclength) grid, then take EVERY quantity as
an analytic derivative of that one surface:

    theta      = atan2( dY/ds ,  dX/ds )
    theta_dot  = ( Xs*Yst - Ys*Xst ) / (Xs^2 + Ys^2)     [ exact d/dt of atan2 ]
    vx, vy     = dX/dt , dY/dt
    ax, ay     = d2X/dt2 , d2Y/dt2
    px, py     = rho*A * (vx, vy)
    ptheta     = rho*I * theta_dot

Because theta and theta_dot come from the same surface, they are consistent
across BOTH axes by construction -- the kink risk is removed structurally
rather than filtered away, and the surface upsamples to any dt for free.

Requires: numpy, scipy, opencv-python, scikit-image  (the `rodvision` env).
Depends on the extraction primitives from the `rod-video-to-state` skill
kernel (find_fixed_base, extract_centerlines). Those are imported below; if you
run this outside the skill environment, copy that kernel.py alongside this file.
"""
import numpy as np
from scipy.interpolate import RectBivariateSpline

# --- extraction primitives (from the rod-video-to-state skill kernel) --------
# find_fixed_base(video_path, thr=135, ...) -> dict(xb, yb, clamp_edge, W, H, fps, nframes)
# extract_centerlines(video_path, xb, yb, W, nframes, thr, npts, L0) -> (Xpx, Ypx, ppm)
from rod_video_to_state_kernel import find_fixed_base, extract_centerlines  # noqa: E402


# =============================================================================
# 1.  Joint 2-D surface fit
# =============================================================================
def hf_noise(Z):
    """High-frequency (temporal) noise std of a (Nt, Nnode) field, estimated
    from the second temporal difference:  var(d2) = 6*sigma^2 for white noise."""
    dd = Z[2:] - 2 * Z[1:-1] + Z[:-2]
    return float(np.median(np.std(dd, axis=0)) / np.sqrt(6.0))


def fit_surfaces(Xn, Yn, t, snode, mult=1.0):
    """Fit cubic x cubic tensor-product spline surfaces X(t,s), Y(t,s).

    The scipy smoothing factor s is set to  Ntot * sigma^2 * mult, i.e. smooth
    down to the estimated pixel-noise floor (mult=1.0). Increase mult for more
    smoothing, decrease to track the data more closely.
    Returns (spX, spY, info_dict).
    """
    Nt, NN = Xn.shape
    Ntot = Nt * NN
    sigX, sigY = hf_noise(Xn), hf_noise(Yn)
    sX, sY = Ntot * sigX ** 2 * mult, Ntot * sigY ** 2 * mult
    spX = RectBivariateSpline(t, snode, Xn, kx=3, ky=3, s=sX)
    spY = RectBivariateSpline(t, snode, Yn, kx=3, ky=3, s=sY)
    resid = float(np.sqrt(np.mean((spX(t, snode) - Xn) ** 2 +
                                  (spY(t, snode) - Yn) ** 2)))
    return spX, spY, dict(sigX=sigX, sigY=sigY, sX=sX, sY=sY,
                          resid_m=resid, mult=mult)


# =============================================================================
# 2.  State and zdot as analytic derivatives of the surface
# =============================================================================
def state_from_surface(spX, spY, tq, snode, sseg, rho, A, Iarea,
                        thb=None):
    """Assemble z = [x, y, theta, px, py, ptheta] at query times tq.

    Node fields (x, y, px, py) on NN=len(snode) nodes; segment fields
    (theta, ptheta) on len(sseg) segments. First segment is pinned to its
    steady median angle (rigid clamp); node 0 is the clamp. Pass thb to reuse a
    fixed clamp angle (e.g. from the native fit) when upsampling.
    """
    X = spX(tq, snode);          Y = spY(tq, snode)
    Xt = spX(tq, snode, dx=1);   Yt = spY(tq, snode, dx=1)
    Xs = spX(tq, sseg, dy=1);    Ys = spY(tq, sseg, dy=1)
    Xst = spX(tq, sseg, dx=1, dy=1); Yst = spY(tq, sseg, dx=1, dy=1)

    TH = np.unwrap(np.arctan2(Ys, Xs), axis=1)
    THd = (Xs * Yst - Ys * Xst) / (Xs ** 2 + Ys ** 2)
    if thb is None:
        thb = float(np.median(TH[:, 0]))
    TH[:, 0] = thb            # rigid clamp: first segment angle fixed
    THd[:, 0] = 0.0

    PX = rho * A * Xt
    PY = rho * A * Yt
    PTH = rho * Iarea * THd
    z = np.concatenate([X, Y, TH, PX, PY, PTH], axis=1)
    return dict(z=z, X=X, Y=Y, TH=TH, Xt=Xt, Yt=Yt, THd=THd,
                PX=PX, PY=PY, PTH=PTH, thb=thb)


def zdot_from_surface(spX, spY, tq, snode, sseg, rho, A, Iarea):
    """Analytic time-derivative zdot = d/dt [x, y, theta, px, py, ptheta].

    q-dot channels are the node/segment velocities; p-dot channels are the
    accelerations times mass. theta-double-dot is the exact d/dt of theta-dot.
    """
    Xt = spX(tq, snode, dx=1);   Yt = spY(tq, snode, dx=1)
    Xtt = spX(tq, snode, dx=2);  Ytt = spY(tq, snode, dx=2)
    Xs = spX(tq, sseg, dy=1);    Ys = spY(tq, sseg, dy=1)
    Xst = spX(tq, sseg, dx=1, dy=1);  Yst = spY(tq, sseg, dx=1, dy=1)
    Xstt = spX(tq, sseg, dx=2, dy=1); Ystt = spY(tq, sseg, dx=2, dy=1)

    r2 = Xs ** 2 + Ys ** 2
    num = Xs * Yst - Ys * Xst                 # numerator of theta_dot
    dnum = Xs * Ystt - Ys * Xstt              # d/dt num (Xst*Yst-Yst*Xst = 0)
    dr2 = 2.0 * (Xs * Xst + Ys * Yst)         # d/dt r2
    THd = num / r2
    THdd = (dnum * r2 - num * dr2) / r2 ** 2  # quotient rule

    dq_x, dq_y = Xt, Yt
    dq_th = THd.copy(); dq_th[:, 0] = 0.0
    dp_x = rho * A * Xtt
    dp_y = rho * A * Ytt
    dp_th = rho * Iarea * THdd; dp_th[:, 0] = 0.0
    return np.concatenate([dq_x, dq_y, dq_th, dp_x, dp_y, dp_th], axis=1)


# =============================================================================
# 3.  End-to-end driver
# =============================================================================
def process_rod_video_joint2d(video_path, L0=0.5, b=15e-3, h=20e-3,
                              mass=0.149, N=25, fps=None, thr=135,
                              base_xy=None, mult=1.0,
                              upsample_dt=1e-4, upsample_window=(0.0, 2.0)):
    """Full pipeline: video -> joint-2D-smoothed state z and zdot.

    Returns a dict with the native-grid state/zdot AND an upsampled window
    (default: first 2 s at dt=1e-4). Set upsample_dt=None to skip upsampling.
    """
    # --- 3a. raw centerlines (pixels) ---
    info = find_fixed_base(video_path, thr=thr)
    if base_xy is not None:
        info["xb"], info["yb"] = float(base_xy[0]), float(base_xy[1])
    if fps is None:
        fps = info["fps"]
    Xpx, Ypx, ppm = extract_centerlines(video_path, info["xb"], info["yb"],
                                        info["W"], nframes=info["nframes"],
                                        thr=thr, npts=N + 1, L0=L0)

    # --- 3b. pixels -> metres, y-up frame (gravity -> -y) ---
    xb, yb = info["xb"], info["yb"]
    Xn = (Xpx - xb) / ppm
    Yn = -(Ypx - yb) / ppm
    Nt, NN = Xn.shape
    t = np.arange(Nt) / fps
    snode = np.linspace(0.0, 1.0, NN)          # normalized arclength, nodes
    sseg = (snode[:-1] + snode[1:]) / 2.0      # segment midpoints

    # --- 3c. material / geometry ---
    A = b * h
    rho = mass / (A * L0)
    Iarea = h * b ** 3 / 12.0

    # --- 3d. joint 2-D surface + native-grid state/zdot ---
    spX, spY, sinfo = fit_surfaces(Xn, Yn, t, snode, mult=mult)
    st = state_from_surface(spX, spY, t, snode, sseg, rho, A, Iarea)
    zdot = zdot_from_surface(spX, spY, t, snode, sseg, rho, A, Iarea)

    # theta -> node reconstruction consistency (quality check, mm)
    ds = L0 / N
    xr = np.zeros_like(st["X"]); yr = np.zeros_like(st["Y"])
    xr[:, 1:] = np.cumsum(ds * np.cos(st["TH"]), axis=1)
    yr[:, 1:] = np.cumsum(ds * np.sin(st["TH"]), axis=1)
    xr += st["X"][:, :1]; yr += st["Y"][:, :1]
    recon_mm = float(np.hypot(xr - st["X"], yr - st["Y"]).mean() * 1000)

    out = dict(z=st["z"], zdot=zdot, t=t, N=N, D=st["z"].shape[1], L0=L0,
               A=A, I=Iarea, rho=rho, mass=mass, fps=fps, ppm=ppm,
               xb=xb, yb=yb, thb=st["thb"], recon_mm=recon_mm,
               surf_info=sinfo, spX=spX, spY=spY)

    # --- 3e. upsample a window to fine dt (reuse the SAME surface + clamp) ---
    if upsample_dt is not None:
        t0, t1 = upsample_window
        t1 = min(t1, t[-1])
        tq = np.arange(t0, t1 + upsample_dt / 2, upsample_dt)
        stq = state_from_surface(spX, spY, tq, snode, sseg, rho, A, Iarea,
                                 thb=st["thb"])
        zdotq = zdot_from_surface(spX, spY, tq, snode, sseg, rho, A, Iarea)
        out.update(z_up=stq["z"], zdot_up=zdotq, t_up=tq, dt_up=upsample_dt)
    return out


if __name__ == "__main__":
    import sys
    vid = sys.argv[1] if len(sys.argv) > 1 else "video_0.avi"
    # clamp override matches the validated extraction of the source video
    r = process_rod_video_joint2d(vid, base_xy=(676.0, 48.5))
    print("native z:", r["z"].shape, " zdot:", r["zdot"].shape)
    print("upsampled z:", r["z_up"].shape, " dt:", r["dt_up"],
          " t in [", r["t_up"][0], ",", round(r["t_up"][-1], 4), "]")
    print("ppm:", round(r["ppm"], 1), " recon_mm:", round(r["recon_mm"], 2),
          " fit resid mm:", round(r["surf_info"]["resid_m"] * 1000, 3))
    np.savez("real_state_2Dsmooth.npz",
             z_native=r["z"], zdot_native=r["zdot"], t_native=r["t"],
             z=r["z_up"], zdot=r["zdot_up"], t=r["t_up"], dt=r["dt_up"],
             N=r["N"], D=r["D"], L0=r["L0"], A=r["A"], I=r["I"], rho=r["rho"],
             mass=r["mass"], fps=r["fps"], ppm=r["ppm"], xb=r["xb"], yb=r["yb"],
             thb=r["thb"])
    print("wrote real_state_2Dsmooth.npz")
