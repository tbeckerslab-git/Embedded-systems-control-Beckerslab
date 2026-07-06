"""Video -> Cosserat-rod state (z) and derivative (zdot) extraction.

Pipeline (matches the CR-GPR paper's synthetic-data format):
  segmentation (threshold + largest CC) -> skeletonize -> base-to-tip
  geodesic ordering (Dijkstra) -> arc-length spline resample to N+1 nodes
  -> spatial spline + temporal Savitzky-Golay smoothing -> port-Hamiltonian
  state z (154 for N=25) -> GP-prefilter analytic time-derivative zdot.

State layout (y-up, gravity = -y), block order matching synthetic z:
  x(N+1), y(N+1), theta(N), px(N+1), py(N+1), ptheta(N)
  x,y,px,py live on N+1 NODES; theta,ptheta on N SEGMENTS.
"""
import numpy as np

DEFAULT_THR = 135
DEFAULT_NNODE = 26
DEFAULT_WIN = 15


def rod_mask(gray, thr=135):
    """Binary mask of the rod: threshold -> largest 8-connected component -> close."""
    import cv2
    m = (gray > thr).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n <= 1:
        return None
    big = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    m = (lab == big).astype(np.uint8)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def skeleton_endpoints(skel):
    from scipy import ndimage as ndi
    k = np.array([[1, 1, 1], [1, 10, 1], [1, 1, 1]])
    conv = ndi.convolve(skel.astype(int), k, mode="constant")
    return np.argwhere(conv == 11)  # center + exactly 1 neighbour -> (row,col)


def find_fixed_base(video_path, thr=135, n_sample=7, band_px=10,
                    clamp_edge=None):
    """Auto-detect the clamped (stationary) end and its cross-section centre.

    Samples n_sample frames, finds skeleton endpoints, and picks the endpoint
    cluster with the smallest spatial spread across time as the clamp. Refines
    to the rod cross-section centre by taking the mask centroid in a thin band
    at the clamp edge (the raw skeleton endpoint snaps to a corner ~half a
    thickness off; the centroid is the true centreline start).

    Returns dict: xb, yb (col,row px), clamp_edge, W, H, fps, nframes.
    """
    import cv2
    from skimage.morphology import skeletonize
    cap = cv2.VideoCapture(video_path)
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    nframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    idxs = np.linspace(0, max(nframes - 1, 0), n_sample).astype(int)
    ep_all, masks = [], []
    for i in idxs:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, fr = cap.read()
        if not ok:
            continue
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
        m = rod_mask(g, thr)
        if m is None:
            continue
        masks.append(m)
        sk = skeletonize(m > 0)
        ep_all.append(skeleton_endpoints(sk))
    # The clamped end is TEMPORALLY STATIONARY: it lands on nearly the same
    # pixel every frame, forming a dense endpoint cluster. The free tip wanders,
    # so its endpoints are spread out. Pick the densest cluster (mode) as base.
    allpts = np.vstack(ep_all).astype(float)  # (M,2) row,col
    radius = 15.0
    counts = np.array([np.sum(np.hypot(*(allpts - p).T) < radius)
                       for p in allpts])
    seed = allpts[int(np.argmax(counts))]
    base_cluster = allpts[np.hypot(*(allpts - seed).T) < radius]
    br, bc = base_cluster.mean(0)  # row,col approximate clamp
    # decide clamp edge from where the base sits relative to the frame
    if clamp_edge is None:
        cands = {"top": br, "bottom": H - br, "left": bc, "right": W - bc}
        clamp_edge = min(cands, key=cands.get)
    # Refine to true rod exit point by scanning row-by-row (or col-by-col) from
    # the clamp edge inward. The rod cross-section is narrow at the very edge;
    # the mask widens once the clamp hardware merges in. Keep only the rows whose
    # width stays within 1.5x the first (narrowest) row — those are pure rod.
    # Exit boundary coordinate = outermost white pixel of those narrow rows.
    # Cross-section centre = centroid in the perpendicular direction.
    # This guarantees: one pixel past xb/yb toward the clamp = black pixel.
    m0 = masks[0]
    ys, xs = np.nonzero(m0)

    def _narrow_rows(coord_outer, coord_inner, step, get_perp):
        """Scan from edge inward; return pixels of all narrow-width scanlines."""
        narrow_coords, narrow_perps = [], []
        ref_width = None
        for c in range(coord_outer, coord_inner + step, step):
            perp = get_perp(c)
            if len(perp) == 0:
                continue
            w = perp.max() - perp.min() + 1
            if ref_width is None:
                ref_width = w
            if w > 1.5 * ref_width:
                break
            narrow_coords.append(np.full(len(perp), c))
            narrow_perps.append(perp)
        return np.concatenate(narrow_coords), np.concatenate(narrow_perps)

    if clamp_edge == "top":
        row_coords, perps = _narrow_rows(
            ys.min(), ys.min() + band_px, 1,
            lambda r: xs[ys == r])
        xb = float(perps.min())                          # leftmost pixel = exit boundary
        exit_col = int(round(xb))
        col_ys = ys[xs == exit_col]
        yb = float((col_ys.min() + col_ys.max()) / 2)   # cross-section centre at exit col

    elif clamp_edge == "bottom":
        row_coords, perps = _narrow_rows(
            ys.max(), ys.max() - band_px, -1,
            lambda r: xs[ys == r])
        xb = float(perps.min())
        exit_col = int(round(xb))
        col_ys = ys[xs == exit_col]
        yb = float((col_ys.min() + col_ys.max()) / 2)

    elif clamp_edge == "left":
        col_coords, perps = _narrow_rows(
            xs.min(), xs.min() + band_px, 1,
            lambda c: ys[xs == c])
        yb = float(perps.min())                          # topmost pixel = exit boundary
        exit_row = int(round(yb))
        row_xs = xs[ys == exit_row]
        xb = float((row_xs.min() + row_xs.max()) / 2)   # cross-section centre at exit row

    else:  # right
        col_coords, perps = _narrow_rows(
            xs.max(), xs.max() - band_px, -1,
            lambda c: ys[xs == c])
        yb = float(perps.min())
        exit_row = int(round(yb))
        row_xs = xs[ys == exit_row]
        xb = float((row_xs.min() + row_xs.max()) / 2)

    cap.release()
    return dict(xb=xb, yb=yb, clamp_edge=clamp_edge, W=W, H=H,
                fps=fps, nframes=nframes)


def ordered_centerline_fast(gray, W, xb, yb, thr=135):
    """Skeleton pixels ordered base->tip via Dijkstra geodesic. Returns (x,y) px."""
    from skimage.morphology import skeletonize
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    m = rod_mask(gray, thr)
    if m is None:
        return None
    sk = skeletonize(m > 0)
    pts = np.argwhere(sk)
    M = len(pts)
    if M < 10:
        return None
    ids = pts[:, 0] * W + pts[:, 1]
    order = np.argsort(ids)
    ids_s = ids[order]
    rows, cols, wts = [], [], []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nb = ids + dr * W + dc
            pos = np.clip(np.searchsorted(ids_s, nb), 0, M - 1)
            hit = ids_s[pos] == nb
            src = np.nonzero(hit)[0]
            dst = order[pos[hit]]
            rows.append(src); cols.append(dst)
            wts.append(np.full(len(src), np.hypot(dr, dc)))
    G = coo_matrix((np.concatenate(wts),
                    (np.concatenate(rows), np.concatenate(cols))),
                   shape=(M, M)).tocsr()
    base_k = int(np.argmin(np.sum((pts - np.array([yb, xb])) ** 2, axis=1)))
    dist, pred = dijkstra(G, indices=base_k, return_predecessors=True)
    df = dist.copy(); df[np.isinf(df)] = -1
    tip_k = int(np.argmax(df))
    p, cur = [], tip_k
    while cur != base_k and cur >= 0:
        p.append(cur); cur = pred[cur]
    p.append(base_k); p = p[::-1]
    return pts[p][:, ::-1].astype(float)


def centerline_corrected(gray, xb, yb, W, thr=135):
    """Ordered centreline anchored at the true clamp centre (xb,yb), hook removed.

    After Dijkstra ordering, the skeleton tip is extended outward along the local
    tangent direction until the mask boundary is reached. This corrects the ~5-10 px
    shortfall introduced by skeletonize() eroding the free tip.
    """
    m = rod_mask(gray, thr)
    poly = ordered_centerline_fast(gray, W, xb, yb, thr)
    if poly is None:
        return None
    d0 = np.hypot(poly[:, 0] - xb, poly[:, 1] - yb)
    poly = poly[int(np.argmin(d0)):]
    poly = np.vstack([[xb, yb], poly])
    keep = [0]
    for k in range(1, len(poly)):
        if np.hypot(*(poly[k] - poly[keep[-1]])) > 1.0:
            keep.append(k)
    poly = poly[keep]

    # Extend tip along local tangent until mask boundary is reached.
    # Use the last few points to estimate the tangent direction robustly.
    if m is not None and len(poly) >= 4:
        n_tan = min(6, len(poly) - 1)
        tangent = poly[-1] - poly[-1 - n_tan]
        tang_len = np.hypot(*tangent)
        if tang_len > 0:
            tangent = tangent / tang_len          # unit vector (dx, dy)
            H_img, W_img = m.shape
            tip = poly[-1].copy()
            for _ in range(40):                   # max 40 px extension
                candidate = tip + tangent
                cx, cy = int(round(candidate[0])), int(round(candidate[1]))
                if cx < 0 or cx >= W_img or cy < 0 or cy >= H_img:
                    break
                if m[cy, cx] == 0:                # stepped outside mask
                    break
                tip = candidate
            if np.hypot(*(tip - poly[-1])) > 0.5:
                poly = np.vstack([poly, tip[np.newaxis, :]])

    return poly


def resample_frame(poly, npts=26):
    """Arc-length spline resample of a centreline to npts nodes. Returns (npts,2) px."""
    from scipy.interpolate import splprep, splev
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(poly, axis=0).T))]
    u = d / d[-1]
    tck, _ = splprep([poly[:, 0], poly[:, 1]], u=u, s=len(poly) * 1.0, k=3)
    xs, ys = splev(np.linspace(0, 1, npts), tck)
    return np.column_stack([xs, ys])


def extract_centerlines(video_path, xb, yb, W, nframes=None,
                        thr=135, npts=26, L0=0.5):
    """Loop over all frames -> node pixel arrays Xpx,Ypx (Nt,npts) and ppm scale.

    ppm (pixels per metre) self-calibrates from median centreline arclength / L0.
    """
    import cv2
    cap = cv2.VideoCapture(video_path)
    if nframes is None:
        nframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    raw = []
    for _ in range(nframes):
        ok, fr = cap.read()
        if not ok:
            break
        raw.append(centerline_corrected(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY),
                                        xb, yb, W, thr))
    cap.release()
    raw = [p for p in raw if p is not None]
    lens = np.array([np.hypot(*np.diff(p, axis=0).T).sum() for p in raw])
    ppm = float(np.median(lens) / L0)
    Nt = len(raw)
    Xpx = np.zeros((Nt, npts)); Ypx = np.zeros((Nt, npts))
    for i, poly in enumerate(raw):
        r = resample_frame(poly, npts)
        Xpx[i], Ypx[i] = r[:, 0], r[:, 1]
    Xpx[:, 0] = xb; Ypx[:, 0] = yb
    return Xpx, Ypx, ppm


def build_state(Xpx, Ypx, xb, yb, ppm, L0=0.5, b=15e-3, h=20e-3, mass=0.149,
                N=25, fps=194.0, win=15):
    """Assemble the 4*(N+1)+2*N port-Hamiltonian state z (Nt, 4N+2N+4).

    Converts pixels to metres in the paper's y-up frame (gravity = -y), applies
    spatial-then-temporal smoothing, computes node momenta px,py = rho*A*vel and
    segment angular momentum ptheta = rho*I*thetadot. First segment is pinned to
    its steady median angle (rigid clamp). Returns a dict.
    """
    from scipy.signal import savgol_filter
    A = b * h
    rho = mass / (A * L0)
    Iarea = h * b ** 3 / 12.0
    dt = 1.0 / fps
    NN = N + 1
    Xn = (Xpx - xb) / ppm          # +x right, metres
    Yn = -(Ypx - yb) / ppm         # +y up (gravity -> -y)
    Nt = Xn.shape[0]
    Xs = savgol_filter(Xn, win, 3, axis=0)
    Ys = savgol_filter(Yn, win, 3, axis=0)
    Xd = savgol_filter(Xn, win, 3, deriv=1, delta=dt, axis=0)
    Yd = savgol_filter(Yn, win, 3, deriv=1, delta=dt, axis=0)
    PXn = rho * A * Xd
    PYn = rho * A * Yd
    TH = np.unwrap(np.arctan2(np.diff(Ys, axis=1), np.diff(Xs, axis=1)), axis=1)
    theta_base = float(np.median(TH[:, 0])) # clamp segment pinned to median angle
    TH[:, 0] = theta_base
    TH = np.unwrap(TH, axis=0)
    THs = savgol_filter(TH, win, 3, axis=0)
    THd = savgol_filter(TH, win, 3, deriv=1, delta=dt, axis=0)
    THd[:, 0] = 0.0
    PTHseg = rho * Iarea * THd
    z = np.concatenate([Xs, Ys, THs, PXn, PYn, PTHseg], axis=1)  # (Nt, 4NN+2N)
    t = np.arange(Nt) * dt
    s_node = np.linspace(0, L0, NN)
    s_seg = (s_node[:-1] + s_node[1:]) / 2
    # theta[seg] -> node reconstruction error (quality check)
    ds = L0 / N
    xr = np.zeros_like(Xs); yr = np.zeros_like(Ys)
    xr[:, 1:] = np.cumsum(ds * np.cos(THs), axis=1)
    yr[:, 1:] = np.cumsum(ds * np.sin(THs), axis=1)
    recon_mm = float(np.hypot(xr - Xs, yr - Ys).mean() * 1000)
    return dict(z=z, t=t, x=Xs, y=Ys, theta=THs, px=PXn, py=PYn, ptheta=PTHseg,
                N=N, D=z.shape[1], L0=L0, A=A, I=Iarea, rho=rho, mass=mass,
                fps=fps, dt=dt, ppm=ppm, theta_base_const=theta_base,
                s_node=s_node, s_seg=s_seg, recon_mm=recon_mm,
                xb=xb, yb=yb)


def gp_state_derivative(z, t, t_query=None):
    """GP (RBF) prefilter of the full state: smoothed z and analytic zdot.

    Fits (lengthscale, noise) by marginal likelihood on the training times t,
    then evaluates the posterior mean and its analytic derivative at t_query.
    If t_query is None, evaluates at the original t (same Nt output).
    If t_query is provided (e.g. dense 1e-4 s grid), output has len(t_query) rows.

    Returns z_smooth (Nq,D), z_dot (Nq,D), lengthscale, noise.
    """
    from scipy.linalg import cho_factor, cho_solve
    from scipy.optimize import minimize
    tt = t.reshape(-1, 1)           # training times  (Nt, 1)
    Nt = z.shape[0]
    mu = z.mean(0); sd = z.std(0); sd[sd == 0] = 1
    Yn = (z - mu) / sd

    def K_rbf(a, b, l):
        return np.exp(-((a - b.T) ** 2) / (2 * l ** 2))

    sample = Yn[:, ::11]

    def neg_lml(logp):
        l, sn = np.exp(logp)
        K = K_rbf(tt, tt, l) + (sn ** 2) * np.eye(Nt)
        try:
            c, low = cho_factor(K)
        except Exception:
            return 1e9
        a = cho_solve((c, low), sample)
        ld = 2 * np.sum(np.log(np.diag(c)))
        return -np.mean(-0.5 * np.sum(sample * a, axis=0) - 0.5 * ld)

    res = minimize(neg_lml, np.log([0.03, 0.1]), method="Nelder-Mead")
    l_opt, sn_opt = np.exp(res.x)

    # solve alpha on training data
    K = K_rbf(tt, tt, l_opt) + (sn_opt ** 2) * np.eye(Nt)
    c, low = cho_factor(K)
    alpha = cho_solve((c, low), Yn)     # (Nt, D)

    # query points: dense grid if t_query given, else original t
    tq = tt if t_query is None else t_query.reshape(-1, 1)

    Ks = K_rbf(tq, tt, l_opt)                           # (Nq, Nt)
    Kd = -((tq - tt.T) / l_opt ** 2) * Ks               # (Nq, Nt)
    z_smooth = (Ks @ alpha) * sd + mu                    # (Nq, D)
    z_dot    = (Kd @ alpha) * sd                         # (Nq, D)
    return z_smooth, z_dot, float(l_opt), float(sn_opt)


def save_rod_datasets(state, z_smooth, z_dot, l_opt, sn_opt, out_prefix="rod",
                      t_query=None):
    """Write state and derivative .mat/.npz in the exact orientations the
    CR-GPR MATLAB loader expects:
      <prefix>_state_dataset.mat : z (Nt,D),    t (Nt,1),  N, L0, ...
      <prefix>_zdot_dataset.mat  : z_dot (Nq,D), t (Nq,1), N, D, ...
    z is at original camera fps; z_smooth and z_dot are at t_query rate
    (or original fps if t_query is None). MATLAB size(z_dot) = [Nq, 154].
    Returns the list of written filenames.
    """
    from scipy.io import savemat
    t = state["t"]
    t_out = t if t_query is None else t_query   # time axis for smooth/dot outputs
    tcol = t_out.reshape(-1, 1)
    dt_out = float(t_out[1] - t_out[0]) if len(t_out) > 1 else state["dt"]
    z = state["z"]
    sf = f"{out_prefix}_state_dataset.mat"
    zf = f"{out_prefix}_zdot_dataset.mat"
    common = dict(N=float(state["N"]), L0=state["L0"], A=state["A"],
                  I=state["I"], rho=state["rho"], fps=state["fps"],
                  dt=state["dt"],          # original camera dt
                  dt_out=dt_out,           # output dt (1e-4 if upsampled)
                  s_node=state["s_node"].reshape(-1, 1),
                  s_seg=state["s_seg"].reshape(-1, 1))
    # state file: original camera-rate z + original t
    t_orig_col = t.reshape(-1, 1)
    savemat(sf, dict(common, z=z, t=t_orig_col, D=float(state["D"]),
                     mass=state["mass"], ppm=state["ppm"],
                     theta_base_const=state["theta_base_const"],
                     x=state["x"], y=state["y"], theta=state["theta"],
                     px=state["px"], py=state["py"], ptheta=state["ptheta"],
                     state_layout="z=[x(N+1),y(N+1),theta(N),px(N+1),py(N+1),"
                                  "ptheta(N)], y-up, rows=time cols=state"))
    # zdot file: GP-upsampled z_smooth and z_dot at t_out grid
    savemat(zf, dict(common, z_dot=z_dot, z_smooth=z_smooth, t=tcol,
                     D=float(state["D"]), gp_lengthscale=l_opt, gp_noise=sn_opt,
                     state_layout="z_dot=[xdot(N+1),ydot(N+1),thetadot(N),"
                                  "pxdot(N+1),pydot(N+1),pthetadot(N)], "
                                  "rows=time cols=state, y-up"))
    np.savez(f"{out_prefix}_state_dataset.npz", z=z, t=t, **{
        k: state[k] for k in ("x", "y", "theta", "px", "py", "ptheta",
                              "s_node", "s_seg")})
    np.savez(f"{out_prefix}_zdot_dataset.npz", z_dot=z_dot, z_smooth=z_smooth,
             t=t_out, gp_lengthscale=l_opt, gp_noise=sn_opt)
    return [sf, zf, f"{out_prefix}_state_dataset.npz",
            f"{out_prefix}_zdot_dataset.npz"]


def process_rod_video(video_path, L0=0.5, b=15e-3, h=20e-3, mass=0.149, N=25,
                      fps=None, thr=135, win=15, base_xy=None,
                      clamp_edge=None, out_prefix="rod", write_files=True,
                      target_fps=None):
    """End-to-end: .avi -> (z, zdot) datasets in the CR-GPR synthetic format.

    base_xy=(col,row) overrides auto base detection; clamp_edge forces the
    stationary edge; fps overrides the video's stored rate.
    target_fps: if set, GP output is evaluated on a dense uniform grid at this
    rate (e.g. target_fps=10000 gives dt=1e-4 s matching the Verlet integrator).
    z is always saved at the original camera rate; z_smooth and z_dot are saved
    at target_fps (or original fps if target_fps is None).
    Returns a dict with state, z_smooth, z_dot, t_query, gp hyperparameters,
    base info, and written files.
    """
    info = find_fixed_base(video_path, thr=thr, clamp_edge=clamp_edge)
    if base_xy is not None:
        info["xb"], info["yb"] = float(base_xy[0]), float(base_xy[1])
    if fps is None:
        fps = info["fps"]
    Xpx, Ypx, ppm = extract_centerlines(video_path, info["xb"], info["yb"],
                                        info["W"], nframes=info["nframes"],
                                        thr=thr, npts=N + 1, L0=L0)
    state = build_state(Xpx, Ypx, info["xb"], info["yb"], ppm, L0=L0, b=b, h=h,
                        mass=mass, N=N, fps=fps, win=win)
    t_orig = state["t"]
    if target_fps is not None:
        dt_target = 1.0 / target_fps
        t_query = np.arange(t_orig[0], t_orig[-1] + dt_target * 0.5, dt_target)
        print(f"  Upsampling GP output: {len(t_orig)} → {len(t_query)} steps "
              f"(dt={dt_target:.2e} s, {target_fps} Hz)")
    else:
        t_query = None
    z_smooth, z_dot, l_opt, sn_opt = gp_state_derivative(
        state["z"], t_orig, t_query=t_query)
    files = save_rod_datasets(state, z_smooth, z_dot, l_opt, sn_opt,
                              out_prefix, t_query=t_query) if write_files else []
    return dict(state=state, z_smooth=z_smooth, z_dot=z_dot,
                t_query=t_query, gp_lengthscale=l_opt, gp_noise=sn_opt,
                base=info, Xpx=Xpx, Ypx=Ypx, ppm=ppm, files=files)
