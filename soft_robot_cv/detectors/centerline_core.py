"""
centerline_core.py — sub-pixel rod centreline extraction.

Single source of truth for the contour-midpoint algorithm used by:
  - skeletonization_v3.ipynb  (offline notebook pipeline)
  - ros_centerline_2.py       (real-time ROS node)

Public API
----------
process_frame(frame_bgr, roi, thresh_val, invert, clamp_px,
              n_out, morph_close_k, morph_open_k, smooth_k)
    Full per-frame pipeline.  Returns (line_pts, debug_mask) or (None, None).
    line_pts : (n_out, 2) float64 array  [col, row] = [x, y]
    debug_mask : uint8 binary mask after morphology

_contour_midpoints(clean_mask)
    Low-level: returns (mpts, widths, tip_pt, cut_idx, scan_rows).

_subpixel_centreline(mask, clamp_rc, n_out, smooth_k)
    Mid-level: returns (n_out, 2) float64 or None.

Default constants (override per-call via keyword arguments)
-----------------------------------------------------------
N_OUT            = 26     number of output nodes (must match MATLAB pipeline N+1)
SPLINE_SMOOTH_K  = 0.05   splprep s = smooth_k * n_points  (keep small: 0.01–0.1)
THRESH_VAL       = 150    binary threshold (rod brighter than background)
MORPH_CLOSE_K    = 7      morphological closing kernel size (px)
MORPH_OPEN_K     = 5      morphological opening kernel size (px)
MIN_SCANLINE_PTS = 10     minimum reliable scanlines to attempt spline fit
WIDTH_FRAC       = 0.8    stop at last scanline where width >= WIDTH_FRAC * nominal_w
"""

from __future__ import annotations
from typing import Optional, Tuple

import cv2
import numpy as np
from scipy.interpolate import splprep, splev

# ---------------------------------------------------------------------------
# Default constants — import and override in the notebook config cell
# ---------------------------------------------------------------------------
N_OUT            = 26
SPLINE_SMOOTH_K  = 0.05
THRESH_VAL       = 150
MORPH_CLOSE_K    = 7
MORPH_OPEN_K     = 5
MIN_SCANLINE_PTS = 10
WIDTH_FRAC       = 0.8


# ---------------------------------------------------------------------------
# Core algorithm
# ---------------------------------------------------------------------------

def _contour_midpoints(
    clean_mask: np.ndarray,
    width_frac: float = WIDTH_FRAC,
) -> Tuple:
    """Extract centreline midpoints from a binary rod mask.

    For each scanline (row or column, chosen by rod orientation), computes the
    midpoint between the left and right contour edges.  Stops at the last
    scanline where width >= width_frac * nominal_width, excluding the asymmetric
    taper zone near the round tip where one edge reverses before the other.

    Parameters
    ----------
    clean_mask  : uint8 binary mask, rod = 255
    width_frac  : cutoff as fraction of nominal rod width (default 0.8)

    Returns
    -------
    mpts      : (M, 2) float64  [col, row] midpoints (all scanlines)
    widths    : (M,)   float64  edge-to-edge width per scanline
    tip_pt    : (2,)   float64  [col, row] of last contour scanline (exact tip)
    cut_idx   : int    first index to exclude (use mpts[:cut_idx])
    scan_rows : bool   True if rod was taller than wide (scanning rows)
    On failure returns (None, None, None, None, None).
    """
    contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None, None, None, None, None
    cnt = max(contours, key=len).reshape(-1, 2)
    if len(cnt) < 10:
        return None, None, None, None, None

    r_span = int(cnt[:, 1].max()) - int(cnt[:, 1].min())
    c_span = int(cnt[:, 0].max()) - int(cnt[:, 0].min())
    scan_rows = (r_span >= c_span)

    if scan_rows:
        scan_vals = np.unique(cnt[:, 1])
        mpts, widths = [], []
        for sv in scan_vals:
            cols_at = cnt[cnt[:, 1] == sv, 0]
            lo, hi  = float(cols_at.min()), float(cols_at.max())
            mpts.append((0.5 * (lo + hi), float(sv)))
            widths.append(hi - lo)
    else:
        scan_vals = np.unique(cnt[:, 0])
        mpts, widths = [], []
        for sv in scan_vals:
            rows_at = cnt[cnt[:, 0] == sv, 1]
            lo, hi  = float(rows_at.min()), float(rows_at.max())
            mpts.append((float(sv), 0.5 * (lo + hi)))
            widths.append(hi - lo)

    mpts   = np.array(mpts,   dtype=np.float64)
    widths = np.array(widths, dtype=np.float64)
    tip_pt = mpts[-1].copy()

    # Nominal width: median over the well-sampled interior
    # (ignore thinnest 20% at each end — clamp hardware and tip taper)
    n = len(widths)
    interior  = widths[n // 5 : 4 * n // 5]
    nominal_w = float(np.median(interior)) if len(interior) else 12.0
    reliable_w = nominal_w * width_frac

    # Stop at last scanline where both edges are still symmetric enough.
    # width >= reliable_w excludes the round-tip taper zone where the
    # left/right edges narrow asymmetrically and bias the midpoint laterally.
    reliable = np.where(widths >= reliable_w)[0]
    cut_idx  = int(reliable[-1]) + 1 if len(reliable) else len(mpts)

    return mpts, widths, tip_pt, cut_idx, scan_rows


def _subpixel_centreline(
    mask: np.ndarray,
    clamp_rc: Optional[Tuple[float, float]] = None,
    n_out: int = N_OUT,
    smooth_k: float = SPLINE_SMOOTH_K,
    min_scanline_pts: int = MIN_SCANLINE_PTS,
    width_frac: float = WIDTH_FRAC,
) -> Optional[np.ndarray]:
    """Fit a smoothing spline through reliable contour midpoints.

    Parameters
    ----------
    mask         : uint8 binary mask (rod region = 255)
    clamp_rc     : (row, col) of clamp pixel in mask coordinates, or None
    n_out        : number of output nodes
    smooth_k     : splprep smoothing factor s = smooth_k * n_points
                   0.05 → s ≈ 47 for ~950 midpoints → <0.1 px endpoint error
    min_scanline_pts : minimum reliable scanlines needed to attempt spline fit
    width_frac   : passed to _contour_midpoints

    Returns
    -------
    (n_out, 2) float64 array [col, row], or None on failure.
    Node 0 is snapped to exact clamp pixel when clamp_rc is provided.
    """
    if mask is None or mask.sum() == 0:
        return None

    result = _contour_midpoints(mask, width_frac=width_frac)
    if result[0] is None:
        return None
    mpts, widths, tip_pt, cut_idx, scan_rows = result

    if cut_idx < min_scanline_pts:
        return None

    cols_c = mpts[:cut_idx, 0].copy()
    rows_c = mpts[:cut_idx, 1].copy()

    # Orient so clamp end comes first
    if clamp_rc is not None:
        cr, cc = float(clamp_rc[0]), float(clamp_rc[1])
        d0 = (rows_c[0]  - cr) ** 2 + (cols_c[0]  - cc) ** 2
        dn = (rows_c[-1] - cr) ** 2 + (cols_c[-1] - cc) ** 2
        if dn < d0:
            cols_c = cols_c[::-1]
            rows_c = rows_c[::-1]

    # Remove duplicate consecutive points (splprep requires distinct knots)
    diffs = np.sqrt(np.diff(cols_c) ** 2 + np.diff(rows_c) ** 2)
    keep  = np.concatenate([[True], diffs > 0.1])
    cols_c = cols_c[keep]
    rows_c = rows_c[keep]
    if len(cols_c) < 6:
        return None

    # Arc-length parameterisation
    seg = np.sqrt(np.diff(cols_c) ** 2 + np.diff(rows_c) ** 2)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    if arc[-1] < 5.0:
        return None

    try:
        tck, _ = splprep([cols_c, rows_c], u=arc, s=smooth_k * len(cols_c), k=3)
    except Exception:
        return None

    c_s, r_s = splev(np.linspace(0.0, arc[-1], n_out), tck)
    result_out = np.stack([c_s, r_s], axis=1)

    # Snap node 0 to exact clamp pixel
    if clamp_rc is not None:
        result_out[0, 0] = float(clamp_rc[1])   # col = x
        result_out[0, 1] = float(clamp_rc[0])   # row = y

    return result_out


def process_frame(
    frame_bgr: np.ndarray,
    roi: Optional[Tuple[int, int, int, int]],
    thresh_val: int = THRESH_VAL,
    invert: bool = False,
    clamp_px: Optional[Tuple[float, float]] = None,
    n_out: int = N_OUT,
    morph_close_k: int = MORPH_CLOSE_K,
    morph_open_k: int = MORPH_OPEN_K,
    smooth_k: float = SPLINE_SMOOTH_K,
    width_frac: float = WIDTH_FRAC,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Full per-frame centreline extraction pipeline.

    Parameters
    ----------
    frame_bgr   : BGR image from cv2.VideoCapture
    roi         : (x1, y1, x2, y2) crop rectangle, or None for full frame
    thresh_val  : binary threshold value (rod brighter than background)
    invert      : True if rod is darker than background
    clamp_px    : (col, row) = (x, y) clamp pixel in full-frame coordinates
    n_out       : number of output centreline nodes
    morph_close_k : morphological closing kernel size
    morph_open_k  : morphological opening kernel size
    smooth_k    : spline smoothing factor (see _subpixel_centreline)
    width_frac  : tip cutoff fraction (see _contour_midpoints)

    Returns
    -------
    line_pts   : (n_out, 2) float64 [col, row] in full-frame coordinates, or None
    debug_mask : uint8 binary mask after morphology (ROI coords), or None
    """
    if roi is not None:
        x1, y1, x2, y2 = roi
        region = frame_bgr[y1:y2, x1:x2]
        ox, oy = x1, y1
    else:
        region = frame_bgr
        ox, oy = 0, 0

    gray  = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
    ttype = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    _, mask = cv2.threshold(gray, thresh_val, 255, ttype)

    n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n_lab < 2:
        return None, None
    largest    = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    clean_mask = np.zeros_like(mask)
    clean_mask[labels == largest] = 255

    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_close_k, morph_close_k))
    k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_open_k,  morph_open_k))
    clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, k_close, iterations=1)
    clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_OPEN,  k_open,  iterations=1)

    clamp_rc = None
    if clamp_px is not None:
        clamp_rc = (clamp_px[1] - oy, clamp_px[0] - ox)   # (row, col) in ROI coords

    line_pts = _subpixel_centreline(
        clean_mask,
        clamp_rc=clamp_rc,
        n_out=n_out,
        smooth_k=smooth_k,
        width_frac=width_frac,
    )
    if line_pts is None:
        return None, clean_mask

    line_pts[:, 0] += ox
    line_pts[:, 1] += oy

    return line_pts, clean_mask
