#!/usr/bin/env python3
"""
ros_centerline_3.py  — sub-pixel edge detection pipeline (Option 3)

Replaces skeletonization with sub-pixel Canny edge detection + rod centreline
estimation. The key idea: detect both edges of the rod (left and right boundary),
then compute the midpoint between them at each row/column position. This gives
sub-pixel centreline coordinates that are independent of the pixel grid orientation.

Why this fixes the arc-length orientation bias:
  - Skeletonization produces a staircase path → arc length depends on orientation
  - Sub-pixel edge midpoints give continuous (x, y) coordinates → arc length is
    orientation-independent regardless of rod angle

Pipeline per frame:
  1. Threshold + morphological clean (same as ros_centerline_2.py)
  2. Determine dominant orientation from mask bounding box:
       rod spans more rows → scan row-by-row  (Sobel_x, L/R edges)
       rod spans more cols → scan col-by-col  (Sobel_y, T/B edges)
     This is the key fix: always scan along the dominant axis so the number
     of samples is maximised regardless of rod orientation — vertical, horizontal,
     and diagonal rods all get the same number of centreline points per unit length.
  3. For each scanline, find both rod boundaries via sub-pixel quadratic
     interpolation of the Sobel gradient peak → float coordinates
  4. Centreline = midpoint of the two boundaries → orientation-independent arc length
  5. Sort clamp→tip, fit cubic spline, resample to N_OUT=25 points

Same save format as ros_centerline_2.py:
  [x0..x24, y0..y24, frame_index, timestamp] = 52 values per row

Subscribes:  /camera/image_color
Publishes:   /skeleton  (Float32MultiArray, 50 floats)
Saves:       centerline_data.txt

Key bindings: same as ros_centerline_2.py

Usage:
  python3 ros_centerline_3.py --topic /camera/image_color --thresh 120
  rosrun cosserat_skeletonization ros_centerline_3.py
"""

from __future__ import annotations

import argparse
import logging
from dataclasses import dataclass, field
from time import perf_counter, sleep
from typing import Optional, Tuple

import cv2 as cv
import numpy as np
from scipy.interpolate import splprep, splev

try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
    from cv_bridge import CvBridge
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False
    print("WARNING: rospy / cv_bridge not found — ROS functionality disabled.")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
N_OUT  = 25    # number of output centreline points
Point  = Tuple[int, int]
Rect   = Tuple[Point, Point]

SLIDER_MAX    = 200
SLIDER_CENTER = 100


# ---------------------------------------------------------------------------
# BCG (reused from ros_centerline_2.py verbatim)
# ---------------------------------------------------------------------------
def _slider_to_pct(pos: int) -> int:
    return int(np.clip(pos, 0, SLIDER_MAX)) - SLIDER_CENTER


@dataclass
class BCGState:
    bri_pos: int = SLIDER_CENTER
    con_pos: int = SLIDER_CENTER
    gam_pos: int = SLIDER_CENTER

    def is_active(self) -> bool:
        return any(p != SLIDER_CENTER for p in (self.bri_pos, self.con_pos, self.gam_pos))


def apply_bcg(img: np.ndarray, st: BCGState) -> np.ndarray:
    if not st.is_active():
        return img
    x = img.astype(np.float32)
    bri = (_slider_to_pct(st.bri_pos) / 100.0) * 255.0
    con = 1.0 + _slider_to_pct(st.con_pos) / 100.0
    gam = float(2.0 ** (_slider_to_pct(st.gam_pos) / 100.0))
    if bri != 0:
        x += bri
    if con != 1.0:
        x = (x - 128.0) * con + 128.0
    x = np.clip(x, 0, 255)
    if gam != 1.0:
        x = 255.0 * (x / 255.0) ** gam
    return np.clip(x, 0, 255).astype(np.uint8)


class BCGControls:
    def __init__(self, win: str, state: BCGState) -> None:
        self.win = win; self.state = state; self.visible = False

    def _cb_bri(self, v): self.state.bri_pos = v
    def _cb_con(self, v): self.state.con_pos = v
    def _cb_gam(self, v): self.state.gam_pos = v

    def show(self):
        if self.visible: return
        cv.namedWindow(self.win, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.win, 560, 180)
        for lbl, pos, cb in [("brightness (%)", self.state.bri_pos, self._cb_bri),
                              ("contrast   (%)", self.state.con_pos, self._cb_con),
                              ("gamma      (%)", self.state.gam_pos, self._cb_gam)]:
            cv.createTrackbar(lbl, self.win, pos, SLIDER_MAX, cb)
        self.visible = True

    def hide(self):
        if not self.visible: return
        for lbl, attr in [("brightness (%)", "bri_pos"),
                           ("contrast   (%)", "con_pos"),
                           ("gamma      (%)", "gam_pos")]:
            try: setattr(self.state, attr, cv.getTrackbarPos(lbl, self.win))
            except cv.error: pass
        try: cv.destroyWindow(self.win)
        except cv.error: pass
        self.visible = False

    def toggle(self): self.hide() if self.visible else self.show()

    def poll(self):
        if not self.visible: return
        try:
            if cv.getWindowProperty(self.win, cv.WND_PROP_VISIBLE) < 1:
                self.visible = False; return
            self.state.bri_pos = cv.getTrackbarPos("brightness (%)", self.win)
            self.state.con_pos = cv.getTrackbarPos("contrast   (%)", self.win)
            self.state.gam_pos = cv.getTrackbarPos("gamma      (%)", self.win)
        except cv.error:
            self.visible = False


# ---------------------------------------------------------------------------
# Sub-pixel centreline extraction (core of Option 3)
# ---------------------------------------------------------------------------

def _scan_axis(
    mask: np.ndarray,
    sobel_perp: np.ndarray,
    scan_rows: bool,
) -> list:
    """Scan along rows (scan_rows=True) or columns (scan_rows=False).

    For each scanline, finds the two rod boundaries via sub-pixel quadratic
    interpolation of the perpendicular Sobel gradient, then returns the
    midpoint as a sub-pixel (col, row) coordinate.

    scan_rows=True  → scans row by row, Sobel_x detects left/right boundaries
    scan_rows=False → scans col by col, Sobel_y detects top/bottom boundaries
    """
    h, w = mask.shape
    pts = []

    n_lines = h if scan_rows else w
    for i in range(2, n_lines - 2):
        line_mask = mask[i, :] if scan_rows else mask[:, i]
        on = np.where(line_mask > 0)[0]
        if len(on) < 3:
            continue

        lo_px = float(on[0])
        hi_px = float(on[-1])

        grad_line = sobel_perp[i, :] if scan_rows else sobel_perp[:, i]

        # Sub-pixel refinement — low boundary (positive gradient)
        a0, a1 = max(0, int(lo_px) - 3), min(len(grad_line), int(lo_px) + 4)
        g = grad_line[a0:a1]
        if len(g) > 0 and g.max() > 0:
            pi = int(np.argmax(g))
            if 0 < pi < len(g) - 1:
                denom = g[pi-1] - 2*g[pi] + g[pi+1]
                if abs(denom) > 1e-6:
                    pi_sub = pi - 0.5*(g[pi+1] - g[pi-1]) / denom
                    lo_px = a0 + pi_sub

        # Sub-pixel refinement — high boundary (negative gradient)
        b0, b1 = max(0, int(hi_px) - 3), min(len(grad_line), int(hi_px) + 4)
        g = grad_line[b0:b1]
        if len(g) > 0 and g.min() < 0:
            pi = int(np.argmin(g))
            if 0 < pi < len(g) - 1:
                denom = g[pi-1] - 2*g[pi] + g[pi+1]
                if abs(denom) > 1e-6:
                    pi_sub = pi - 0.5*(g[pi+1] - g[pi-1]) / denom
                    hi_px = b0 + pi_sub

        mid = 0.5 * (lo_px + hi_px)
        if scan_rows:
            pts.append((mid, float(i)))   # (col, row)
        else:
            pts.append((float(i), mid))   # (col, row)

    return pts


def _subpixel_centreline(
    mask: np.ndarray,
    clamp_rc: Optional[Tuple[int, int]] = None,
    n_out: int = N_OUT,
) -> Optional[np.ndarray]:
    """Extract rod centreline at sub-pixel resolution — orientation-adaptive.

    Method:
      1. Measure the rod's bounding box to determine dominant orientation.
         - If rod spans more rows than columns → scan row-by-row (use Sobel_x)
         - If rod spans more columns than rows → scan col-by-col (use Sobel_y)
         This ensures maximum sampling density regardless of rod angle, fixing
         the orientation-dependent arc-length bias of the skeletonization approach.
      2. For each scanline, find both rod boundaries via sub-pixel quadratic
         interpolation of the Sobel gradient peak.
      3. Centreline = midpoint of the two boundaries at each scanline → float coords.
      4. Fit cubic spline, resample to n_out equidistant points.

    Returns (n_out, 2) int32 array [col, row] = [x, y], or None.
    """
    if mask is None or mask.sum() == 0:
        return None

    h, w = mask.shape
    mask_f = mask.astype(np.float32) / 255.0

    # Determine dominant orientation from bounding box of rod mask
    rows_on = np.any(mask > 0, axis=1)
    cols_on = np.any(mask > 0, axis=0)
    n_rows_span = int(rows_on.sum())
    n_cols_span = int(cols_on.sum())

    # Choose scan direction: whichever gives more scanlines = more samples
    scan_rows = (n_rows_span >= n_cols_span)

    if scan_rows:
        # Rod is more vertical — scan row by row, Sobel_x detects L/R edges
        sobel_perp = cv.Sobel(mask_f, cv.CV_32F, 1, 0, ksize=3)
    else:
        # Rod is more horizontal — scan col by col, Sobel_y detects T/B edges
        sobel_perp = cv.Sobel(mask_f, cv.CV_32F, 0, 1, ksize=3)

    centreline_pts = _scan_axis(mask, sobel_perp, scan_rows=scan_rows)

    if len(centreline_pts) < 10:
        return None

    pts    = np.array(centreline_pts, dtype=np.float64)
    cols_c = pts[:, 0]
    rows_c = pts[:, 1]

    # Orient clamp→tip
    if clamp_rc is not None:
        cr, cc = clamp_rc
        d_start = (rows_c[0]  - cr)**2 + (cols_c[0]  - cc)**2
        d_end   = (rows_c[-1] - cr)**2 + (cols_c[-1] - cc)**2
        if d_end < d_start:
            rows_c = rows_c[::-1]
            cols_c = cols_c[::-1]
        rows_c[0] = float(cr)
        cols_c[0] = float(cc)
    else:
        # No clamp: orient so smallest row (highest in image) is first
        if rows_c[0] > rows_c[-1]:
            rows_c = rows_c[::-1]
            cols_c = cols_c[::-1]

    # Remove duplicate consecutive points before spline fit
    diffs = np.sqrt(np.diff(cols_c)**2 + np.diff(rows_c)**2)
    keep  = np.concatenate([[True], diffs > 0.1])
    cols_c, rows_c = cols_c[keep], rows_c[keep]

    if len(cols_c) < 4:
        return None

    try:
        tck, _ = splprep([cols_c, rows_c], s=len(cols_c) * 0.5, k=3)
    except Exception:
        return None

    t_out = np.linspace(0.0, 1.0, n_out)
    c_s, r_s = splev(t_out, tck)

    return np.stack([c_s, r_s], axis=1).astype(np.int32)


def extract_centerline(
    frame_bgr: np.ndarray,
    thresh_val: int,
    roi_rect: Optional[Rect],
    overlay: bool,
    invert: bool = False,
    clamp_pt: Optional[Tuple[int, int]] = None,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Returns (display_frame, line_pts) where line_pts is (N_OUT,2) int32 or None."""
    h, w = frame_bgr.shape[:2]
    disp = frame_bgr.copy()

    if roi_rect is not None:
        (x0, y0), (x1, y1) = roi_rect
        x0, x1 = max(0, x0), min(w, x1)
        y0, y1 = max(0, y0), min(h, y1)
        if x1 <= x0 or y1 <= y0:
            roi_rect = None

    if roi_rect is not None:
        (x0, y0), (x1, y1) = roi_rect
        region = frame_bgr[y0:y1, x0:x1]
        offset = (x0, y0)
    else:
        region = frame_bgr
        offset = (0, 0)

    gray = cv.cvtColor(region, cv.COLOR_BGR2GRAY)
    thresh_type = cv.THRESH_BINARY_INV if invert else cv.THRESH_BINARY
    _, mask = cv.threshold(gray, thresh_val, 255, thresh_type)

    # Keep largest connected component
    n_labels, labels, stats, _ = cv.connectedComponentsWithStats(mask, connectivity=8)
    if n_labels < 2:
        return disp, None
    largest = 1 + int(np.argmax(stats[1:, cv.CC_STAT_AREA]))
    clean_mask = np.zeros_like(mask)
    clean_mask[labels == largest] = 255

    # Light morphological cleaning (smaller than ros_centerline_2 to preserve sub-pixel detail)
    kernel_sm = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    kernel_lg = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))
    clean_mask = cv.morphologyEx(clean_mask, cv.MORPH_CLOSE, kernel_lg, iterations=1)
    clean_mask = cv.morphologyEx(clean_mask, cv.MORPH_OPEN,  kernel_sm, iterations=1)

    ox, oy = offset

    # Convert clamp to ROI coords
    clamp_rc = None
    if clamp_pt is not None:
        clamp_rc = (clamp_pt[1] - oy, clamp_pt[0] - ox)  # (row, col)

    line_pts = _subpixel_centreline(clean_mask, clamp_rc=clamp_rc, n_out=N_OUT)
    if line_pts is None:
        return disp, None

    # Shift back to full-image coordinates
    line_pts[:, 0] += ox
    line_pts[:, 1] += oy

    if not overlay:
        mask_bgr = cv.cvtColor(
            cv.resize(clean_mask, (x1-x0 if roi_rect else w, y1-y0 if roi_rect else h)),
            cv.COLOR_GRAY2BGR)
        if roi_rect is not None:
            disp[y0:y1, x0:x1] = mask_bgr
        else:
            disp = mask_bgr

    return disp, line_pts


# ---------------------------------------------------------------------------
# ROI selector (unchanged)
# ---------------------------------------------------------------------------
class ROISelector:
    def __init__(self):
        self.get_roi = self.have_rect = False
        self._start: Point = (-1, -1)
        self.rect: Rect = ((0, 0), (0, 0))

    def clear(self):
        self.get_roi = self.have_rect = False
        self.rect = ((0, 0), (0, 0))

    def begin(self):
        self.get_roi = True; self.have_rect = False
        self.rect = ((0, 0), (0, 0))

    @staticmethod
    def _norm(a, b):
        return (min(a[0],b[0]), min(a[1],b[1])), (max(a[0],b[0]), max(a[1],b[1]))

    def on_mouse(self, event, x, y, flags, _p):
        if not self.get_roi: return
        if event == cv.EVENT_LBUTTONDOWN:
            self._start = (x, y)
        elif event == cv.EVENT_MOUSEMOVE and (flags & cv.EVENT_FLAG_LBUTTON):
            self.rect = self._norm(self._start, (x, y)); self.have_rect = True
        elif event == cv.EVENT_LBUTTONUP:
            self.rect = self._norm(self._start, (x, y))
            (lx,ly),(rx,ry) = self.rect
            self.have_rect = (rx > lx) and (ry > ly)
            self.get_roi = False


# ---------------------------------------------------------------------------
# App state
# ---------------------------------------------------------------------------
@dataclass
class AppState:
    mod_stage: Optional[int] = None
    bcg_state: BCGState      = field(default_factory=BCGState)
    gauss_enabled: bool      = False
    gauss_sigma: int         = 0
    gauss_slider_max: int    = 13
    thresh_val: int          = 100
    thresh_max: int          = 255
    overlay_on: bool         = True
    invert: bool             = False
    paused: bool             = False
    clamp_pt: Optional[Tuple[int, int]] = None
    picking_clamp: bool      = False
    saving: bool             = False
    frames_saved: int        = 0


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
class ROSCenterlineNode:
    WIN     = "ROS Centerline 3 (sub-pixel)"
    WIN_BCG = "S1 BCG"
    TB_THRESH = "Threshold"
    TB_GAUSS  = "Gauss sigma"
    W, H    = 600, 450   # preserves 1440×1080 aspect ratio (4:3)

    ALPHA_MIN      = 0.05
    ALPHA_MAX      = 0.5
    MOTION_THRESH  = 5.0

    def __init__(self, topic: str, thresh: int = 100,
                 save_path: str = "centerline_data.txt", alpha: float = 0.05):
        self.topic     = topic
        self.save_path = save_path
        self.st        = AppState(thresh_val=thresh)
        self.roi       = ROISelector()
        self.bcg_ctrl  = BCGControls(self.WIN_BCG, self.st.bcg_state)
        self._win_ready    = False
        self._last_t       = perf_counter()
        self._alpha        = float(np.clip(alpha, 0.0, 1.0))
        self._ema_pts: Optional[np.ndarray] = None
        self._pending_frame: Optional[np.ndarray] = None
        self._pending_stamp: float = 0.0

        if _HAS_ROS:
            self._bridge = CvBridge()
            self._pub = rospy.Publisher("/skeleton", Float32MultiArray, queue_size=2)
            rospy.Subscriber(self.topic, Image, self._ros_callback,
                             queue_size=1, buff_size=2**24)
            rospy.loginfo("ROSCenterlineNode3: subscribed to %s", self.topic)

    def _ros_callback(self, msg):
        try:
            if msg.encoding in ("mono8", "8UC1"):
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
            else:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(5.0, "cv_bridge error: %s", e)
            return
        self._pending_frame = cv.resize(frame, (self.W, self.H))
        self._pending_stamp = msg.header.stamp.to_sec()

    def _build_window(self, first_frame=None):
        if self._win_ready:
            try: cv.destroyWindow(self.WIN)
            except cv.error: pass
        cv.namedWindow(self.WIN, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.WIN, self.W, self.H)
        cv.setMouseCallback(self.WIN, self._on_mouse)
        cv.createTrackbar(self.TB_THRESH, self.WIN, self.st.thresh_val,
                          self.st.thresh_max, self._cb_thresh)
        if first_frame is not None:
            cv.imshow(self.WIN, first_frame)
        self._win_ready = True

    def _on_mouse(self, event, x, y, flags, _p):
        if self.st.picking_clamp:
            if event == cv.EVENT_LBUTTONDOWN:
                self.st.clamp_pt = (x, y)
                self.st.picking_clamp = False
        else:
            self.roi.on_mouse(event, x, y, flags, _p)

    def _cb_thresh(self, v): self.st.thresh_val = v

    def _hud(self, img, fps):
        lines = [
            f"MOD: S{self.st.mod_stage}" if self.st.mod_stage else "MOD: -",
            f"S1 BCG: {'ON' if self.st.bcg_state.is_active() else 'OFF'}",
            f"S3 Thresh: {self.st.thresh_val}  inv:{'ON' if self.st.invert else 'OFF'}",
            f"S4 ROI: {'ON' if self.roi.have_rect else 'OFF'}",
            f"FPS: {fps:.1f}  [sub-pixel]  {'[PAUSED]' if self.st.paused else ''}",
            f"Clamp: {'PICKING' if self.st.picking_clamp else (str(self.st.clamp_pt) if self.st.clamp_pt else 'OFF (c to set)')}",
            f"Save: {'ON  n=' + str(self.st.frames_saved) if self.st.saving else 'OFF (s to start)'}",
        ]
        for i, txt in enumerate(lines):
            cv.putText(img, txt, (10, 18 + i*18),
                       cv.FONT_HERSHEY_SIMPLEX, 0.46, (0, 0, 220), 1, cv.LINE_AA)

    def _publish(self, line_pts):
        if not _HAS_ROS: return
        flat = np.concatenate([line_pts[:, 0], line_pts[:, 1]]).astype(np.float32)
        n_floats = 2 * N_OUT
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="xy", size=n_floats, stride=n_floats)],
            data_offset=0)
        msg.data = flat.tolist()
        self._pub.publish(msg)

    def _handle_key(self, key, last, fps):
        if key == ord(" "):
            self.st.paused = not self.st.paused
        elif key == ord("c"):
            if self.st.picking_clamp:
                self.st.picking_clamp = False
            elif self.st.clamp_pt is not None:
                self.st.clamp_pt = None
            else:
                self.st.picking_clamp = True
        elif key == ord("s"):
            self.st.saving = not self.st.saving
            if self.st.saving:
                self.st.frames_saved = 0
                open(self.save_path, 'w').close()
        elif key == ord("i"):
            self.st.invert = not self.st.invert
        elif key == ord("0"):
            self.st.overlay_on = not self.st.overlay_on
        elif key in map(ord, "1234"):
            n = key - ord("0")
            self.st.mod_stage = None if self.st.mod_stage == n else n
        elif self.st.mod_stage == 1 and key == ord("a"):
            self.bcg_ctrl.toggle()
        elif self.st.mod_stage == 4 and key == ord("m"):
            if self.roi.have_rect or self.roi.get_roi:
                self.roi.clear()
            else:
                self.roi.begin()

    def run(self):
        logging.info("Waiting for first frame on %s …", self.topic)
        deadline = perf_counter() + 10.0
        while self._pending_frame is None:
            if _HAS_ROS and rospy.is_shutdown(): return 1
            if perf_counter() > deadline:
                logging.error("No frame received within 10s.")
                return 1
            sleep(0.05)

        self._build_window(self._pending_frame)
        last_disp = self._pending_frame.copy()

        while True:
            if _HAS_ROS and rospy.is_shutdown(): break
            self.bcg_ctrl.poll()
            fps_est = 1.0 / max(1e-6, perf_counter() - self._last_t)

            frame = self._pending_frame
            self._pending_frame = None

            if frame is not None and not self.st.paused and not self.st.picking_clamp:
                out = apply_bcg(frame, self.st.bcg_state)
                roi_rect = self.roi.rect if self.roi.have_rect else None
                disp, line_pts = extract_centerline(
                    out, self.st.thresh_val, roi_rect,
                    self.st.overlay_on, self.st.invert, self.st.clamp_pt)

                if line_pts is not None:

                    #### ------ Debug: print arc length and chord length to check for orientation bias ----
                    # Arc length along centerline
                    # t_fine = np.linspace(0.0, 1.0, 2000)  # dense sampling for accurate arc length
                    # c_fine, r_fine = splev(t_fine,tck)
                    # seg = np.diff(np.stack([c_fine, r_fine], axis=1), axis=0)
                    # L_arc = np.sum(np.sqrt(np.sum(seg**2, axis=1)))

                    seg = np.diff(line_pts.astype(np.float64), axis=0)
                    L_arc = np.sum(np.linalg.norm(seg, axis=1))
                    # L_arc = np.sum(np.sqrt(np.sum(seg**2, axis=1)))

                    # End-to-end distance
                    L_chord = np.linalg.norm(
                        line_pts[-1].astype(np.float64) -
                        line_pts[0].astype(np.float64)
                    )

                    print(f"L_arc={L_arc:.2f} px, L_chord={L_chord:.2f} px, ratio={L_arc/L_chord:.4f}")

                    pts_f = line_pts.astype(np.float64)
                    if self._ema_pts is None:
                        self._ema_pts = pts_f
                    else:
                        motion = float(np.mean(np.sqrt(
                            np.sum((pts_f - self._ema_pts)**2, axis=1))))
                        t = min(1.0, motion / self.MOTION_THRESH)
                        alpha = self.ALPHA_MIN + t * (self.ALPHA_MAX - self.ALPHA_MIN)
                        self._ema_pts = alpha * pts_f + (1.0 - alpha) * self._ema_pts

                if self._ema_pts is not None:
                    line_pts = np.round(self._ema_pts).astype(np.int32)
                    cv.polylines(disp, [line_pts.reshape(-1, 1, 2)], False,
                                 (0, 255, 0), 2, cv.LINE_AA)
                    step = max(1, N_OUT // 8)
                    for pt in line_pts[::step]:
                        cv.circle(disp, (int(pt[0]), int(pt[1])), 3, (0, 80, 255), -1)
                    if self.st.clamp_pt is not None:
                        cx, cy = self.st.clamp_pt
                        cv.circle(disp, (cx, cy), 8, (255, 255, 0), 2, cv.LINE_AA)
                        cv.line(disp, (cx-10,cy),(cx+10,cy),(255,255,0),2,cv.LINE_AA)
                        cv.line(disp, (cx,cy-10),(cx,cy+10),(255,255,0),2,cv.LINE_AA)

                    self._publish(line_pts)

                    if self.st.saving:
                        row = np.concatenate([line_pts[:, 0], line_pts[:, 1],
                                              [self.st.frames_saved],
                                              [self._pending_stamp]])
                        with open(self.save_path, 'a') as f:
                            coords = ' '.join(f'{v:.1f}' for v in row[:-1])
                            f.write(f'{coords} {self._pending_stamp:.6f}\n')
                        self.st.frames_saved += 1

                if roi_rect is not None:
                    (x0,y0),(x1,y1) = roi_rect
                    cv.rectangle(disp, (x0,y0),(x1,y1),(255,80,0),2)
                self._hud(disp, fps_est)
                cv.imshow(self.WIN, disp)
                last_disp = disp
                self._last_t = perf_counter()

            elif self.st.paused or self.st.picking_clamp:
                if last_disp is not None:
                    overlay = last_disp.copy()
                    if self.st.paused:
                        cv.putText(overlay, "PAUSED  (space to resume)",
                                   (10, self.H-40), cv.FONT_HERSHEY_SIMPLEX,
                                   0.7, (0,200,255), 2, cv.LINE_AA)
                    if self.st.picking_clamp:
                        cv.putText(overlay, "CLICK to set clamp  (c to cancel)",
                                   (10, self.H-15), cv.FONT_HERSHEY_SIMPLEX,
                                   0.7, (0,255,255), 2, cv.LINE_AA)
                    cv.imshow(self.WIN, overlay)

            key = cv.waitKey(1) & 0xFF
            if key in (27, ord("q")): break
            if key != 255: self._handle_key(key, last_disp, fps_est)
            sleep(0.001)

        self._shutdown()
        return 0

    def _shutdown(self):
        self.bcg_ctrl.hide()
        try: cv.destroyAllWindows()
        except cv.error: pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description="ROS centerline node — sub-pixel edge detection (Option 3)")
    ap.add_argument("--topic",  default="/camera/image_color")
    ap.add_argument("--thresh", type=int, default=100)
    ap.add_argument("--save",   default="centerline_data.txt")
    ap.add_argument("--alpha",  type=float, default=0.05)
    ap.add_argument("--log",    default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=getattr(logging, args.log.upper(), logging.INFO),
                        format="%(levelname)s: %(message)s")

    if not _HAS_ROS:
        logging.error("rospy not available. Install ROS and source setup.bash.")
        return 1

    rospy.init_node("ros_centerline_3", anonymous=False)
    topic  = rospy.get_param("~topic",  args.topic)
    thresh = int(rospy.get_param("~thresh", args.thresh))
    save   = rospy.get_param("~save",   args.save)
    alpha  = float(rospy.get_param("~alpha", args.alpha))

    node = ROSCenterlineNode(topic=topic, thresh=thresh,
                             save_path=save, alpha=alpha)
    return node.run()


if __name__ == "__main__":
    raise SystemExit(main())
