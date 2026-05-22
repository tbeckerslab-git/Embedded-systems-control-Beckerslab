#!/usr/bin/env python3
"""
ros_centerline.py

ROS node that runs the live_centerline skeletonization pipeline on a ROS
camera topic and publishes the 100-point centerline as Float32MultiArray.

Subscribes:
  /camera/image_color  (sensor_msgs/Image)   — change with --topic

Publishes:
  /skeleton            (std_msgs/Float32MultiArray)
      layout: flat array  [x0, x1, …, x99, y0, y1, …, y99]  (200 floats)
      Same order as centerline_data.txt used by the notebook pipeline.

Saves:
  centerline_data.txt  in the working directory when 's' is pressed
  (same format as live_centerline.py — compatible with convert_centerline.py)

Key bindings (same as live_centerline.py):
  q / ESC : quit
  s       : start / stop saving centerline to centerline_data.txt
  c       : click to set clamp point / clear existing clamp
  i       : toggle invert threshold
  0       : toggle overlay vs mask-only view
  space   : pause / resume
  1 → a   : BCG sliders window
  2 → g   : toggle Gaussian blur
  4 → m   : draw / clear ROI
  Trackbar: Threshold  (and Gauss sigma when blur is ON)

Usage:
  rosrun cosserat_skeletonization ros_centerline.py
  rosrun cosserat_skeletonization ros_centerline.py _topic:=/camera/image_raw
  python3 scripts/ros_centerline.py --topic /camera/image_color --thresh 120
"""

from __future__ import annotations

import argparse
import logging
from collections import deque
from dataclasses import dataclass, field
from time import perf_counter, sleep
from typing import Deque, List, Optional, Tuple, Union

import cv2 as cv
import numpy as np
from skimage.morphology import skeletonize

# ROS imports — guarded so the file can be imported / linted without ROS
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
# Re-use the entire processing core from live_centerline.py verbatim
# ---------------------------------------------------------------------------
Point = Tuple[int, int]
Rect  = Tuple[Point, Point]

SLIDER_MAX    = 200
SLIDER_CENTER = 100


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
        self.win   = win
        self.state = state
        self.visible = False

    def _cb_bri(self, v: int) -> None: self.state.bri_pos = v
    def _cb_con(self, v: int) -> None: self.state.con_pos = v
    def _cb_gam(self, v: int) -> None: self.state.gam_pos = v

    def show(self) -> None:
        if self.visible:
            return
        cv.namedWindow(self.win, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.win, 560, 180)
        for lbl, pos, cb in [
            ("brightness (%)", self.state.bri_pos, self._cb_bri),
            ("contrast   (%)", self.state.con_pos, self._cb_con),
            ("gamma      (%)", self.state.gam_pos, self._cb_gam),
        ]:
            cv.createTrackbar(lbl, self.win, pos, SLIDER_MAX, cb)
        self.visible = True

    def hide(self) -> None:
        if not self.visible:
            return
        for lbl, attr in [
            ("brightness (%)", "bri_pos"),
            ("contrast   (%)", "con_pos"),
            ("gamma      (%)", "gam_pos"),
        ]:
            try:
                setattr(self.state, attr, cv.getTrackbarPos(lbl, self.win))
            except cv.error:
                pass
        try:
            cv.destroyWindow(self.win)
        except cv.error:
            pass
        self.visible = False

    def toggle(self) -> None:
        self.hide() if self.visible else self.show()

    def poll(self) -> None:
        if not self.visible:
            return
        try:
            if cv.getWindowProperty(self.win, cv.WND_PROP_VISIBLE) < 1:
                self.visible = False
                return
            self.state.bri_pos = cv.getTrackbarPos("brightness (%)", self.win)
            self.state.con_pos = cv.getTrackbarPos("contrast   (%)", self.win)
            self.state.gam_pos = cv.getTrackbarPos("gamma      (%)", self.win)
        except cv.error:
            self.visible = False


_SQRT2_OVER_2 = float(np.sqrt(2.0) / 2.0)


def apply_gaussian(img: np.ndarray, int_sigma: int) -> np.ndarray:
    if int_sigma <= 0:
        return img
    s = float(int_sigma) * _SQRT2_OVER_2
    k = int(np.rint(((((s - 0.8) / 0.3) + 1.0) / 0.5) + 1.0))
    k = max(1, k + (k % 2 == 0))
    kern = cv.getGaussianKernel(k, s)
    b, g, r = cv.split(img)
    def filt(ch): return cv.sepFilter2D(ch, cv.CV_32F, kern, kern,
                                         borderType=cv.BORDER_REFLECT101)
    return cv.merge([np.clip(filt(c), 0, 255).astype(np.uint8) for c in (b, g, r)])


def _prune_skeleton(skel: np.ndarray, min_branch: int = 30) -> np.ndarray:
    skel = skel.copy().astype(np.uint8)
    kern = np.ones((3, 3), dtype=np.uint8)
    for _ in range(min_branch):
        neighbour_count = cv.filter2D(skel, -1, kern,
                                       borderType=cv.BORDER_CONSTANT) * skel
        endpoints = (neighbour_count == 2).astype(np.uint8) * skel
        if not endpoints.any():
            break
        skel = skel & ~endpoints
    return skel.astype(bool)


def _longest_path_on_skeleton(skel_img: np.ndarray) -> np.ndarray:
    pts = np.argwhere(skel_img)
    if len(pts) == 0:
        return pts
    h, w = skel_img.shape
    idx_map = np.full((h, w), -1, dtype=np.int32)
    idx_map[skel_img] = np.arange(len(pts), dtype=np.int32)

    def bfs_from(start_idx: int):
        dist   = np.full(len(pts), -1, dtype=np.int32)
        parent = np.full(len(pts), -1, dtype=np.int32)
        dist[start_idx] = 0
        queue = deque([start_idx])
        far_idx, far_dist = start_idx, 0
        while queue:
            ci = queue.popleft()
            r, c = int(pts[ci][0]), int(pts[ci][1])
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        ni = idx_map[nr, nc]
                        if ni >= 0 and dist[ni] == -1:
                            dist[ni] = dist[ci] + 1
                            parent[ni] = ci
                            queue.append(ni)
                            if dist[ni] > far_dist:
                                far_dist, far_idx = dist[ni], ni
        return far_idx, far_dist, parent

    def trace(ep_b, parent):
        path = []
        cur = ep_b
        while cur != -1:
            path.append(cur)
            cur = int(parent[cur])
        return pts[path]

    skel_u8 = skel_img.astype(np.uint8)
    kern = np.ones((3, 3), dtype=np.uint8)
    neighbour_count = cv.filter2D(skel_u8, -1, kern,
                                   borderType=cv.BORDER_CONSTANT) * skel_u8
    endpoint_mask   = (neighbour_count == 2)
    endpoint_indices = [idx_map[r, c]
                        for r, c in np.argwhere(endpoint_mask)
                        if idx_map[r, c] >= 0]

    if len(endpoint_indices) >= 2:
        best_len, best_path = 0, pts[:1]
        for start in endpoint_indices:
            ep_far, d, parent = bfs_from(start)
            if d > best_len:
                best_len = d
                best_path = trace(ep_far, parent)
        return best_path
    else:
        start = int(np.argmax(pts[:, 0]))
        ep_a, _, _ = bfs_from(start)
        ep_b, _, parent = bfs_from(ep_a)
        return trace(ep_b, parent)


def _smooth_centerline(pts_rc: np.ndarray, n_out: int = 100) -> Optional[np.ndarray]:
    if len(pts_rc) < 10:
        return None
    rows = pts_rc[:, 0].astype(np.float32)
    cols = pts_rc[:, 1].astype(np.float32)
    dr = np.diff(rows); dc = np.diff(cols)
    arc = np.concatenate([[0.0], np.cumsum(np.sqrt(dr**2 + dc**2))])
    if arc[-1] < 5:
        return None
    t_new = np.linspace(0.0, arc[-1], n_out)
    r_new = np.interp(t_new, arc, rows)
    c_new = np.interp(t_new, arc, cols)
    w = max(3, n_out // 8) | 1
    kernel = np.ones(w, dtype=np.float32) / w
    r_s = np.convolve(r_new, kernel, mode='valid')
    c_s = np.convolve(c_new, kernel, mode='valid')
    t_v = np.linspace(0.0, 1.0, len(r_s))
    t_o = np.linspace(0.0, 1.0, n_out)
    return np.stack([np.interp(t_o, t_v, c_s),
                     np.interp(t_o, t_v, r_s)], axis=1).astype(np.int32)


def extract_centerline(
    frame_bgr: np.ndarray,
    thresh_val: int,
    roi_rect: Optional[Rect],
    overlay: bool,
    invert: bool = False,
    clamp_pt: Optional[Tuple[int, int]] = None,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Returns (display_frame, line_pts) where line_pts is (100,2) int32 or None."""
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

    n_labels, labels, stats, _ = cv.connectedComponentsWithStats(mask, connectivity=8)
    if n_labels < 2:
        return disp, None
    largest = 1 + int(np.argmax(stats[1:, cv.CC_STAT_AREA]))
    clean_mask = np.zeros_like(mask)
    clean_mask[labels == largest] = 255

    kernel_sm = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    kernel_lg = cv.getStructuringElement(cv.MORPH_ELLIPSE, (15, 15))
    clean_mask = cv.morphologyEx(clean_mask, cv.MORPH_CLOSE, kernel_lg, iterations=3)
    clean_mask = cv.morphologyEx(clean_mask, cv.MORPH_OPEN,  kernel_sm, iterations=1)

    skel_bool = skeletonize(clean_mask > 0)
    skel_bool = _prune_skeleton(skel_bool, min_branch=5)

    skel_bool[:2, :]  = False
    skel_bool[-2:, :] = False
    skel_bool[:, :2]  = False
    skel_bool[:, -2:] = False

    skel_u8 = skel_bool.astype(np.uint8) * 255
    n_labels, labels, stats, _ = cv.connectedComponentsWithStats(skel_u8, connectivity=8)
    if n_labels < 2:
        return disp, None
    largest_skel = 1 + int(np.argmax(stats[1:, cv.CC_STAT_AREA]))
    skel_bool = labels == largest_skel

    if np.argwhere(skel_bool).shape[0] < 10:
        return disp, None

    sorted_pts = _longest_path_on_skeleton(skel_bool)
    line_pts   = _smooth_centerline(sorted_pts, n_out=100)
    if line_pts is None:
        return disp, None

    ox, oy = offset
    line_pts[:, 0] += ox
    line_pts[:, 1] += oy

    if not overlay:
        mask_bgr = cv.cvtColor(
            cv.resize(clean_mask,
                      (x1 - x0 if roi_rect else w, y1 - y0 if roi_rect else h)),
            cv.COLOR_GRAY2BGR,
        )
        if roi_rect is not None:
            disp[y0:y1, x0:x1] = mask_bgr
        else:
            disp = mask_bgr

    if clamp_pt is not None:
        cx, cy = clamp_pt
        d_start = (int(line_pts[0, 0]) - cx)**2 + (int(line_pts[0, 1]) - cy)**2
        d_end   = (int(line_pts[-1, 0]) - cx)**2 + (int(line_pts[-1, 1]) - cy)**2
        if d_end < d_start:
            line_pts = line_pts[::-1]
        line_pts[0] = [cx, cy]

    cv.polylines(disp, [line_pts.reshape(-1, 1, 2)], False,
                 (0, 255, 0), 2, cv.LINE_AA)
    for pt in line_pts[::3]:
        cv.circle(disp, (int(pt[0]), int(pt[1])), 3, (0, 80, 255), -1)

    if clamp_pt is not None:
        cx, cy = clamp_pt
        cv.circle(disp, (cx, cy), 8,  (255, 255, 0), 2, cv.LINE_AA)
        cv.line(disp, (cx - 10, cy), (cx + 10, cy), (255, 255, 0), 2, cv.LINE_AA)
        cv.line(disp, (cx, cy - 10), (cx, cy + 10), (255, 255, 0), 2, cv.LINE_AA)

    return disp, line_pts


# ---------------------------------------------------------------------------
# ROI selector (unchanged from live_centerline.py)
# ---------------------------------------------------------------------------
class ROISelector:
    def __init__(self) -> None:
        self.get_roi   = False
        self.have_rect = False
        self._start: Point = (-1, -1)
        self.rect: Rect = ((0, 0), (0, 0))

    def clear(self) -> None:
        self.get_roi = self.have_rect = False
        self.rect = ((0, 0), (0, 0))

    def begin(self) -> None:
        self.get_roi   = True
        self.have_rect = False
        self.rect = ((0, 0), (0, 0))

    @staticmethod
    def _norm(a: Point, b: Point) -> Rect:
        return (min(a[0], b[0]), min(a[1], b[1])), (max(a[0], b[0]), max(a[1], b[1]))

    def on_mouse(self, event, x, y, flags, _p) -> None:
        if not self.get_roi:
            return
        if event == cv.EVENT_LBUTTONDOWN:
            self._start = (x, y)
        elif event == cv.EVENT_MOUSEMOVE and (flags & cv.EVENT_FLAG_LBUTTON):
            self.rect = self._norm(self._start, (x, y))
            self.have_rect = True
        elif event == cv.EVENT_LBUTTONUP:
            self.rect = self._norm(self._start, (x, y))
            (lx, ly), (rx, ry) = self.rect
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
# ROS centerline node
# ---------------------------------------------------------------------------
class ROSCenterlineNode:
    WIN     = "ROS Centerline"
    WIN_BCG = "S1 BCG"
    TB_GAUSS  = "Gauss sigma"
    TB_THRESH = "Threshold"
    # W, H    = 2100, 1300
    W, H    = 1200,680
    # W, H    = 320, 240
    # W, H    = 240,180

    def __init__(self, topic: str, thresh: int = 100, save_path: str = "centerline_data.txt",
                 alpha: float = 0.3) -> None:
        self.topic      = topic
        self.save_path  = save_path
        self.st         = AppState(thresh_val=thresh)
        self.roi        = ROISelector()
        self.bcg_ctrl   = BCGControls(self.WIN_BCG, self.st.bcg_state)
        self._win_ready = False
        self._last_t    = perf_counter()
        self._frame_count = 0   # monotonically increasing ROS frame counter

        # Temporal EMA smoothing: output = alpha*new + (1-alpha)*previous
        # alpha=1.0 → no smoothing (raw); alpha~0.2 → very stable, slow to respond
        self._alpha: float = float(np.clip(alpha, 0.0, 1.0))
        self._ema_pts: Optional[np.ndarray] = None   # float64 (100, 2)

        # Latest decoded frame from ROS callback (protected by GIL — single writer)
        self._pending_frame: Optional[np.ndarray] = None

        if _HAS_ROS:
            self._bridge = CvBridge()
            self._pub    = rospy.Publisher(
                "/skeleton", Float32MultiArray, queue_size=2
            )
            rospy.Subscriber(self.topic, Image, self._ros_callback, queue_size=1,
                             buff_size=2**24)
            rospy.loginfo("ROSCenterlineNode: subscribed to %s", self.topic)

    # ---- ROS callback (runs in a separate thread) --------------------------

    def _ros_callback(self, msg: "Image") -> None:
        try:
            # Accept both mono8 and bgr8 / rgb8 images
            if msg.encoding in ("mono8", "8UC1"):
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
            else:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(5.0, "cv_bridge error: %s", e)
            return
        self._pending_frame = cv.resize(frame, (self.W, self.H))

    # ---- OpenCV window setup -----------------------------------------------

    def _build_window(self, first_frame: Optional[np.ndarray] = None) -> None:
        if self._win_ready:
            try:
                cv.destroyWindow(self.WIN)
            except cv.error:
                pass
        cv.namedWindow(self.WIN, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.WIN, self.W, self.H)
        cv.setMouseCallback(self.WIN, self._on_mouse)
        if self.st.gauss_enabled:
            cv.createTrackbar(self.TB_GAUSS, self.WIN, self.st.gauss_sigma,
                              self.st.gauss_slider_max, self._cb_gauss)
        cv.createTrackbar(self.TB_THRESH, self.WIN, self.st.thresh_val,
                          self.st.thresh_max, self._cb_thresh)
        if first_frame is not None:
            cv.imshow(self.WIN, first_frame)
        self._win_ready = True

    def _on_mouse(self, event, x, y, flags, _p) -> None:
        if self.st.picking_clamp:
            if event == cv.EVENT_LBUTTONDOWN:
                self.st.clamp_pt    = (x, y)
                self.st.picking_clamp = False
                logging.info("Clamp set at (%d, %d)", x, y)
        else:
            self.roi.on_mouse(event, x, y, flags, _p)

    def _cb_gauss(self,  v: int) -> None: self.st.gauss_sigma = v
    def _cb_thresh(self, v: int) -> None: self.st.thresh_val  = v

    # ---- HUD ---------------------------------------------------------------

    def _hud(self, img: np.ndarray, fps: float) -> None:
        lines = [
            f"MOD: S{self.st.mod_stage}" if self.st.mod_stage else "MOD: -",
            f"S1 BCG: {'ON' if self.st.bcg_state.is_active() else 'OFF'}",
            f"S2 Gauss: {'ON' if self.st.gauss_enabled else 'OFF'}",
            f"S3 Thresh: {self.st.thresh_val}  inv:{'ON' if self.st.invert else 'OFF'}  "
            f"overlay:{'ON' if self.st.overlay_on else 'OFF'}",
            f"S4 ROI: {'ON' if self.roi.have_rect else 'OFF'}",
            f"FPS: {fps:.1f}  topic: {self.topic}  {'[PAUSED]' if self.st.paused else ''}",
            f"Clamp: {'PICKING — click now!' if self.st.picking_clamp else (str(self.st.clamp_pt) if self.st.clamp_pt else 'OFF  (c to set)')}",
            f"Save: {'ON  frames=' + str(self.st.frames_saved) if self.st.saving else 'OFF  (s to start)  → ' + self.save_path}",
        ]
        for i, txt in enumerate(lines):
            cv.putText(img, txt, (10, 18 + i * 18),
                       cv.FONT_HERSHEY_SIMPLEX, 0.46, (0, 0, 220), 1, cv.LINE_AA)

    def _draw_roi(self, img: np.ndarray) -> None:
        if self.roi.have_rect or self.roi.get_roi:
            (x0, y0), (x1, y1) = self.roi.rect
            cv.rectangle(img, (x0, y0), (x1, y1), (255, 80, 0), 2)

    # ---- publish -----------------------------------------------------------

    def _publish(self, line_pts: np.ndarray) -> None:
        """Publish centerline as Float32MultiArray (200 floats).

        Active layout — interleaved, consistent with old skeleton.py / new_skeleton.py:
            [x0, y0, x1, y1, …, x99, y99]

        Commented-out alternative — separated, consistent with centerline_data.txt / notebook:
            [x0, x1, …, x99, y0, y1, …, y99]
        """
        if not _HAS_ROS:
            return

        # Interleaved: [x0, y0, x1, y1, …, x99, y99]
        flat = line_pts.astype(np.float32).flatten()   # shape (100,2) → (200,)

        # # Separated: [x0..x99, y0..y99]  — uncomment to switch back
        # flat = np.concatenate([line_pts[:, 0], line_pts[:, 1]]).astype(np.float32)

        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="xy", size=200, stride=200)],
            data_offset=0,
        )
        msg.data = flat.tolist()
        self._pub.publish(msg)

    # ---- key handling ------------------------------------------------------

    def _handle_key(self, key: int, last: Optional[np.ndarray], fps: float) -> None:
        if key == ord(" "):
            self.st.paused = not self.st.paused
        elif key == ord("c"):
            if self.st.picking_clamp:
                self.st.picking_clamp = False
            elif self.st.clamp_pt is not None:
                self.st.clamp_pt = None
                logging.info("Clamp cleared")
            else:
                self.st.picking_clamp = True
                logging.info("Click on the clamp point in the window")
        elif key == ord("s"):
            self.st.saving = not self.st.saving
            if self.st.saving:
                self.st.frames_saved = 0
                open(self.save_path, 'w').close()
                logging.info("Saving to %s", self.save_path)
            else:
                logging.info("Saving stopped. %d frames saved.", self.st.frames_saved)
        elif key == ord("i"):
            self.st.invert = not self.st.invert
        elif key == ord("0"):
            self.st.overlay_on = not self.st.overlay_on
        elif key == ord("?"):
            print(__doc__)
        elif key in map(ord, "1234"):
            n = key - ord("0")
            self.st.mod_stage = None if self.st.mod_stage == n else n
        elif self.st.mod_stage == 1 and key == ord("a"):
            self.bcg_ctrl.toggle()
        elif self.st.mod_stage == 2 and key == ord("g"):
            self.st.gauss_enabled = not self.st.gauss_enabled
            self._build_window(last)
        elif self.st.mod_stage == 4 and key == ord("m"):
            if self.roi.have_rect or self.roi.get_roi:
                self.roi.clear()
                logging.info("ROI cleared")
            else:
                self.roi.begin()
                logging.info("Draw ROI with left mouse button")

    # ---- main loop ---------------------------------------------------------

    def run(self) -> int:
        # Wait for first frame
        logging.info("Waiting for first frame on %s …", self.topic)
        deadline = perf_counter() + 10.0
        while self._pending_frame is None:
            if _HAS_ROS and rospy.is_shutdown():
                return 1
            if perf_counter() > deadline:
                logging.error("No frame received on %s within 10 s. Check topic name.",
                              self.topic)
                return 1
            sleep(0.05)

        self._build_window(self._pending_frame)
        last_disp: Optional[np.ndarray] = self._pending_frame.copy()

        while True:
            if _HAS_ROS and rospy.is_shutdown():
                break

            self.bcg_ctrl.poll()
            fps_est = 1.0 / max(1e-6, perf_counter() - self._last_t)

            frame = self._pending_frame
            self._pending_frame = None   # consume

            if frame is not None and not self.st.paused and not self.st.picking_clamp:
                out = apply_bcg(frame, self.st.bcg_state)
                if self.st.gauss_enabled and self.st.gauss_sigma > 0:
                    out = apply_gaussian(out, self.st.gauss_sigma)
                roi_rect = self.roi.rect if self.roi.have_rect else None
                disp, line_pts = extract_centerline(
                    out, self.st.thresh_val, roi_rect,
                    self.st.overlay_on, self.st.invert, self.st.clamp_pt
                )

                if line_pts is not None:
                    # ---- Temporal EMA smoothing (anti-jitter) ----------------
                    # Blend new detection with previous output so stationary
                    # fingers stay still.  alpha=1 disables smoothing entirely.
                    pts_f = line_pts.astype(np.float64)
                    if self._ema_pts is None:
                        self._ema_pts = pts_f
                    else:
                        self._ema_pts = self._alpha * pts_f + (1.0 - self._alpha) * self._ema_pts
                    line_pts = np.round(self._ema_pts).astype(np.int32)
                    # ----------------------------------------------------------

                    # Publish over ROS
                    self._publish(line_pts)

                    # Save to file
                    if self.st.saving:
                        row = np.concatenate([line_pts[:, 0], line_pts[:, 1],
                                              [self.st.frames_saved]])
                        with open(self.save_path, 'a') as f:
                            f.write(' '.join(f'{v:.1f}' for v in row) + '\n')
                        self.st.frames_saved += 1

                self._draw_roi(disp)
                self._hud(disp, fps_est)
                cv.imshow(self.WIN, disp)
                last_disp = disp
                self._last_t = perf_counter()

            elif self.st.paused or self.st.picking_clamp:
                if last_disp is not None:
                    overlay = last_disp.copy()
                    if self.st.paused:
                        cv.putText(overlay, "PAUSED  (space to resume)",
                                   (10, self.H - 40), cv.FONT_HERSHEY_SIMPLEX,
                                   0.7, (0, 200, 255), 2, cv.LINE_AA)
                    if self.st.picking_clamp:
                        cv.putText(overlay, "CLICK to set clamp point  (c to cancel)",
                                   (10, self.H - 15), cv.FONT_HERSHEY_SIMPLEX,
                                   0.7, (0, 255, 255), 2, cv.LINE_AA)
                    cv.imshow(self.WIN, overlay)

            key = cv.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key != 255:
                self._handle_key(key, last_disp, fps_est)

            sleep(0.001)

        self._shutdown()
        return 0

    def _shutdown(self) -> None:
        self.bcg_ctrl.hide()
        try:
            cv.destroyAllWindows()
        except cv.error:
            pass
        if self.st.saving:
            logging.info("Saving stopped on exit. %d frames saved.", self.st.frames_saved)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="ROS centerline node — live_centerline pipeline on a ROS camera topic")
    ap.add_argument("--topic",  default="/camera/image_color",
                    help="ROS image topic to subscribe to (default: /camera/image_color)")
    ap.add_argument("--thresh", type=int, default=100,
                    help="Initial threshold value (0–255, default: 100)")
    ap.add_argument("--save",   default="centerline_data.txt",
                    help="Output file for centerline data (default: centerline_data.txt)")
    ap.add_argument("--alpha", type=float, default=0.3,
                    help="EMA smoothing factor 0–1 (0=frozen, 1=no smoothing, default: 0.3)")
    ap.add_argument("--log",    default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=getattr(logging, args.log.upper(), logging.INFO),
                        format="%(levelname)s: %(message)s")

    if not _HAS_ROS:
        logging.error("rospy is not available. Install ROS and source setup.bash.")
        return 1

    rospy.init_node("ros_centerline", anonymous=False)

    # Allow ROS private params to override CLI args
    # Usage: rosrun pkg ros_centerline.py _topic:=/my/camera _thresh:=130
    topic  = rospy.get_param("~topic",  args.topic)
    thresh = int(rospy.get_param("~thresh", args.thresh))
    save   = rospy.get_param("~save",   args.save)
    alpha  = float(rospy.get_param("~alpha", args.alpha))

    node = ROSCenterlineNode(topic=topic, thresh=thresh, save_path=save, alpha=alpha)
    return node.run()


if __name__ == "__main__":
    raise SystemExit(main())
