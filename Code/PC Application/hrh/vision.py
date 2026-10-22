"""Webcam capture + MediaPipe HandLandmarker -> retargeted joint angles.

Runs on a background QThread so the GUI stays responsive. Talks back to
the main thread exclusively via Qt signals (frame_ready / angles_ready /
error / stopped / stats_ready) -- Qt auto-marshals cross-thread signal
emissions onto the receiver's thread, so no manual locking is needed here.

This module owns only the camera, the MediaPipe task and the preview
image. Everything between "MediaPipe found these hands" and "here are 19
joint angles" -- hand selection, handedness, track-loss handling, IK,
wrist, filtering -- lives in tracking.HandTrackingPipeline, which has no Qt
or camera dependency. See that module's docstring for the per-frame
steps and the measurements behind them.
"""

from __future__ import annotations

import time
import urllib.request
from collections import deque
from pathlib import Path
from typing import Optional

# camera.py sets OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS before importing
# cv2, so it is imported first.
from . import camera as cam
import cv2
import mediapipe as mp
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QImage

from .tracking import (
    DEFAULT_BETA,
    DEFAULT_LANDMARK_WINDOW,
    DEFAULT_MINCUTOFF,
    HandTrackingPipeline,
    LatestValueWorker,
    observations_from_result,
)

# Optimisation-based retargeting against the real CAD kinematics (on synthetic
# trajectories ~6.7x more accurate than measuring each angle directly from the
# landmarks: 4.5 deg mean joint error vs 30.3 deg, at comparable jitter).
from .ik_retarget import HandRetargeter
from .wrist_retarget import WristRetargeter

STATS_INTERVAL_S = 0.5  # how often the worker publishes diagnostics counters

# How many hands MediaPipe looks for. 1, deliberately: MediaPipe only skips
# its (expensive) palm detector while it is tracking as many hands as it was
# asked for, so with 2 and one hand in view it runs the detector on every
# frame. Measured with an offline benchmark replaying a recorded clip live
# (60 fps feed, one hand, 2-core machine, IK on the worker): with 1, angles
# for 296 of 300 frames at 27.4 ms mean latency; with 2, only 235 of 300
# (MediaPipe drops the rest) at 45.4 ms. The pipeline still copes with 2 (it
# follows the tracked hand by continuity and ignores the other), so set this
# to 2 if a second hand in view is a bigger problem for you than latency.
NUM_HANDS = 1
# MediaPipe's own defaults. Named here so they are visible and tunable:
# raising presence/tracking makes MediaPipe fall back to the (slower) palm
# detector more often; lowering detection admits more false hands.
MIN_DETECTION_CONFIDENCE = 0.5
MIN_PRESENCE_CONFIDENCE = 0.5
MIN_TRACKING_CONFIDENCE = 0.5

# Consecutive failed cap.read() calls (at ~5 ms apart) before giving up: a
# camera that has been unplugged or grabbed by another app otherwise leaves
# the loop spinning silently with a frozen preview.
MAX_CONSECUTIVE_READ_FAILURES = 200

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
)
MODEL_DIR = Path(__file__).resolve().parent.parent / "models"
MODEL_PATH = MODEL_DIR / "hand_landmarker.task"

# Standard 21-point MediaPipe hand skeleton edges, for the preview overlay
# (the "solutions" convenience API that used to export this constant is
# gone in mediapipe>=1.0, so it's inlined here).
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),  # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),  # index
    (5, 9), (9, 10), (10, 11), (11, 12),  # middle
    (9, 13), (13, 14), (14, 15), (15, 16),  # ring
    (13, 17), (17, 18), (18, 19), (19, 20),  # pinky
    (0, 17),  # wrist - pinky mcp (palm edge)
]


def ensure_model_downloaded() -> Path:
    """Download the hand landmark model on first use; reuse it after that."""
    if MODEL_PATH.exists():
        return MODEL_PATH
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    tmp_path = MODEL_PATH.with_suffix(".task.part")
    urllib.request.urlretrieve(MODEL_URL, tmp_path)
    tmp_path.rename(MODEL_PATH)
    return MODEL_PATH


#: The preview is shrunk to at most this width in the worker thread, before
#: it is turned into a QImage: it is shown in a ~370 px panel, and the GUI
#: thread (which also runs the 30 Hz send timer and the 3D view) then only
#: rescales a small image. Minor, but free: measured ~1.4 -> 0.5 ms of GUI-
#: thread time per frame, and it also drops a second full-frame colour
#: conversion from the capture loop.
PREVIEW_MAX_WIDTH = 480

#: If the running camera rate stays below this for LOW_FPS_WARN_S, say so
#: (once) -- a room that gets darker slows an auto-exposing webcam down.
LOW_FPS_WARN_S = 3.0


class VisionWorker(QObject):
    """Owns the camera and the MediaPipe landmarker. Move to a QThread and
    connect thread.started to run(); call stop() from the GUI thread to
    end the loop."""

    frame_ready = Signal(QImage)
    angles_ready = Signal(list)
    error = Signal(str)
    stopped = Signal()
    #: periodic {metric: value} for the diagnostics panel. Emitted from the
    #: worker thread at STATS_INTERVAL_S so the GUI never has to reach across
    #: the thread boundary to read counters.
    stats_ready = Signal(dict)
    #: one-line advice about the camera (e.g. "delivers only 10 fps ...")
    #: for the status bar. Not an error: tracking keeps running.
    camera_notice = Signal(str)
    #: (text, state) whenever the tracking status line changes -- e.g.
    #: ("RIGHT hand", "tracking"). The GUI shows it as a proper label above
    #: the preview; it is no longer burned into the image, where it shrank
    #: with the preview and became unreadable. Emitted on change only.
    status_changed = Signal(str, str)

    def __init__(
        self,
        camera_index: int = 0,
        mincutoff: float = DEFAULT_MINCUTOFF,
        beta: float = DEFAULT_BETA,
        landmark_smoothing_window: int = DEFAULT_LANDMARK_WINDOW,
        filtering_enabled: bool = True,
        capture_size: tuple = (1280, 720),
        capture_fps: int = 60,
        hand_preference: str = "auto",
        wrist_enabled: bool = False,
        short_exposure: bool = False,
        resolution: str = cam.DEFAULT_RESOLUTION,
        pinch_snap: bool = True,
    ):
        # Filter defaults live in tracking.py (One Euro 0.5 / 0.1, no landmark
        # pre-average), where the measurements behind them are recorded: on
        # real video they give less still-hand jitter than the previous
        # 2-frame mean + One Euro 1.5 / 0.1 AND 15 ms less lag. Lower
        # mincutoff if it looks too jittery when you hold still; raise it
        # (or beta) if it feels laggy.
        super().__init__()
        self._camera_index = camera_index
        # capture_size / capture_fps are accepted for compatibility but no
        # longer used: the camera mode is negotiated by measurement from
        # camera.MODES (see camera.py).
        self._mincutoff, self._beta = mincutoff, beta
        self._landmark_window = landmark_smoothing_window
        self._running = False
        self._landmarker = None
        # Settings are kept here as well as forwarded, because the pipeline
        # is only built once run() starts on the worker thread (loading the
        # URDF + meshes takes ~100 ms and must not block the GUI thread).
        self._filtering_enabled = filtering_enabled
        self._hand_preference = hand_preference
        self._wrist_enabled = wrist_enabled
        self._pinch_snap = pinch_snap
        self._pipeline: Optional[HandTrackingPipeline] = None
        self._last_timestamp_ms = -1  # see run(): stamps must strictly increase
        self.camera_info = None
        self.camera_backend = None
        self.camera_mode = None
        self._short_exposure = short_exposure
        self._resolution = resolution
        self._req_resolution: Optional[str] = None  # applied by the capture loop
        self._req_exposure: Optional[bool] = None   # applied by the capture loop
        self._req_settings = False
        self._read_ms = deque(maxlen=60)    # waiting in cap.read() per frame
        self._work_ms = deque(maxlen=60)    # everything else in the loop
        self._low_fps_since: Optional[float] = None
        self._low_fps_warned = False

        # Overlay state, written by the MediaPipe callback and read by the
        # capture loop. Each is replaced wholesale (a single reference
        # assignment), never mutated, so a reader always sees a consistent
        # snapshot without locking.
        self._overlay = None  # (list of image landmark arrays, tracked idx, status text)
        self._last_status: Optional[tuple] = None  # last (text, state) sent via status_changed

        # Diagnostics counters. Captured on the worker thread and published
        # through stats_ready; see _publish_stats.
        self._stat_frames = 0        # frames pulled off the camera
        self._stat_detections = 0    # frames MediaPipe found a hand in
        self._stat_misses = 0        # ...and frames it did not
        self._stat_frame_errors = 0
        self._stat_left_hands = 0
        self._stat_extra_hands = 0   # frames with a second hand in view
        self._solve_ms = deque(maxlen=90)
        self._latency_ms = deque(maxlen=90)   # frame capture -> angles emitted
        self._start_time = 0.0
        self._worker: Optional[LatestValueWorker] = None
        self._cam_dt = deque(maxlen=90)
        self._last_frame_t = 0.0
        self._last_stats_emit = 0.0
        self._last_state = "searching"
        self._last_hand = None

    # -- settings (called from the GUI thread) --------------------------------

    def stop(self) -> None:
        """Thread-safe-enough: a plain bool flag checked once per loop
        iteration. Called from the GUI thread."""
        self._running = False

    def set_filtering_enabled(self, enabled: bool) -> None:
        """Safe from the GUI thread: the pipeline only records the request
        and applies it at the start of its next frame. Re-enabling starts the
        smoother and One Euro filter fresh, so they don't blend in whatever
        they saw before filtering was switched off."""
        self._filtering_enabled = enabled
        if self._pipeline is not None:
            self._pipeline.set_filtering_enabled(enabled)

    def set_hand_preference(self, pref: str) -> None:
        """"auto" | "right" | "left" -- which hand to follow (see tracking.py)."""
        self._hand_preference = pref
        if self._pipeline is not None:
            self._pipeline.set_hand_preference(pref)

    def set_wrist_enabled(self, enabled: bool) -> None:
        """Drive W_1..W_3 from the palm orientation. Enabling captures a
        neutral pose (from ~0.3 s of a still hand) if none is set yet;
        disabling ramps the wrist home at its slew limit."""
        self._wrist_enabled = enabled
        if self._pipeline is not None:
            self._pipeline.set_wrist_enabled(enabled)

    def set_pinch_snap(self, enabled: bool) -> None:
        """Fingertip snapping on/off (ik_retarget.HandRetargeter.pinch_snap).
        Applied by the pipeline at the start of its next frame."""
        self._pinch_snap = enabled
        if self._pipeline is not None:
            self._pipeline.set_pinch_snap(enabled)

    def set_short_exposure(self, enabled: bool) -> None:
        """Short fixed exposure (faster in dim light, darker picture) or back
        to auto-exposure. Applied by the capture loop on its next frame."""
        self._short_exposure = enabled
        self._req_exposure = bool(enabled)

    def set_resolution(self, key: str) -> None:
        """"480p" | "720p" | "auto" (camera.RESOLUTIONS). While tracking, the
        capture loop reopens the camera in the new mode on its next frame."""
        if key in cam.RESOLUTIONS and key != self._resolution:
            self._resolution = key
            self._req_resolution = key

    def request_camera_settings(self) -> None:
        """Open the camera driver's own settings page (DirectShow), where
        low-light compensation, exposure and anti-flicker live."""
        self._req_settings = True

    def request_wrist_neutral(self) -> None:
        """Re-capture the wrist neutral (from ~0.3 s of a still hand)."""
        if self._pipeline is not None:
            self._pipeline.request_wrist_neutral()

    # -- setup --------------------------------------------------------------

    def _build_pipeline(self) -> HandTrackingPipeline:
        """Loads the URDF retargeters (~100 ms) on the worker thread. Any
        failure here (URDF export missing or unreadable) is reported by run()
        and tracking stops cleanly."""
        retargeter = HandRetargeter()
        wrist = WristRetargeter(retargeter.model, retargeter._robot_R)
        pipeline = HandTrackingPipeline(
            retargeter, wrist,
            mincutoff=self._mincutoff, beta=self._beta,
            landmark_window=self._landmark_window,
            filtering=self._filtering_enabled,
            hand_preference=self._hand_preference,
        )
        pipeline.set_wrist_enabled(self._wrist_enabled)
        pipeline.set_pinch_snap(self._pinch_snap)
        return pipeline

    def _open_camera(self):
        """Open the camera in the fastest mode that works -- see camera.py.
        Returns the capture or None."""
        try:
            sel = cam.negotiate(
                self._camera_index, self._short_exposure,
                keep_going=lambda: self._running,
                progress=lambda text: self.status_changed.emit(text, "idle"),
                resolution=self._resolution,
            )
        except Exception as exc:  # noqa: BLE001
            self.error.emit(f"Could not open webcam index {self._camera_index}: {exc}")
            return None
        if sel is None:
            if self._running:
                self.error.emit(f"Could not open webcam index {self._camera_index}")
            return None
        w, h, _reported, fourcc = sel.granted
        self.camera_info = (w, h, sel.measured_fps)
        self.camera_backend = sel.backend
        self.camera_mode = f"{w}x{h} {fourcc}"
        advice = sel.diagnosis()
        if advice:
            self._low_fps_warned = True
            self.camera_notice.emit(advice)
        return sel.cap

    # -- main loop ------------------------------------------------------------

    def run(self) -> None:
        self._running = True
        try:
            model_path = ensure_model_downloaded()
        except Exception as exc:
            self.error.emit(f"Could not download hand landmark model: {exc}")
            self.stopped.emit()
            return

        try:
            self._pipeline = self._build_pipeline()
        except Exception as exc:  # noqa: BLE001
            self.error.emit(f"Could not build the tracking pipeline: {exc}")
            self.stopped.emit()
            return

        try:
            options = mp.tasks.vision.HandLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
                running_mode=mp.tasks.vision.RunningMode.LIVE_STREAM,
                num_hands=NUM_HANDS,
                min_hand_detection_confidence=MIN_DETECTION_CONFIDENCE,
                min_hand_presence_confidence=MIN_PRESENCE_CONFIDENCE,
                min_tracking_confidence=MIN_TRACKING_CONFIDENCE,
                result_callback=self._on_result,
            )
            self._landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)
        except Exception as exc:
            self.error.emit(f"Could not start MediaPipe HandLandmarker: {exc}")
            self.stopped.emit()
            return

        cap = self._open_camera()
        if cap is None:
            self._landmarker.close()
            self.stopped.emit()
            return

        # The IK runs here, not in MediaPipe's callback -- see
        # tracking.LatestValueWorker for the measured difference.
        self._worker = LatestValueWorker(self._pipeline, self._on_output, self._on_worker_error)
        self._worker.start()
        start_time = time.monotonic()
        self._start_time = start_time
        read_failures = 0
        try:
            while self._running:
                # Everything in this iteration is wrapped, not just
                # detect_async: a single bad frame (a NaN/out-of-range
                # landmark coordinate, a transient camera read glitch,
                # anything) must never be allowed to escape this loop --
                # run() is invoked as a Qt slot (via thread.started), so an
                # escaping exception could take the whole application down
                # instead of just this one frame.
                try:
                    if self._req_resolution is not None:
                        self._req_resolution = None
                        cap.release()
                        self._low_fps_warned = False   # re-diagnose the new mode
                        cap = self._open_camera()
                        if cap is None:
                            break   # _open_camera has reported why
                        self._cam_dt.clear()
                        self._last_frame_t = 0.0
                        self._low_fps_since = None
                    self._apply_camera_requests(cap)
                    t_read = time.perf_counter()
                    ok, frame_bgr = cap.read()
                    t_got = time.perf_counter()
                    if not ok or frame_bgr is None:
                        read_failures += 1
                        if read_failures >= MAX_CONSECUTIVE_READ_FAILURES:
                            self.error.emit("camera stopped delivering frames (unplugged, or in use by another app?)")
                            break
                        time.sleep(0.005)
                        continue
                    read_failures = 0

                    # MediaPipe gets the TRUE (unmirrored) frame; only the
                    # preview is mirrored, with the overlay flipped to match.
                    # Mirroring is a determinant -1 transform: feeding a
                    # mirrored frame would hand the retargeter the geometry
                    # of the opposite hand. (The Tasks API labels unmirrored
                    # frames correctly -- verified, see tracking.py.)
                    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                    # LIVE_STREAM requires *strictly* increasing millisecond
                    # timestamps; truncating a monotonic clock to ms is not
                    # injective at 60 fps, so force at least one tick.
                    timestamp_ms = int((time.monotonic() - start_time) * 1000)
                    if timestamp_ms <= self._last_timestamp_ms:
                        timestamp_ms = self._last_timestamp_ms + 1
                    self._last_timestamp_ms = timestamp_ms
                    self._landmarker.detect_async(mp_image, timestamp_ms)

                    now = time.monotonic()
                    if self._last_frame_t:
                        self._cam_dt.append(now - self._last_frame_t)
                    self._last_frame_t = now
                    self._stat_frames += 1

                    self.frame_ready.emit(self._preview(rgb))
                    self._read_ms.append((t_got - t_read) * 1000.0)
                    self._work_ms.append((time.perf_counter() - t_got) * 1000.0)
                    self._publish_stats(now)
                except Exception as exc:  # noqa: BLE001 - deliberately broad, see above
                    self._stat_frame_errors += 1
                    self.error.emit(f"hand-tracking frame error (skipped): {exc}")
                    continue
        finally:
            if cap is not None:
                cap.release()
            # Close MediaPipe first (no more callbacks), then the worker.
            self._landmarker.close()
            self._worker.stop()

        self.stopped.emit()

    def _publish_stats(self, now: float) -> None:
        """Emit a diagnostics snapshot, at most every STATS_INTERVAL_S.

        Everything here is measured on the worker thread and handed over as
        a plain dict, so the GUI never touches this object's counters across
        the thread boundary. Rates are computed from the sampled intervals
        rather than from wall-clock deltas of the totals, which keeps them
        stable even when the GUI thread is busy.
        """
        if now - self._last_stats_emit < STATS_INTERVAL_S:
            return
        self._last_stats_emit = now
        cam_dt = list(self._cam_dt)
        solve = list(self._solve_ms)
        total = self._stat_detections + self._stat_misses
        stats = {
            "camera_fps": (len(cam_dt) / sum(cam_dt)) if cam_dt and sum(cam_dt) > 0 else 0.0,
            "frames": self._stat_frames,
            "detections": self._stat_detections,
            "misses": self._stat_misses,
            "detect_rate": (self._stat_detections / total) if total else 0.0,
            "frame_errors": self._stat_frame_errors,
            "left_hands": self._stat_left_hands,
            "extra_hands": self._stat_extra_hands,
            "solve_ms_avg": (sum(solve) / len(solve)) if solve else 0.0,
            "solve_ms_max": max(solve) if solve else 0.0,
            "latency_ms": self._latency_summary(),
            "stale_skipped": self._worker.dropped if self._worker is not None else 0,
            "capture": self.camera_info,
            "camera_backend": self.camera_backend,
            "camera_mode": self.camera_mode,
            "camera_timing": self._timing_summary(),
            "track_state": self._last_state,
            "tracked_hand": self._last_hand,
        }
        p = self._pipeline
        if p is not None:
            stats["acquisitions"] = p.acquisitions
            stats["flicker_suppressed"] = p.flicker_suppressed
            if p.retargeter is not None:
                stats["solver_residual"] = float(p.last_residual or 0.0)
                stats["solver"] = "URDF optimiser"
            else:
                stats["solver"] = "legacy geometric"
            stats["wrist"] = p.wrist_status
        self.stats_ready.emit(stats)
        self._watch_camera_rate(stats["camera_fps"], now)

    def _timing_summary(self):
        """(ms waiting for the camera, ms of work) per frame, averaged. If
        the wait dominates, the camera is what limits the rate; if the work
        does, this PC is."""
        if not self._read_ms:
            return None
        return (sum(self._read_ms) / len(self._read_ms), sum(self._work_ms) / len(self._work_ms))

    def _watch_camera_rate(self, fps: float, now: float) -> None:
        if fps <= 0 or fps >= cam.GOOD_FPS:
            self._low_fps_since = None
            return
        if self._low_fps_since is None:
            self._low_fps_since = now
        elif not self._low_fps_warned and now - self._low_fps_since > LOW_FPS_WARN_S:
            self._low_fps_warned = True
            wait, work = self._timing_summary() or (0.0, 0.0)
            if work > wait:
                self.camera_notice.emit(
                    f"Camera loop runs at {fps:.0f} fps and this PC is the limit "
                    f"({work:.0f} ms of work per frame) -- close other heavy programs.")
            else:
                self.camera_notice.emit(
                    f"Camera slowed to {fps:.0f} fps -- usually auto-exposure in dim light. "
                    "More light on your hand, Short exposure, or Camera Settings "
                    "(turn off low-light compensation).")

    def _apply_camera_requests(self, cap) -> None:
        if self._req_exposure is not None:
            enabled, self._req_exposure = self._req_exposure, None
            cam.set_short_exposure(cap, enabled)
            self._low_fps_warned = False
        if self._req_settings:
            self._req_settings = False
            if not cam.open_settings_dialog(cap):
                self.camera_notice.emit("This camera backend has no settings page "
                                        f"({self.camera_backend}); use the camera maker's app.")

    def _latency_summary(self):
        lat = sorted(self._latency_ms)
        if not lat:
            return None
        return (sum(lat) / len(lat), lat[min(len(lat) - 1, int(0.95 * len(lat)))])

    # -- MediaPipe callback and the retarget worker --------------------------

    def _on_result(self, result, output_image, timestamp_ms: int) -> None:
        # Invoked by MediaPipe's C++ task graph, from a thread it manages. It
        # only converts the result and hands it to the retarget worker: the
        # ~10 ms solve used to run right here, stalling MediaPipe's graph
        # (see tracking.LatestValueWorker). An uncaught Python exception in a
        # callback invoked from native code can crash the whole process, so
        # this is wrapped defensively.
        try:
            hands = observations_from_result(result)
            if hands:
                self._stat_detections += 1
                if len(hands) > 1:
                    self._stat_extra_hands += 1
            else:
                self._stat_misses += 1
            worker = self._worker
            if worker is not None:
                worker.submit(hands, timestamp_ms / 1000.0, (timestamp_ms, [h.image for h in hands]))
        except Exception as exc:  # noqa: BLE001 - deliberately broad, see above
            self.error.emit(f"hand-tracking callback error: {exc}")

    def _on_output(self, out, n_hands: int, solve_ms: float, tag) -> None:
        """Runs on the retarget worker thread after each processed frame."""
        timestamp_ms, images = tag
        if out.angles is not None:
            self._solve_ms.append(solve_ms)
            # Frame timestamps are monotonic-clock ms since start_time, taken
            # when cap.read() returned: this is MediaPipe + IK + filter time.
            # Camera exposure, driver and USB latency come on top and are not
            # visible from here.
            self._latency_ms.append((time.monotonic() - self._start_time) * 1000.0 - timestamp_ms)
            if out.mirrored:
                self._stat_left_hands += 1
            self.angles_ready.emit(out.angles)
        self._last_state = out.state
        self._last_hand = out.hand if out.angles is not None else None
        images = [im for im in images if im is not None]
        status = self._status_text(out, n_hands)
        self._overlay = (images, out.index, status)
        key = (status, out.state or "none")
        if key != self._last_status:
            self._last_status = key
            self.status_changed.emit(*key)

    def _on_worker_error(self, exc: Exception) -> None:
        self._stat_frame_errors += 1
        self.error.emit(f"hand-tracking retarget error: {exc}")

    def _status_text(self, out, n_hands: int) -> str:
        p = self._pipeline
        if out.state == "tracking":
            text = f"{out.hand.upper()} hand" + (" (mirrored)" if out.mirrored else "")
            if n_hands > 1:
                text += " - other hand ignored"
            if p is not None and p.wrist_status != "off":
                text += f" | wrist {p.wrist_status}"
            return text
        if out.state == "confirming":
            return f"Confirming {p.hand_preference if p is not None else ''} hand..."
        if out.state == "wrong_hand":
            pref = p.hand_preference if p is not None else self._hand_preference
            return f"Only the other hand is visible (following: {pref})"
        return "No hand"

    # -- preview frame rendering --------------------------------------------

    def _draw_overlay(self, frame_bgr) -> None:  # any 3-channel image
        overlay = self._overlay
        h, w = frame_bgr.shape[:2]
        if overlay is None:
            status, images, tracked = "No hand", [], None
        else:
            images, tracked, status = overlay
        for i, lm in enumerate(images):
            try:
                # x is mirrored to match the mirrored preview; detection ran
                # on the unmirrored frame (see run()).
                points = [(int((1.0 - p[0]) * w), int(p[1] * h)) for p in lm]
            except (ValueError, OverflowError, TypeError):
                continue  # NaN/inf landmark this frame -- skip this hand
            color = (0, 200, 0) if i == tracked else (140, 140, 140)
            for a, b in HAND_CONNECTIONS:
                cv2.line(frame_bgr, points[a], points[b], color, 2)
            for x, y in points:
                cv2.circle(frame_bgr, (x, y), 3, color, -1)
        # No status text is drawn onto the frame any more: it was rendered at
        # camera resolution and then scaled down with the whole preview, so
        # in the narrow tab panel it came out a few pixels tall. The same
        # string now goes to the GUI via status_changed and is shown as a
        # full-size label above the preview (see gui.CameraPanel).

    def _preview(self, rgb) -> QImage:
        """Mirrored, shrunk preview with the skeleton overlay, straight from
        the RGB frame MediaPipe got (no second colour conversion)."""
        h, w = rgb.shape[:2]
        if w > PREVIEW_MAX_WIDTH:
            small = cv2.resize(rgb, (PREVIEW_MAX_WIDTH, round(h * PREVIEW_MAX_WIDTH / w)),
                               interpolation=cv2.INTER_LINEAR)
        else:
            small = rgb.copy()
        small = cv2.flip(small, 1)
        self._draw_overlay(small)
        ph, pw, ch = small.shape
        image = QImage(small.data, pw, ph, ch * pw, QImage.Format_RGB888)
        return image.copy()  # detach from the numpy buffer before it's reused

    @staticmethod
    def list_camera_indices(max_check: int = 4) -> list[int]:
        """Best-effort probe of available camera indices (0..max_check-1),
        using the same preferred backend as tracking (DirectShow on
        Windows, which opens quickly). Not called by the GUI, which offers a
        fixed 0-3 list to avoid opening every camera at start-up."""
        name, api = cam.backends()[0]
        found = []
        for i in range(max_check):
            cap = cv2.VideoCapture(i, api)
            if cap.isOpened():
                found.append(i)
            cap.release()
        return found
