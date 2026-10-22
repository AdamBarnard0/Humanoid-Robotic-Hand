"""Webcam opening, mode negotiation and diagnosis (Qt-free).

Why this exists: at 10 fps the camera alone costs ~100-150 ms of latency (a
new frame only every 100 ms, plus up to 100 ms of exposure inside it), which
dwarfs everything downstream -- MediaPipe + solve is ~27 ms, the One Euro
filter ~12 ms (measured at both 30 and 10 fps), the 30 Hz
send ~17 ms on average. So the single biggest latency win is getting the
camera itself to 30 fps. A webcam stuck at ~10 fps nearly always has one of
three causes, and this module deals with each:

  1. Uncompressed video. 1280x720 in YUY2 needs ~18 MB/s at 10 fps; over
     USB 2.0 most webcams cap it at 7.5-10 fps. MJPG (compressed) runs the
     same resolution at 30 fps. Drivers silently fall back to YUY2 when the
     MJPG request is not honoured -- so the reported format and the
     MEASURED frame rate are checked, and smaller / other formats are tried.
  2. Low light. With auto-exposure on, the camera lengthens each exposure
     in dim light and its frame rate drops with it (a 1/10 s exposure cannot
     run faster than 10 fps). Fixes: more light on the hand, the driver's
     "low light compensation" off (Logitech's RightLight and similar), or a
     short fixed exposure (`short_exposure`, trades brightness for speed).
     `open_settings_dialog` pops the driver's own property page (DirectShow).
  3. The wrong backend. Windows Media Foundation (OpenCV's default) is slow
     to open and often ignores format/rate requests; DirectShow is tried
     first.

Negotiation: for each backend, try the modes of the chosen RESOLUTIONS entry
in order, MEASURE the frame rate each one actually delivers (the rate a
driver reports is often fiction), and keep the first that reaches GOOD_FPS at
the chosen size; otherwise the fastest at that size, and only if the camera
cannot do that size at all, the fastest of the fallbacks. The result is
cached per camera and resolution for the rest of the session, so restarting
tracking or switching back is quick.

    python -m hrh.camera [--index 0]

runs the full matrix (every backend x mode x auto/short exposure) and prints
the measured frame rate of each, which tells you exactly which of the three
causes you have.
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

# Must be set before cv2 is imported (see opencv/opencv#17687): with hardware
# transforms enabled, Media Foundation can take tens of seconds to open.
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

import cv2  # noqa: E402

#: a mode delivering at least this is taken without trying the rest
GOOD_FPS = 24.0

# On Windows (DirectShow and Media Foundation alike) the exposure property is
# IAMCameraControl's log2(seconds): -6 = 1/64 s (15.6 ms), short enough that
# exposure can never be what limits 30 fps. CAP_PROP_AUTO_EXPOSURE there is
# 1 = auto, 0 = manual (checked against OpenCV's cap_dshow.cpp / cap_msmf.cpp);
# Linux V4L2 uses 0.75 / 0.25 instead.
SHORT_EXPOSURE_LOG2_S = -6.0
if sys.platform.startswith("win"):
    _AUTO_EXPOSURE_ON, _AUTO_EXPOSURE_OFF = 1.0, 0.0
else:
    _AUTO_EXPOSURE_ON, _AUTO_EXPOSURE_OFF = 0.75, 0.25


@dataclass(frozen=True)
class CameraMode:
    fourcc: Optional[str]   # "MJPG", or None for the driver's default format
    width: int
    height: int
    fps: int

    @property
    def label(self) -> str:
        return f"{self.fourcc or 'default'} {self.width}x{self.height}"


_MJPG_720 = CameraMode("MJPG", 1280, 720, 60)
_MJPG_480 = CameraMode("MJPG", 640, 480, 60)
_RAW_480 = CameraMode(None, 640, 480, 30)

#: Every mode, for the probe.
MODES: Tuple[CameraMode, ...] = (_MJPG_720, _MJPG_480, _RAW_480)

#: The Resolution choices offered in the GUI: key -> (label, modes to try in
#: order). Each list ends in fallbacks, so a camera that cannot do the chosen
#: size still opens.
#:
#: "480p" is the default. At 640x480 almost every webcam manages 30 fps --
#: even uncompressed over USB 2.0, which 720p cannot -- and it costs little
#: tracking accuracy: MediaPipe's palm detector works on a 192x192 downscale
#: and its landmark model on a 224x224 crop of the hand, so a hand at normal
#: distance is already well covered. "720p" gives the landmark model more
#: pixels when the hand is small in the picture (far from the camera), if
#: the camera can deliver it fast enough. "auto" takes 720p when it reaches
#: GOOD_FPS and 480p otherwise.
RESOLUTIONS = {
    "480p": ("640x480 (fast)", (_MJPG_480, _RAW_480, _MJPG_720)),
    "720p": ("1280x720 (detail)", (_MJPG_720, _MJPG_480, _RAW_480)),
    "auto": ("Auto", (_MJPG_720, _MJPG_480, _RAW_480)),
}
DEFAULT_RESOLUTION = "480p"


def backends() -> List[Tuple[str, int]]:
    """(name, cv2 API id) in the order to try them."""
    if sys.platform.startswith("win"):
        return [("DirectShow", cv2.CAP_DSHOW), ("MediaFoundation", cv2.CAP_MSMF)]
    return [("default", cv2.CAP_ANY)]


def _fourcc_str(value: float) -> str:
    v = int(value)
    s = "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))
    return s if s.isprintable() and s.strip() else "?"


def set_short_exposure(cap, enabled: bool) -> None:
    """Short fixed exposure (faster, darker) or back to auto-exposure. Every
    driver interprets these properties a little differently; failures are
    ignored -- the measured frame rate is what counts."""
    try:
        if enabled:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, _AUTO_EXPOSURE_OFF)
            cap.set(cv2.CAP_PROP_EXPOSURE, SHORT_EXPOSURE_LOG2_S)
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, _AUTO_EXPOSURE_ON)
    except Exception:  # noqa: BLE001
        pass


def open_settings_dialog(cap) -> bool:
    """Pop the driver's own camera settings page (DirectShow only). That is
    where "low light compensation", exposure and anti-flicker live."""
    try:
        return bool(cap.set(cv2.CAP_PROP_SETTINGS, 1))
    except Exception:  # noqa: BLE001
        return False


def open_capture(index: int, api: int, mode: CameraMode, short_exposure: bool = False):
    """Open and configure one camera mode. Returns the capture or None.

    The format has to be requested BEFORE the resolution, or many drivers
    quietly ignore the resolution request. Everything is a request, not a
    guarantee -- see `granted` and `measure_fps`."""
    cap = cv2.VideoCapture(index, api)
    if not cap.isOpened():
        cap.release()
        return None
    try:
        if mode.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*mode.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, mode.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
        cap.set(cv2.CAP_PROP_FPS, mode.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # newest frame, not a backlog
        if short_exposure:
            set_short_exposure(cap, True)
    except Exception:  # noqa: BLE001 - a backend that rejects a property
        pass
    return cap


def granted(cap) -> Tuple[int, int, float, str]:
    """(width, height, reported fps, fourcc) as the driver reports them."""
    return (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            float(cap.get(cv2.CAP_PROP_FPS)), _fourcc_str(cap.get(cv2.CAP_PROP_FOURCC)))


def measure_fps(cap, max_seconds: float = 1.0, frames: int = 12, warmup: int = 4,
                keep_going: Callable[[], bool] = lambda: True) -> float:
    """Frames per second actually delivered, 0 if none. The first frames
    after opening are often slow (auto-exposure settling), so a few are
    discarded first."""
    deadline = time.monotonic() + 3.0
    got = 0
    while got < warmup and time.monotonic() < deadline and keep_going():
        ok, f = cap.read()
        if ok and f is not None and f.size:
            got += 1
    if got == 0:
        return 0.0
    t0 = time.monotonic()
    n = 0
    while n < frames and time.monotonic() - t0 < max_seconds and keep_going():
        ok, f = cap.read()
        if ok and f is not None and f.size:
            n += 1
    dt = time.monotonic() - t0
    return n / dt if dt > 0 and n else 0.0


@dataclass
class CameraSelection:
    cap: object
    backend: str
    mode: CameraMode
    measured_fps: float
    granted: Tuple[int, int, float, str]
    tried: List[str] = field(default_factory=list)
    #: (width, height, measured fps) of a faster mode seen while probing, if
    #: the chosen resolution kept this one slow
    faster: Optional[Tuple[int, int, float]] = None

    @property
    def limited(self) -> bool:
        return self.measured_fps < GOOD_FPS

    def diagnosis(self) -> Optional[str]:
        """Plain-language reason for a slow camera, or None if it is fine."""
        if not self.limited:
            return None
        w, h, reported, fourcc = self.granted
        msg = f"Camera delivers only {self.measured_fps:.0f} fps"
        if self.faster is not None:
            fw, fh, ffps = self.faster
            return (msg + f" at {w}x{h} ({fourcc}), but {ffps:.0f} fps at {fw}x{fh}: the "
                    "larger size doesn't fit its USB bandwidth. Choose the 640x480 "
                    "Resolution for lower latency.")
        if reported >= GOOD_FPS:
            return (msg + f" although the driver claims {reported:.0f}: it is almost certainly "
                    "lengthening its exposure in low light. Put more light on your hand, turn off "
                    "'low light compensation' in Camera Settings, or enable Short exposure.")
        return (msg + f" (driver grants {w}x{h} {fourcc} at {reported:.0f} fps). Try another "
                "USB port (USB 3 if available), or a camera that supports MJPG.")


_cache: dict = {}   # (index, short_exposure, resolution) -> (backend name, api, mode)


def negotiate(index: int, short_exposure: bool = False,
              keep_going: Callable[[], bool] = lambda: True,
              progress: Callable[[str], None] = lambda s: None,
              resolution: str = DEFAULT_RESOLUTION) -> Optional[CameraSelection]:
    """Open camera `index` in the fastest mode that works (see module doc)."""
    modes = RESOLUTIONS.get(resolution, RESOLUTIONS[DEFAULT_RESOLUTION])[1]
    # With a fixed size chosen ("480p" / "720p") that size wins whenever the
    # driver actually grants it, even if slow -- it is what was asked for,
    # and the diagnosis says why it is slow; the other size is only a
    # fallback for a camera that cannot do it at all. "auto" takes the first
    # mode that reaches GOOD_FPS, else the fastest.
    want = (modes[0].width, modes[0].height) if resolution != "auto" else None
    key = (index, bool(short_exposure), resolution)
    if key in _cache:
        name, api, mode = _cache[key]
        cap = open_capture(index, api, mode, short_exposure)
        if cap is not None:
            fps = measure_fps(cap, max_seconds=0.5, frames=6, warmup=2, keep_going=keep_going)
            if fps > 0:
                return CameraSelection(cap, name, mode, fps, granted(cap), ["cached"])
            cap.release()
        del _cache[key]

    tried: List[str] = []
    slow: List[float] = []
    best = None        # (fps, name, api, mode) over everything
    best_size = None   # ...over modes that got the wanted size
    exposure_limited = False
    for name, api in backends():
        if exposure_limited:
            break
        for mode in modes:
            if not keep_going():
                return None
            progress(f"Testing camera: {mode.label} via {name}...")
            cap = open_capture(index, api, mode, short_exposure)
            if cap is None:
                tried.append(f"{name}: not opened")
                break   # this backend can't open the camera at all
            fps = measure_fps(cap, keep_going=keep_going)
            g = granted(cap)
            tried.append(f"{name} {mode.label}: {fps:.0f} fps (got {g[0]}x{g[1]} {g[3]})")
            size_ok = want is None or (g[0], g[1]) == want
            if fps >= GOOD_FPS and size_ok:
                _cache[key] = (name, api, mode)
                return CameraSelection(cap, name, mode, fps, g, tried)
            cap.release()
            if fps > 0:
                if best is None or fps > best[0] * 1.1:
                    best = (fps, name, api, mode)
                if want is not None and size_ok and (best_size is None or fps > best_size[0] * 1.1):
                    best_size = (fps, name, api, mode)
            slow.append(fps)
            if (len(slow) >= 2 and min(slow) > 0 and max(slow) < 1.15 * min(slow)
                    and g[2] >= GOOD_FPS):
                # Different modes, same slow rate, driver claiming full rate:
                # that is exposure, not bandwidth or backend -- stop probing
                # (another backend won't help and each try costs ~1.5 s).
                tried.append("same rate in every mode: exposure-limited, stopped probing")
                exposure_limited = True
                break
    pick = best_size or best
    if pick is None:
        return None
    fps, name, api, mode = pick
    faster = None
    if best is not None and best[0] >= GOOD_FPS and best[0] > 1.5 * fps:
        faster = (best[3].width, best[3].height, best[0])
    cap = open_capture(index, api, mode, short_exposure)
    if cap is None:
        return None
    _cache[key] = (name, api, mode)
    return CameraSelection(cap, name, mode, fps, granted(cap), tried, faster)


def main(argv=None) -> None:
    import argparse
    ap = argparse.ArgumentParser(description="Measure what frame rate the webcam really delivers.")
    ap.add_argument("--index", type=int, default=0)
    args = ap.parse_args(argv)
    print(f"camera {args.index}: measured fps per backend / mode / exposure "
          f"(>= {GOOD_FPS:.0f} is good)\n")
    print(f"{'backend':16s}{'mode requested':20s}{'granted':22s}{'auto exp':>10s}{'short exp':>11s}")
    for name, api in backends():
        for mode in MODES:
            row = []
            g = None
            for short in (False, True):
                cap = open_capture(args.index, api, mode, short)
                if cap is None:
                    row.append("n/a")
                    continue
                row.append(f"{measure_fps(cap, max_seconds=1.5, frames=30):.1f}")
                g = granted(cap)
                cap.release()
            gs = f"{g[0]}x{g[1]} {g[3]} @{g[2]:.0f}" if g else "-"
            print(f"{name:16s}{mode.label:20s}{gs:22s}{row[0]:>10s}{row[1]:>11s}")
    print("\nReading it: if 'auto exp' is ~10 but 'short exp' reaches ~30, the camera is slowing "
          "itself down in low light (more light, or turn off low-light compensation). If 720p is "
          "slow but 480p is fast, it is USB bandwidth / no MJPG. The app picks the first mode that "
          "reaches the target automatically.")


if __name__ == "__main__":
    main()
