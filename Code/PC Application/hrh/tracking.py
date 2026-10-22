"""MediaPipe hand observations -> filtered HRH1 joint targets (Qt-free).

This is everything that happens between "MediaPipe returned some hands" and
"here are 19 firmware angles", pulled out of vision.py's Qt worker so it can be
unit-tested and replayed offline against recorded video (every number quoted
below came from an offline benchmark run on a recorded clip). vision.py now only
owns the camera, the MediaPipe task and the preview.

Per frame, in order:

 1. Hand selection. The tracked hand is followed by image-space continuity
    (nearest palm centre); a jump of more than _SWITCH_DIST is treated as a
    different hand. On acquisition the "Tracked hand" preference decides
    (Right / Left / Auto, where Auto prefers a right hand because HRH1 is
    one). MediaPipe itself runs with num_hands=1 by default -- see
    vision.NUM_HANDS for why 2 is not the default (measured: +18 ms latency
    and a fifth of the frames lost).

 2. Handedness, decided once per track. HRH1 is a right hand, so a left hand
    is mirrored into right-hand geometry before retargeting. MediaPipe's
    label flickers (2 of 621 frames of a clearly visible left hand came back
    "Right" on the bench clip, --flip), and bursts of several frames are possible
    with the back of the hand or an edge-on hand toward the camera.
    Previously every mislabelled frame was retargeted with the wrong
    chirality and warm-started the next solve from it. Now the label is a
    score-weighted vote over the first HANDEDNESS_LOCK_S after acquisition
    and is then LOCKED for the life of the track -- a genuine change of hand
    comes with a track loss or a jump in position, both of which start a new
    track and a new vote. With a fixed Right/Left preference, the vote must
    agree with the preference -- and nothing is sent to the robot until it
    does, so a flickered label can never let the other hand drive it (bench
    --flip --prefer right: 0 frames sent). Bench (--mislabel 200:8, the real
    right hand relabelled "Left" for 8 frames mid-track): before, all 8 were
    retargeted mirrored (up to 52.6 deg off the correctly labelled run);
    after, none (0.0 deg).

    The price is paid when the START of a track is mislabelled (a hand
    entering edge-on): the lock then holds the wrong chirality until a slow
    re-vote (_REVOTE_TAU_S) overturns it (with a fixed preference: releases
    the track) -- --mislabel 0:12 gives 54 mirrored
    frames (~1.8 s) after vs 12 before. Accepted because mid-track bursts
    (back of the hand, edge-on) are the common case. Checked and NOT used as
    a tie-breaker: the IK residual of the two chiralities picks the right one
    on every sampled frame of the clip, but on the frames where MediaPipe
    mislabels, its 3D landmarks are fooled too (frame 508 of the mirrored
    clip fits the wrong hand better), so it is not independent evidence.

    (Verified along the way: the Tasks API labels UNMIRRORED frames
    correctly, unlike the legacy mp.solutions API which assumed selfie input
    -- on the same frames the two APIs give opposite labels. The IK residual
    agrees: fitting a right hand unmirrored costs 0.36 vs 0.66 mirrored. So
    feeding MediaPipe the unmirrored frame and mirroring "Left" is right.)

 3. Track loss. When no hand has been seen for LOST_RESET_S, the smoother,
    One Euro filter, handedness vote and the solver's warm start are all
    reset, so nothing from the previous track (a warm start from the other
    hand's geometry, say) can leak into the next one. The output then ramps
    from the last commanded pose to the new one over REACQUIRE_BLEND_S
    instead of stepping. Bench (--blank 300:330: a 1 s dropout cut into the
    real clip, each pipeline compared with itself on the uninterrupted clip):
    the largest step within 0.5 s of re-acquiring is 21.4 deg before / 13.8
    deg after; the ramp costs some error while it runs (0.67 -> 0.99 deg mean
    over the following 2 s), and from 0.5 s on both are within 0.1 deg of the
    uninterrupted run (0.05 / 0.07).

 4. Finger IK (ik_retarget, with an abduction-flexion coupling prior),
    optional wrist solve (wrist_retarget, rate-limited, ramps back to 0 when
    switched off), then the One Euro filter on the final angles.

Filtering was re-tuned on the real clip with lag measured (--video CLIP
--sweep filter): the previous 2-frame landmark pre-average + One Euro(1.5,
0.1) has a still-hand jitter of 0.098 deg/frame^2 and 26 ms of lag against
the unfiltered solve; no pre-average + One Euro(0.5, 0.1) gives 0.088 and
11 ms. Put plainly: 15 ms less lag, jitter about the same (the --video runs
show 0.093 -> 0.088 on the clip, 0.091 -> 0.091 mirrored, 0.103 -> 0.108 with
the fingertips cropped). Nearly all of the lag saving comes from dropping
the 2-frame landmark mean (window 2 with One Euro(0.5, 0.1) is still 27 ms):
real MediaPipe noise is strongly correlated frame to frame, so averaging
consecutive frames mostly adds lag. On synthetic ground truth (--synthetic
--sweep filter) the new setting beats the old one (6.62 -> 6.00 deg with
white landmark noise, 8.78 -> 7.95 deg with correlated noise) but is no
more accurate than no filter at all (6.03 / 7.42 deg) -- the filter is
there for smoothness on the servos, not accuracy. Being faster it follows
genuinely fast motion more closely, so the largest single-frame change on
the clip rises (36 -> 47 deg); the firmware slews each joint at <= 400 deg/s
regardless.

Tried and rejected, so nobody re-adds them blind: down-weighting landmarks
MediaPipe places outside the image made cropped-fingertip tracking WORSE in
a development experiment (2.97 -> 3.63 deg mean error against the uncropped
run -- its extrapolated landmarks beat the solver's own guess; the code was
removed, so this one number is not reproducible from the bench); gating
frames on a high solver residual would freeze the hand, because on real
video the high-residual stretches are sustained (a pose the robot cannot
make), not glitches; and a one-frame spike filter has nothing to remove
(the bench's "one-frame spikes > 10 deg" row is 0 on the clip and 1 on the
mirrored clip -- the large single-frame changes are genuine fast motion).
"""
from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

import numpy as np

from .joints import JOINTS, NUM_JOINTS
from .one_euro import VectorOneEuroFilter

# --- defaults (see module docstring for the measurements) -------------------
DEFAULT_MINCUTOFF = 0.5
DEFAULT_BETA = 0.1
DEFAULT_LANDMARK_WINDOW = 1   # 1 = no landmark pre-average

#: when a hand is (re)acquired, the output ramps from the last commanded pose
#: to the new one over this long instead of stepping. Only applies when there
#: WAS a previous pose (the first acquisition of a session goes straight in).
REACQUIRE_BLEND_S = 0.2

#: seconds without a hand before the track is considered lost and all
#: temporal state is reset. Short dropouts (a frame or two of motion blur)
#: stay well under this and keep their warm start.
LOST_RESET_S = 0.3

#: handedness is voted over this long after acquisition (at least
#: _LOCK_MIN_FRAMES frames), then fixed until the track ends.
HANDEDNESS_LOCK_S = 0.3
_LOCK_MIN_FRAMES = 5

#: In Auto mode a locked decision can still be overturned, but only by
#: SUSTAINED contrary labels: a running average of the labels (time constant
#: _REVOTE_TAU_S) has to swing past -_REVOTE_LEVEL. That takes ~1.5 s of
#: confident contrary labels -- far longer than any flicker burst seen in
#: testing (a 20-frame burst moves it barely past zero) -- and exists so that
#: a track whose first 0.3 s happened to be mislabelled (a hand entering the
#: view edge-on) is not stuck with the wrong chirality for its whole life.
_REVOTE_TAU_S = 1.0
_REVOTE_LEVEL = 0.5

#: wrist outputs never move faster than this (deg/s) -- the robot's own wrist
#: slew ceiling (joints.WRIST_MAX_SPEED). Covers switching wrist tracking on
#: or off and re-capturing the neutral, which would otherwise step the
#: heaviest axes by up to 90 deg when filtering is off.
WRIST_SLEW_DEG_S = 240.0

# A tracked hand whose palm centre jumps further than this (normalised image
# units) between detections is treated as a different hand: re-acquire.
_SWITCH_DIST = 0.25

_PALM_IDX = (0, 5, 9, 13, 17)
_WRIST_IDX = [j.index for j in JOINTS if j.name in ("W_1", "W_2", "W_3")]
_INDEX = {j.name: j.index for j in JOINTS}

HAND_PREFERENCES = ("auto", "right", "left")


@dataclass
class HandObservation:
    """One detected hand, as MediaPipe reports it."""
    world: np.ndarray                    # (21, 3) world landmarks, metres
    image: Optional[np.ndarray] = None   # (21, 2|3) normalised image landmarks
    label: str = ""                      # handedness category ("Left"/"Right")
    score: float = 0.0                   # handedness confidence

    @property
    def is_left_label(self) -> bool:
        return str(self.label).strip().lower().startswith("l")

    def palm_centre(self) -> Optional[np.ndarray]:
        if self.image is None:
            return None
        c = np.asarray(self.image, float)[list(_PALM_IDX), :2].mean(axis=0)
        return c if np.all(np.isfinite(c)) else None


@dataclass
class TrackingOutput:
    angles: Optional[List[float]]   # 19 firmware degrees, None = no new target
    state: str                      # "tracking" | "confirming" | "searching" | "wrong_hand"
    hand: Optional[str] = None      # "Right" / "Left" as retargeted
    index: Optional[int] = None     # which observation was used
    mirrored: bool = False
    event: Optional[str] = None     # "acquired" | "lost" | "handedness_changed" | "released"
    locked: bool = False            # handedness decision final for this track


class LandmarkSmoother:
    """Rolling mean over the last `window` frames of raw landmark positions,
    applied before retargeting. window=1 is a pass-through (the default: on
    real video this stage mostly added lag -- see the module docstring)."""

    def __init__(self, window: int = DEFAULT_LANDMARK_WINDOW):
        self._buf: deque = deque(maxlen=max(1, int(window)))

    def reset(self) -> None:
        self._buf.clear()

    def smooth(self, points: np.ndarray) -> np.ndarray:
        self._buf.append(np.asarray(points, float))
        return np.mean(self._buf, axis=0)


class HandTrackingPipeline:
    """Stateful per-frame processor. Call process() once per camera frame,
    with every hand MediaPipe found (possibly none), and a timestamp in
    seconds, always from the same thread.

    The set_*/request_* methods may be called from any other thread (the
    GUI): they only store a value or raise a flag, and process() applies it
    at the start of its next frame, so no piece of state is ever modified
    by two threads."""

    def __init__(
        self,
        retargeter=None,
        wrist=None,
        *,
        fallback: Optional[Callable[[Sequence], List[float]]] = None,
        mincutoff: float = DEFAULT_MINCUTOFF,
        beta: float = DEFAULT_BETA,
        landmark_window: int = DEFAULT_LANDMARK_WINDOW,
        filtering: bool = True,
        hand_preference: str = "auto",
        handedness_vote: bool = True,
        lost_reset_s: Optional[float] = LOST_RESET_S,
        reacquire_blend_s: float = REACQUIRE_BLEND_S,
        continuity: bool = True,
    ):
        if retargeter is None and fallback is None:
            raise ValueError("need a retargeter or a fallback retarget function")
        self.retargeter = retargeter
        self.wrist = wrist
        self._fallback = fallback
        self._mincutoff, self._beta = mincutoff, beta
        self._smoother = LandmarkSmoother(landmark_window)
        self._filter = self._new_filter()
        self.handedness_vote = handedness_vote
        self.lost_reset_s = lost_reset_s
        self.reacquire_blend_s = reacquire_blend_s
        self.continuity = continuity

        # Requested settings (written by any thread) and the values in force
        # (only touched by process()).
        self._req_filtering = bool(filtering)
        self._req_pref = hand_preference if hand_preference in HAND_PREFERENCES else "auto"
        self._req_wrist = False
        self._req_neutral = False
        self._req_pinch: Optional[bool] = None
        self.filtering = self._req_filtering
        self.hand_preference = self._req_pref
        self.wrist_enabled = False

        # track state
        self._tracking = False
        self._last_seen_t: Optional[float] = None
        self._last_t: Optional[float] = None
        self._centre: Optional[np.ndarray] = None
        self._is_left = False
        self._locked = False
        self._vote_sum = 0.0
        self._vote_n = 0
        self._lock_at = 0.0
        self._confirming = False
        self._revote = 0.0
        self._neutral_left = False   # chirality the wrist neutral was captured in

        # output shaping
        self._last_output: Optional[np.ndarray] = None
        self._blend_from: Optional[np.ndarray] = None
        self._blend_t0 = 0.0
        self._wrist_out = np.zeros(len(_WRIST_IDX))

        # counters for diagnostics
        self.acquisitions = 0
        self.resets = 0
        self.flicker_suppressed = 0
        self.handedness_changes = 0
        self.last_residual: Optional[float] = None

    # -- settings (any thread) -----------------------------------------------

    def set_filtering_enabled(self, enabled: bool) -> None:
        self._req_filtering = bool(enabled)

    def set_hand_preference(self, pref: str) -> None:
        pref = str(pref).lower()
        if pref in HAND_PREFERENCES:
            self._req_pref = pref

    def set_wrist_enabled(self, enabled: bool) -> None:
        self._req_wrist = bool(enabled) and self.wrist is not None

    def set_pinch_snap(self, enabled: bool) -> None:
        """Fingertip snapping (ik_retarget's DexPilot pinch projection) on or
        off. Applied at the start of the next frame, on the solver's thread."""
        self._req_pinch = bool(enabled)

    def request_wrist_neutral(self) -> None:
        self._req_neutral = True

    @property
    def wrist_status(self) -> str:
        if self.wrist is None:
            return "unavailable"
        if not (self.wrist_enabled or self._req_wrist):
            return "off"
        if self.wrist.capturing:
            return "capturing neutral (hold still)"
        return "on" if self.wrist.has_neutral else "waiting for hand"

    # -- state -------------------------------------------------------------

    def _new_filter(self) -> VectorOneEuroFilter:
        return VectorOneEuroFilter(NUM_JOINTS, mincutoff=self._mincutoff, beta=self._beta)

    def _reset_temporal(self) -> None:
        """Smoother, One Euro and solver warm start: everything that assumes
        consecutive frames come from one continuous motion of one hand."""
        self._smoother.reset()
        self._filter = self._new_filter()
        if self.retargeter is not None:
            self.retargeter.reset()

    def reset(self) -> None:
        """Forget the track and all temporal state."""
        self._reset_temporal()
        self._tracking = False
        self._centre = None
        self._locked = False
        self._confirming = False
        if self.wrist is not None and self.wrist.capturing:
            self.wrist.request_neutral()   # restart: drop frames from the old track
        self.resets += 1

    @property
    def tracking(self) -> bool:
        return self._tracking

    def _apply_requests(self) -> None:
        pinch, self._req_pinch = self._req_pinch, None
        if pinch is not None and self.retargeter is not None and hasattr(self.retargeter, "pinch_snap"):
            self.retargeter.pinch_snap = pinch
        if self._req_filtering != self.filtering:
            self.filtering = self._req_filtering
            if self.filtering:
                # Re-enabled: the smoother and filter hold whatever they last
                # saw, possibly from long ago. Start them fresh.
                self._smoother.reset()
                self._filter = self._new_filter()
        if self._req_pref != self.hand_preference:
            self.hand_preference = self._req_pref
            self.reset()   # re-select from scratch under the new rule
        if self._req_wrist != self.wrist_enabled:
            self.wrist_enabled = self._req_wrist
            if self.wrist_enabled and not self.wrist.has_neutral:
                self.wrist.request_neutral()
        if self._req_neutral:
            self._req_neutral = False
            if self.wrist is not None:
                self.wrist.request_neutral()

    # -- per frame -----------------------------------------------------------

    def process(self, hands: Sequence[HandObservation], t: float) -> TrackingOutput:
        self._apply_requests()
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t

        hands = [h for h in hands if h is not None and np.all(np.isfinite(np.asarray(h.world, float)))]
        gap = None if self._last_seen_t is None else t - self._last_seen_t
        long_gap = (self._tracking and self.lost_reset_s is not None
                    and gap is not None and gap > self.lost_reset_s)

        if not hands:
            event = None
            if long_gap:
                self.reset()
                event = "lost"
            return TrackingOutput(self._wrist_home_only(dt), "searching", event=event)

        # A hand is back after a long gap: start clean even if no empty frame
        # arrived in between to trigger the reset (LIVE_STREAM drops frames).
        if long_gap:
            self.reset()

        idx = self._select(hands)
        if idx is None:
            # Only the non-preferred hand is in view: like no hand (a long
            # gap has already reset the track just above).
            return TrackingOutput(self._wrist_home_only(dt), "wrong_hand",
                                  event="lost" if long_gap else None)
        obs = hands[idx]

        event = None
        if not self._tracking:
            self._start_track(obs, t)
            event = "acquired"
        else:
            was_locked = self._locked
            changed = self._update_handedness(obs, t, dt)
            if changed == "released":
                self.reset()
                return TrackingOutput(None, "wrong_hand", event="released")
            if self._locked and not was_locked and self._confirming:
                # Fixed preference confirmed: output starts now, so ramp in
                # from the last command from now.
                self._confirming = False
                self._begin_blend(t)
            if changed:
                # Chirality flipped at lock time: the warm start and filter
                # history belong to the other-handed geometry. Start them over
                # and ramp the output across instead of stepping.
                self._reset_temporal()
                self._begin_blend(t)
                self.handedness_changes += 1
                event = "handedness_changed"
        self._last_seen_t = t
        c = obs.palm_centre()
        if c is not None:
            self._centre = c

        world = np.array(obs.world, dtype=float)
        if self._is_left:
            world[:, 0] = -world[:, 0]   # left hand -> right-hand geometry
        if self.filtering:
            world = self._smoother.smooth(world)

        if self.retargeter is not None:
            angles = np.asarray(self.retargeter.landmarks_to_angles(world), float)
            self.last_residual = float(getattr(self.retargeter, "last_cost", 0.0))
        else:
            angles = np.asarray(self._fallback(world), float)

        # Not while a fixed-preference track is still being confirmed: a
        # neutral capture must not take frames from a hand about to be
        # released.
        angles[_WRIST_IDX] = self._wrist_out if self._confirming else self._wrist_step(world, dt)

        if self.filtering:
            angles = np.asarray(self._filter.filter(list(angles), t), float)
        if self._confirming:
            # Fixed Right/Left preference: nothing reaches the robot until
            # the vote has confirmed this really is that hand (a single
            # flickered label on the other hand would otherwise drive the
            # robot for the whole vote window). The solve and filter still
            # run, so they are warm when output starts.
            return TrackingOutput(None, "confirming", index=idx, mirrored=self._is_left, event=event)
        angles = self._blend_in(angles, t)
        self._last_output = angles
        return TrackingOutput(
            [float(a) for a in angles], "tracking", hand="Left" if self._is_left else "Right",
            index=idx, mirrored=self._is_left, event=event, locked=self._locked,
        )

    # -- helpers -------------------------------------------------------------

    def _start_track(self, obs: HandObservation, t: float) -> None:
        self._tracking = True
        self.acquisitions += 1
        self._locked = not self.handedness_vote
        self._vote_sum, self._vote_n = 0.0, 0
        self._lock_at = t + HANDEDNESS_LOCK_S
        if self.hand_preference != "auto":
            self._is_left = self.hand_preference == "left"
        else:
            self._is_left = obs.is_left_label
        self._confirming = self.hand_preference != "auto" and not self._locked
        self._add_vote(obs)
        self._begin_blend(t)

    def _begin_blend(self, t: float) -> None:
        # Ramp from whatever the robot was last told, not a step.
        self._blend_from = None if self._last_output is None else self._last_output.copy()
        self._blend_t0 = t

    def _add_vote(self, obs: HandObservation) -> None:
        self._vote_sum += (1.0 if obs.is_left_label else -1.0) * max(float(obs.score), 0.05)
        self._vote_n += 1

    def _update_handedness(self, obs: HandObservation, t: float, dt: float = 0.0):
        """Returns True if the chirality decision changed, "released" if a
        fixed-preference track turned out to be the other hand, else False."""
        sample = (1.0 if obs.is_left_label else -1.0) * max(float(obs.score), 0.05)
        if not self.handedness_vote:
            # Legacy behaviour (kept for the bench's "before" pipeline):
            # trust every frame's label, and reset nothing when it flips.
            if self.hand_preference == "auto":
                self._is_left = obs.is_left_label
            return False
        if self._locked:
            if obs.is_left_label != self._is_left:
                self.flicker_suppressed += 1
            a = min(1.0, dt / _REVOTE_TAU_S) if dt > 0 else 0.0
            self._revote = (1.0 - a) * self._revote + a * sample
            if (self._revote < -_REVOTE_LEVEL and self._is_left) or \
                    (self._revote > _REVOTE_LEVEL and not self._is_left):
                if self.hand_preference != "auto":
                    # Sustained evidence this is the other hand after all
                    # (its first 0.3 s were mislabelled): let go of it.
                    return "released"
                self._is_left = not self._is_left
                self._revote = 1.0 if self._is_left else -1.0
                return True
            return False
        self._add_vote(obs)
        voted_left = self._vote_sum > 0.0
        if self.hand_preference != "auto" and voted_left != (self.hand_preference == "left"):
            return "released"      # the running vote already says: other hand
        if t < self._lock_at or self._vote_n < _LOCK_MIN_FRAMES:
            if self.hand_preference == "auto":
                was = self._is_left
                self._is_left = voted_left      # provisional, follows the vote
                return self._is_left != was
            return False
        self._locked = True
        if self.hand_preference != "auto":
            self._revote = 1.0 if self._is_left else -1.0
            return False
        was = self._is_left
        self._is_left = voted_left
        self._revote = 1.0 if self._is_left else -1.0
        return self._is_left != was

    def _wrist_home_only(self, dt: float) -> Optional[List[float]]:
        """With no usable hand, the only thing still worth sending is a wrist
        that was switched off mid-deflection and hasn't reached home yet."""
        if self.wrist_enabled or self._last_output is None or not np.any(self._wrist_out):
            return None
        step = WRIST_SLEW_DEG_S * dt if dt > 0 else 0.0
        self._wrist_out = self._wrist_out + np.clip(-self._wrist_out, -step, step)
        out = self._last_output.copy()
        out[_WRIST_IDX] = self._wrist_out
        self._last_output = out
        return [float(a) for a in out]

    def _select(self, hands: Sequence[HandObservation]) -> Optional[int]:
        cand = list(range(len(hands)))
        if self._tracking and not self.continuity:
            return 0   # legacy: whatever MediaPipe lists first
        if self._tracking and self._centre is not None:
            # Follow the tracked hand by continuity, whatever its label says
            # this frame -- labels flicker, positions don't jump.
            dists = []
            for i in cand:
                c = hands[i].palm_centre()
                dists.append(np.inf if c is None else float(np.linalg.norm(c - self._centre)))
            best = int(np.argmin(dists))
            if dists[best] <= _SWITCH_DIST:
                return cand[best]
            # Nothing near the tracked hand: it is a different hand. Start a
            # fresh track (full reset) and pick afresh below.
            self.reset()
        elif self._tracking and len(cand) == 1:
            return 0   # no image landmarks to judge continuity by
        pref = self.hand_preference
        if pref != "auto":
            want_left = pref == "left"
            cand = [i for i in cand if hands[i].is_left_label == want_left]
            if not cand:
                return None
        else:
            rights = [i for i in cand if not hands[i].is_left_label]
            if rights:
                cand = rights
        return max(cand, key=lambda i: hands[i].score)

    def _wrist_step(self, world: np.ndarray, dt: float) -> np.ndarray:
        """This frame's wrist angles (firmware deg, W_1..W_3 order)."""
        target = np.zeros(len(_WRIST_IDX))
        if self.wrist_enabled and self.wrist is not None:
            # A neutral captured from the other hand lives in mirrored
            # coordinates and would be meaningless: capture a fresh one.
            if self.wrist.has_neutral and not self.wrist.capturing and self._neutral_left != self._is_left:
                self.wrist.request_neutral()
            if self.wrist.capturing:
                self._neutral_left = self._is_left
            deg = self.wrist.update(self._palm_frame(world))
            if deg is None or self.wrist.capturing:
                target = self._wrist_out.copy()     # hold while capturing
            else:
                target = np.array([deg[JOINTS[i].name] for i in _WRIST_IDX])
        step = WRIST_SLEW_DEG_S * dt if dt > 0 else 0.0
        self._wrist_out = self._wrist_out + np.clip(target - self._wrist_out, -step, step)
        return self._wrist_out

    def _blend_in(self, angles: np.ndarray, t: float) -> np.ndarray:
        """Linear-in-smoothstep ramp from the last commanded pose to the new
        track over reacquire_blend_s, so a re-acquired hand in a different
        pose doesn't command a full-range step."""
        if self._blend_from is None or self.reacquire_blend_s <= 0:
            return angles
        k = (t - self._blend_t0) / self.reacquire_blend_s
        if k >= 1.0:
            self._blend_from = None
            return angles
        k = max(k, 0.0)
        k = k * k * (3.0 - 2.0 * k)   # zero velocity at both ends
        return self._blend_from + (angles - self._blend_from) * k

    @staticmethod
    def _palm_frame(world: np.ndarray) -> np.ndarray:
        from .ik_retarget import palm_frame
        return palm_frame(world[0], world[5], world[9], world[17])


class LatestValueWorker:
    """Runs pipeline.process on its own thread, always on the NEWEST input.

    MediaPipe's LIVE_STREAM result callback runs on MediaPipe's own graph
    thread. The ~10 ms IK solve used to run inside it, which stalls the graph.
    Measured with the bench's --live mode (60 fps feed, one hand, 2-core
    machine), frame-to-angles latency mean / p95: 30.8 / 55.8 ms with the
    solve in the callback vs 27.4 / 39.9 ms on this worker (an independent
    rerun: 31.8 / 56.6 vs 29.7 / 48.1). So a few ms on the mean and 8-16 ms
    on the tail -- modest on 2 cores, where MediaPipe and the solver compete;
    more headroom on a typical 4+ core PC. If the solver falls behind, stale
    results are skipped (counted in `dropped`) rather than queued, so latency
    cannot build up; a skipped frame is simply superseded by a newer one.
    """

    def __init__(self, pipeline: HandTrackingPipeline,
                 on_output: Callable[[TrackingOutput, int, float, int], None],
                 on_error: Optional[Callable[[Exception], None]] = None):
        self.pipeline = pipeline
        self._on_output = on_output
        self._on_error = on_error
        self._cv = threading.Condition()
        self._item = None
        self._stop = False
        self.dropped = 0
        self._thread = threading.Thread(target=self._loop, name="hand-retarget", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def submit(self, hands: Sequence[HandObservation], t: float, tag=None) -> None:
        """Called from the MediaPipe callback. Never blocks on the solver."""
        with self._cv:
            if self._item is not None:
                self.dropped += 1
            self._item = (hands, t, tag)
            self._cv.notify()

    def stop(self, timeout: float = 2.0) -> None:
        with self._cv:
            self._stop = True
            self._cv.notify()
        if self._thread.is_alive():
            self._thread.join(timeout)

    def _loop(self) -> None:
        import time as _time
        while True:
            with self._cv:
                while self._item is None and not self._stop:
                    self._cv.wait()
                if self._stop:
                    return
                (hands, t, tag), self._item = self._item, None
            try:
                t0 = _time.perf_counter()
                out = self.pipeline.process(hands, t)
                self._on_output(out, len(hands), (_time.perf_counter() - t0) * 1000.0, tag)
            except Exception as exc:  # noqa: BLE001 - never let the worker die
                if self._on_error is not None:
                    self._on_error(exc)


def observations_from_result(result) -> List[HandObservation]:
    """MediaPipe HandLandmarkerResult -> [HandObservation]. Defensive about
    the handedness field's name, which moved between Tasks API versions."""
    out: List[HandObservation] = []
    worlds = getattr(result, "hand_world_landmarks", None) or []
    images = getattr(result, "hand_landmarks", None) or []
    hands = getattr(result, "handedness", None) or []
    for i, wl in enumerate(worlds):
        world = np.array([(p.x, p.y, p.z) for p in wl], dtype=float)
        if world.shape != (21, 3):
            continue
        image = None
        if i < len(images):
            image = np.array([(p.x, p.y, p.z) for p in images[i]], dtype=float)
        label, score = "", 0.0
        try:
            cat = hands[i][0]
            label = getattr(cat, "category_name", None) or getattr(cat, "display_name", "") or ""
            score = float(getattr(cat, "score", 0.0) or 0.0)
        except Exception:  # noqa: BLE001 - absent/renamed field
            pass
        if not math.isfinite(score):
            score = 0.0
        out.append(HandObservation(world, image, str(label), score))
    return out
