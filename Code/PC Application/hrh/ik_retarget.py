"""Optimisation-based MediaPipe -> HRH1 retargeting against the real CAD kinematics.

Replaces an earlier direct-angle-measurement approach, which measured
each joint independently with a hand-tuned gain/sign/neutral per joint. That
had two structural problems:

  1. It leaned hardest on MediaPipe's *worst* signal. "MCP flexion" was
     measured as elevation out of the palm plane, which -- with the palm
     facing the camera -- is read almost entirely off the landmark z, and
     MediaPipe's z is a learned prior, not a metric depth measurement. The
     synthetic tests all passed because the synthetic generator produced
     perfect z; a real camera does not.
  2. Measuring joints independently ignores the robot's actual geometry.
     HRH1's MCP axis is not exactly perpendicular to the palm plane, its
     abduction axis is not exactly the palm normal, and the thumb saddle is
     two skew axes 19 mm apart. Every gain/sign/neutral constant in
     that approach was a patch over that mismatch.

So instead of measuring angles, this solves for them: find the joint vector q
whose *robot* keypoint geometry, computed by the exact URDF forward kinematics
in urdf_model.py, best matches the human's. This is the approach the
teleoperation literature converged on -- AnyTeleop / dex-retargeting
(https://github.com/dexsuite/dex-retargeting, MIT) and DexPilot
(https://arxiv.org/abs/1910.03135). The cost function here follows theirs;
the solver does not (see "Solver" below).

Cost function
-------------
Three kinds of term, all dimensionless so their weights are comparable:

  * Segment directions (the workhorse). For every bone of every digit, match
    the *unit direction* of the robot's segment to the human's. Using
    directions rather than vectors makes these completely immune to hand
    size, to MediaPipe's overall depth-scale error, and to any need for
    calibration. Each finger gets 2 direction terms (4 residual dims) for 3
    DOF, and the thumb 3 terms (6 dims) for 4 DOF, so every digit is
    over-determined -- which is what buys noise rejection.

  * Pinch vectors (DexPilot). Thumb tip -> each fingertip, in units of palm
    width. When the human's tip-to-tip distance drops below a threshold the
    target is *projected* to a small fixed length and the weight jumps ~20x,
    which forces the robot's tips to actually meet instead of hovering near
    each other. This is the term that makes pinches crisp despite imperfect
    pose estimation.

  * Regularisation toward the previous frame's solution. Doubles as temporal
    smoothing (inside the solve, where it cannot fight the data) and as
    Tikhonov conditioning that keeps the solver on one continuous branch of
    the IK solution set rather than flicking between branches.

Frames
------
Human landmarks arrive in MediaPipe's world frame, whose orientation follows
the camera. Both sides are therefore expressed in a canonical palm frame
(forward / lateral / normal) built the same way from wrist + knuckles. The
robot's frame is built ONCE at q=0 from points rigidly attached to the palm
(the W_2 pivot and the four abduction pivots) -- valid precisely because the
three wrist joints are excluded from the solve, so the palm never moves.

Solver
------
dex-retargeting uses NLopt SLSQP with pinocchio kinematics and torch
autograd. That stack (pinocchio especially) is painful to install on Windows,
so this implements the same objective with a projected Levenberg-Marquardt
solver in ~40 lines of numpy. That is not a downgrade: this is a nonlinear
*least-squares* problem, which is exactly what LM is for, and LM converges in
fewer iterations than a general-purpose SLSQP. Gradients are the exact
analytic geometric Jacobian from urdf_model.point_jacobians (verified against
finite differences to 1e-11 m), not autograd or finite differences.

Warm-started from the previous frame, it converges in a handful of iterations.
No camera or display is needed to test any of this -- see the round-trip test
in the __main__ block, which poses the robot, generates the landmarks that
pose would produce, solves, and checks the angles come back.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .hrh1_urdf_map import (
    HRH1_URDF_NAMES,
    HRH1_URDF_SIGNS,
    SOLVED_FIRMWARE_JOINTS,
    hrh1_assembly_xacro,
)
from .joints import JOINTS, JOINTS_BY_NAME, NUM_JOINTS, clamp_deg
from .urdf_model import URDFModel

_EPS = 1e-9

# --- MediaPipe's fixed 21-landmark layout -------------------------------
WRIST = 0
THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP = 1, 2, 3, 4
# per finger: (mcp, pip, tip); the DIP landmark is skipped because HRH1
# fuses the middle and distal phalanges into one link.
FINGER_LANDMARKS = {
    "index": (5, 6, 8),
    "middle": (9, 10, 12),
    "ring": (13, 14, 16),
    "pinky": (17, 18, 20),
}

# --- robot side: which URDF joint pivot / link point matches each landmark ---
# A finger's knuckle is its *_MCP pivot, its middle joint the *_IP pivot, and
# its tip the vertex of the distal link furthest from that *_IP pivot.
_FINGER_JOINTS = {
    "index": ("I_MP", "I_MCP", "I_IP", "I_DI_1"),
    "middle": ("M_ABD", "M_MCP", "M_IP", "M_DI_1"),
    "ring": ("R_ABD", "R_MCP", "R_IP", "R_DI_1"),
    "pinky": ("P_ABD", "P_MCP", "P_IP", "P_DI_1"),
}
_THUMB_JOINTS = ("T_ABD", "T_CMC", "T_MCP", "T_DP", "T_D_1")

# --- tuning -------------------------------------------------------------
# Direction terms come in three spans, and the split is the single most
# important thing in this file for noise rejection.
#
# A landmark position error turns into a direction error of roughly
# atan(err / span), so the SAME camera noise is far more damaging on a short
# bone than on a long one. On HRH1: a proximal phalanx is ~39 mm, so 2 mm of
# landmark noise is ~3 deg; knuckle-to-tip is ~87 mm, so ~1.3 deg; and
# wrist-to-tip is ~200 mm, so ~0.6 deg. Matching only adjacent bones (the
# obvious thing to do, and what the first version of this did) therefore
# throws away the measurements that are most robust.
#
# So every digit is constrained at all three spans at once. The long spans
# pin down overall posture cheaply and quietly; the short spans resolve the
# individual joints. They are redundant on clean data and deliberately so:
# under noise the redundancy averages out, which is the whole point.
_W_SEGMENT = 1.0       # adjacent bones  (~30-47 mm span)
_W_SPAN2 = 1.6         # knuckle -> tip  (~87 mm span)
_W_SPAN3 = 2.0         # wrist -> tip    (~200 mm span)
_W_PINCH = 0.35        # baseline weight on thumb-tip -> fingertip vectors
_W_PINCH_CLOSE = 7.0   # ...raised to this once the human tips are close
_PROJECT_DIST = 0.030  # m: human tip gap below which a pinch is "intended"
_ESCAPE_DIST = 0.055   # m: ...and above which it is released (hysteresis)
_PINCH_TARGET = 0.004  # m: gap the projected pinch target asks the robot for
# Pull toward the previous frame's solution, per radian. Deliberately gentle:
# the One Euro filter downstream already does the bulk of the temporal
# smoothing, and measurements showed raising this past ~0.3 starts costing
# accuracy faster than it buys smoothness (at 1.0 the still-hand jitter is
# only 27% lower but tracking error is 16% worse). Its other job -- keeping
# the solver on one continuous branch of the IK solution set instead of
# flicking between branches -- is done at any nonzero value.
_W_REG = 0.3

_LM_LAMBDA0 = 1e-3
_MIN_SEG = 1e-4        # m: shorter than this and a direction is meaningless

# How much to trust the camera-depth component of a residual, relative to the
# two in-image-plane components. MediaPipe's x/y are sub-pixel accurate but
# its z is a learned prior rather than a measurement, and is several times
# noisier; weighting all three equally lets the worst channel dominate a
# least-squares fit. So each residual is scaled down along the camera's depth
# axis (expressed in the palm frame, recomputed per frame as the hand turns).
#
# Measured on synthetic trajectories, the error falls monotonically as this
# drops and then plateaus: 1.0 -> 6.23 deg, 0.4 -> 5.46, 0.2 -> 5.20,
# 0.12 -> 5.14, and 0.0 -> 5.16. That the plateau extends all the way to zero
# is the striking part: MediaPipe's depth contributes essentially nothing
# beyond noise, and the in-plane landmarks alone determine the pose. A small
# nonzero value is kept anyway, as a tie-breaker for the poses where in-plane
# information really is degenerate -- a finger pointed straight at the camera.
_DEPTH_WEIGHT = 0.15

# Huber threshold on each 3-vector residual: terms larger than this have their
# influence capped (implemented as IRLS re-weighting inside the LM loop), so
# one badly-placed landmark cannot drag the whole hand. Worth ~10% with
# occasional gross errors from occlusion, and ~4% even without them, since
# ordinary noise still throws the occasional large residual that plain least
# squares over-weights. Set to None for pure least squares.
_HUBER_DELTA = 0.10

# Flexion-dependent abduction prior (the "abduction-flexion coupling").
#
# A finger's side-to-side (abduction) angle is only observable while the finger
# is roughly straight. Once the proximal phalanx curls toward the palm, a pure
# abduction rotation mostly swings the fingertip along the line of sight, and
# the curled segments are partly hidden behind the knuckles -- so MediaPipe's
# landmarks say very little about it and the solver is free to put it anywhere.
# On a real recorded hand (offline benchmark) that showed up as abduction
# pinned at a joint LIMIT (+-24 deg) in about a third of all curled-finger
# frames -- i.e. the robot splaying curled fingers into each other.
#
# Human hands do the opposite: the MCP collateral ligaments tighten in flexion,
# so abduction range shrinks towards zero in a fist. This term encodes exactly
# that: a pull of abduction toward 0 whose weight ramps smoothly (smoothstep)
# from zero at _COUPLE_FLEX_LO of MCP flexion to _W_COUPLE at _COUPLE_FLEX_HI.
# A straight finger is completely unaffected. The weight is per radian of
# abduction, so it is comparable with the unit-direction residuals.
#
# Tuned with an offline benchmark (a recorded clip and synthetic poses). On the real recording, the share of curled-finger frames with
# abduction pinned at a limit is 34.3% without the prior and 22.0% / 8.9% /
# 0.5% / 0.0% at weight 0.3 / 0.6 / 1.0 / 2.0. On synthetic ground truth (2 mm
# noise, human-like poses) weight 1.0 takes the curled-finger abduction error
# from 5.56 to 3.89 deg with the other joints unchanged (4.92 -> 4.83) and the
# best overall mean (5.22 deg). The price is on poses no human makes -- a
# fully curled finger that is ALSO splayed: on uniformly random "free" poses
# the mean error goes 6.11 / 6.28 / 6.47 deg at weight 0 / 1.0 / 1.5.
_W_COUPLE = 1.0
_COUPLE_FLEX_LO = np.radians(25.0)
_COUPLE_FLEX_HI = np.radians(75.0)

# Max Levenberg-Marquardt iterations per frame. Warm-started from the last
# frame it converges in about 4; 6 leaves margin for the first frame after a
# re-detection without costing much (measured 7.8 ms/solve => ~129 Hz, against
# a 30 Hz camera).
_LM_ITERS = 6


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > _EPS else v


def palm_frame(wrist: np.ndarray, index_mcp: np.ndarray, middle_mcp: np.ndarray,
               pinky_mcp: np.ndarray) -> np.ndarray:
    """3x3 rotation whose ROWS are (lateral, forward, normal).

    Multiplying a world vector by this expresses it in the palm's own frame,
    which is what makes the human and robot comparable regardless of where
    the camera is. Built identically for both sides so any bias cancels.

    The KNUCKLE ROW is the primary axis, and forward is derived from it.
    This used to be the other way round -- forward = (middle knuckle - wrist)
    taken as primary, with lateral re-orthogonalised against it -- and that
    was the cause of a large, systematic abduction bias: every finger read
    about -15 deg of splay on a hand whose fingers were dead straight.

    The reason is that the wrist landmark is not square behind the middle
    knuckle. On a human, MediaPipe's wrist sits toward the heel of the palm,
    so (middle_mcp - wrist) leans several degrees across the hand; on the
    robot, the equivalent reference is the W_2 pivot, which the CAD places
    2.3 mm to one side. Those two leans differ, so the human and robot
    frames ended up rotated relative to each other about the palm normal --
    and a rotation about the palm normal is *exactly* a constant abduction
    offset, applied identically to all four fingers.

    The knuckle row has no such problem: index-to-pinky is a long, well-
    conditioned baseline measured between two landmarks of the same kind on
    both sides. Deriving forward as perpendicular to it inside the palm
    plane leaves the wrist contributing only the plane's sense, where a few
    millimetres sideways cannot rotate anything. Measured on a synthetic
    straight hand, this takes the bias from -14.7 deg to -1.1 deg.
    """
    lateral = _unit(index_mcp - pinky_mcp)       # pinky -> index (radial +)
    normal = _unit(np.cross(lateral, middle_mcp - wrist))  # out of the back
    forward = _unit(np.cross(normal, lateral))   # square to the knuckle row
    return np.stack([lateral, forward, normal])


@dataclass
class _Segment:
    """One unit-direction match: robot keypoint a->b vs human landmark a->b."""
    robot_from: int
    robot_to: int
    human_from: int
    human_to: int
    weight: float


@dataclass
class _Pinch:
    """One DexPilot thumb-tip -> fingertip vector match."""
    robot_from: int
    robot_to: int
    human_from: int
    human_to: int


class HandRetargeter:
    """Solves HRH1 joint angles from MediaPipe world landmarks.

    Stateful on purpose: each solve warm-starts from the last one, which is
    both what makes it fast and what makes the output temporally coherent.
    """

    #: set by reset(); makes the next solve() do a multi-start
    _cold = True
    #: camera depth axis in palm-frame coords, refreshed by _human_points
    _depth_dir = None
    #: depth weight in force for the current solve -- see solve()
    _depth_weight_now = _DEPTH_WEIGHT

    def __init__(self, xacro_path=None, mirror: bool = False):
        xacro_path = xacro_path or hrh1_assembly_xacro()
        if xacro_path is None:
            raise FileNotFoundError("HRH1 assembly URDF not found")
        self.model = URDFModel.load(xacro_path)
        self.mirror = mirror

        # DOF: the 16 finger/thumb joints, in URDF names, firmware order.
        self._fw_names: List[str] = list(SOLVED_FIRMWARE_JOINTS)
        self.dof: List[str] = [HRH1_URDF_NAMES[n] for n in self._fw_names]
        self._lo = np.array([self.model.joint_by_name(n).lower for n in self.dof])
        self._hi = np.array([self.model.joint_by_name(n).upper for n in self.dof])
        # Firmware limits, mapped through the sign, intersected with the URDF's.
        # Whichever is tighter wins, so the solver can never propose a pose the
        # real hand cannot reach (T_ABD in particular is tighter in the URDF).
        for i, fw in enumerate(self._fw_names):
            spec = JOINTS_BY_NAME[fw]
            s = HRH1_URDF_SIGNS.get(fw, 1.0)
            a, b = np.radians(spec.min_deg) * s, np.radians(spec.max_deg) * s
            self._lo[i] = max(self._lo[i], min(a, b))
            self._hi[i] = min(self._hi[i], max(a, b))

        self._build_keypoints()
        self._build_terms()
        self.reset()

    # -- setup ------------------------------------------------------------

    def _build_keypoints(self) -> None:
        """Resolve every robot keypoint to an (link, point-in-link) pair and
        record which row of the FK output it lands in."""
        rest = self.model.joint_world_frames({})
        targets: List[Tuple[str, np.ndarray]] = []
        index: Dict[str, int] = {}

        def add(name: str, target: Tuple[str, np.ndarray]) -> None:
            index[name] = len(targets)
            targets.append(target)

        def tip(distal: str, hinge: str):
            """Fingertip: the point furthest from that finger's own hinge,
            over the distal phalanx AND anything welded to it (the rubber
            grips), resolved through the version-tolerant link lookup."""
            link = self.model.resolve_link(distal)
            return (link, self.model.link_extreme_point(link, rest[hinge][0]))

        add("wrist", self.model.joint_origin_in_parent("W_2"))
        for finger, (abd, mcp, ip, distal) in _FINGER_JOINTS.items():
            add(f"{finger}_abd", self.model.joint_origin_in_parent(abd))
            add(f"{finger}_mcp", self.model.joint_origin_in_parent(mcp))
            add(f"{finger}_pip", self.model.joint_origin_in_parent(ip))
            add(f"{finger}_tip", tip(distal, ip))
        t_abd, t_cmc, t_mcp, t_dp, t_distal = _THUMB_JOINTS
        add("thumb_cmc", self.model.joint_origin_in_parent(t_abd))
        add("thumb_mcp", self.model.joint_origin_in_parent(t_mcp))
        add("thumb_ip", self.model.joint_origin_in_parent(t_dp))
        add("thumb_tip", tip(t_distal, t_dp))

        self._targets = targets
        self._kp = index
        self._only_links = self.model.required_links([t[0] for t in targets])

        # Canonical robot palm frame + palm width, from the knuckle pivots at
        # the REST pose. It is a fixed triad (computed once, never updated),
        # which is what lets the solver treat it as a constant rotation -- but
        # taking it at rest from the *knuckles* rather than from the abduction
        # pivots matters: a human's knuckles are rigid with the palm, so the
        # human frame is effectively a rest-pose knuckle frame too, and the
        # two must be built from corresponding points or every direction picks
        # up a constant bias. (Measured: using the abduction pivots instead
        # costs ~5.5 deg of round-trip error, using the knuckles ~0.9 deg.)
        P0, _ = self.model.point_jacobians(targets, self.dof, {}, only_links=self._only_links)
        self._robot_R = palm_frame(
            P0[index["wrist"]], P0[index["index_mcp"]],
            P0[index["middle_mcp"]], P0[index["pinky_mcp"]],
        )
        self.robot_palm_width = float(
            np.linalg.norm(P0[index["index_mcp"]] - P0[index["pinky_mcp"]])
        )

    def _build_terms(self) -> None:
        k = self._kp
        segments: List[_Segment] = []
        for finger, (h_mcp, h_pip, h_tip) in FINGER_LANDMARKS.items():
            # short: the two bones the robot actually articulates
            segments.append(_Segment(k[f"{finger}_mcp"], k[f"{finger}_pip"],
                                     h_mcp, h_pip, _W_SEGMENT))
            segments.append(_Segment(k[f"{finger}_pip"], k[f"{finger}_tip"],
                                     h_pip, h_tip, _W_SEGMENT))
            # medium: knuckle -> tip, and long: wrist -> tip
            segments.append(_Segment(k[f"{finger}_mcp"], k[f"{finger}_tip"],
                                     h_mcp, h_tip, _W_SPAN2))
            segments.append(_Segment(k["wrist"], k[f"{finger}_tip"],
                                     WRIST, h_tip, _W_SPAN3))
        segments.append(_Segment(k["thumb_cmc"], k["thumb_mcp"],
                                 THUMB_CMC, THUMB_MCP, _W_SEGMENT))
        segments.append(_Segment(k["thumb_mcp"], k["thumb_ip"],
                                 THUMB_MCP, THUMB_IP, _W_SEGMENT))
        segments.append(_Segment(k["thumb_ip"], k["thumb_tip"],
                                 THUMB_IP, THUMB_TIP, _W_SEGMENT))
        segments.append(_Segment(k["thumb_cmc"], k["thumb_tip"],
                                 THUMB_CMC, THUMB_TIP, _W_SPAN2))
        segments.append(_Segment(k["wrist"], k["thumb_tip"],
                                 WRIST, THUMB_TIP, _W_SPAN3))
        self._segments = segments
        self._pinches = [
            _Pinch(k["thumb_tip"], k[f"{f}_tip"], THUMB_TIP, t)
            for f, (_, _, t) in FINGER_LANDMARKS.items()
        ]
        self._pinch_latched = np.zeros(len(self._pinches), dtype=bool)
        # Index arrays so the per-frame residual/Jacobian assembly is a
        # handful of vectorised numpy ops rather than a Python loop over
        # 19 terms -- this runs 5-10x per frame on the vision thread.
        self._seg_from = np.array([s.robot_from for s in segments])
        self._seg_to = np.array([s.robot_to for s in segments])
        self._seg_hfrom = np.array([s.human_from for s in segments])
        self._seg_hto = np.array([s.human_to for s in segments])
        self._seg_w = np.array([s.weight for s in segments])
        self._pin_from = np.array([p.robot_from for p in self._pinches])
        self._pin_to = np.array([p.robot_to for p in self._pinches])
        self._pin_hfrom = np.array([p.human_from for p in self._pinches])
        self._pin_hto = np.array([p.human_to for p in self._pinches])
        self._reg_block = _W_REG * np.eye(len(self.dof))
        # (abduction DOF, MCP-flexion DOF) per finger for the coupling prior,
        # looked up by firmware name so a reordering can't silently mis-pair.
        fw = self._fw_names
        self._couple_pairs = np.array(
            [(fw.index(f"{d}_MP"), fw.index(f"{d}_P")) for d in ("I", "M", "R", "P")]
        )
        #: weight of the abduction-flexion coupling prior; 0 disables it
        #: (kept as an attribute so benchmarks can ablate it).
        self.w_couple = _W_COUPLE
        #: DexPilot pinch snapping: when the operator's thumb and a fingertip
        #: come within _PROJECT_DIST, ask the robot for a near-closed gap at a
        #: raised weight so its tips actually meet. Off = follow the measured
        #: gap only. Switched from the GUI (a plain bool, read once per solve).
        self.pinch_snap = True

    def reset(self) -> None:
        """Forget the warm start (call when tracking is (re)started)."""
        self.q = np.clip(np.zeros(len(self.dof)), self._lo, self._hi)
        self._pinch_latched[:] = False
        self._cold = True

    # -- per-frame input processing ---------------------------------------

    def _human_points(self, landmarks: Sequence[Sequence[float]]) -> Tuple[np.ndarray, float]:
        """Landmarks expressed in the human's own palm frame, plus palm width."""
        L = np.asarray(landmarks, dtype=float)
        if L.shape != (21, 3):
            raise ValueError(f"expected 21 landmarks, got {L.shape}")
        R = palm_frame(L[WRIST], L[FINGER_LANDMARKS["index"][0]],
                       L[FINGER_LANDMARKS["middle"][0]], L[FINGER_LANDMARKS["pinky"][0]])
        H = (L - L[WRIST]) @ R.T
        # MediaPipe's world frame is camera-aligned, so +z is the depth axis.
        # Carry it into the palm frame for the residual weighting; it has to
        # be recomputed every frame because it swings as the hand turns.
        self._depth_dir = R @ np.array([0.0, 0.0, 1.0])
        if self.mirror:
            H[:, 0] = -H[:, 0]  # flip the lateral axis for a mirrored capture
            self._depth_dir[0] = -self._depth_dir[0]
        width = float(np.linalg.norm(L[FINGER_LANDMARKS["index"][0]]
                                     - L[FINGER_LANDMARKS["pinky"][0]]))
        return H, max(width, 1e-4)

    def _targets_for_frame(self, H: np.ndarray, human_width: float):
        """Precompute the per-frame reference geometry (independent of q)."""
        d = H[self._seg_hto] - H[self._seg_hfrom]
        n = np.linalg.norm(d, axis=1, keepdims=True)
        seg_dirs = d / np.maximum(n, _EPS)

        scale = self.robot_palm_width / human_width
        v = H[self._pin_hto] - H[self._pin_hfrom]
        dist = np.linalg.norm(v, axis=1)
        # DexPilot hysteresis: latch on when the human pinches, release only
        # once clearly open, so a pinch does not chatter at the threshold.
        self._pinch_latched[dist < _PROJECT_DIST] = True
        self._pinch_latched[dist > _ESCAPE_DIST] = False
        if not self.pinch_snap:
            # Snapping switched off (GUI toggle): the pinch terms just follow
            # the measured thumb-to-tip vectors at their baseline weight, so
            # the robot's tips are never pulled together beyond what the hand
            # shows. Also keeps a latch from surviving a switch back on.
            self._pinch_latched[:] = False
        latched = self._pinch_latched
        # Latched: ask for contact, not for the (noisy) measured small gap.
        projected = v / np.maximum(dist, _EPS)[:, None] * _PINCH_TARGET
        pinch_vecs = np.where(latched[:, None], projected, v * scale)
        pinch_w = np.where(latched, _W_PINCH_CLOSE, _W_PINCH)
        return seg_dirs, self._seg_w, pinch_vecs / self.robot_palm_width, pinch_w

    # -- residuals and Jacobian -------------------------------------------

    def _residuals(self, q: np.ndarray, seg_dirs, seg_w, pinch_vecs, pinch_w, q_prev):
        P, J = self.model.point_jacobians(
            self._targets, self.dof, dict(zip(self.dof, q)), only_links=self._only_links
        )
        R = self._robot_R
        P = P @ R.T                                  # into the robot palm frame
        J = np.einsum("ab,kbd->kad", R, J)

        # Unit-direction terms. d(unit(v))/dv = (I - uu^T)/|v|, applied to dv
        # as (dv - u (u.dv)) / |v| so no 3x3 matrices are ever formed.
        v = P[self._seg_to] - P[self._seg_from]                    # (S, 3)
        dv = J[self._seg_to] - J[self._seg_from]                   # (S, 3, D)
        n = np.maximum(np.linalg.norm(v, axis=1), _MIN_SEG)        # (S,)
        u = v / n[:, None]
        udv = np.einsum("sa,sad->sd", u, dv)                       # (S, D)
        r_seg = seg_w[:, None] * (u - seg_dirs)
        J_seg = seg_w[:, None, None] * (dv - u[:, :, None] * udv[:, None, :]) / n[:, None, None]

        # Pinch terms, in units of robot palm width so they are dimensionless.
        pv = (P[self._pin_to] - P[self._pin_from]) / self.robot_palm_width
        pdv = (J[self._pin_to] - J[self._pin_from]) / self.robot_palm_width
        r_pin = pinch_w[:, None] * (pv - pinch_vecs)
        J_pin = pinch_w[:, None, None] * pdv

        D = len(q)
        res = np.concatenate([r_seg, r_pin], axis=0)              # (T, 3)
        jac = np.concatenate([J_seg, J_pin], axis=0)              # (T, 3, D)

        # Trust the camera-depth component of every residual less than the two
        # in-plane ones (see _DEPTH_WEIGHT). W is a symmetric scaling along
        # that one axis, applied to residual and Jacobian alike so the problem
        # stays a consistent weighted least-squares.
        dw = self._depth_weight_now
        if dw != 1.0 and self._depth_dir is not None:
            d = self._depth_dir
            W = np.eye(3) - (1.0 - dw) * np.outer(d, d)
            res = res @ W.T
            jac = np.einsum("ab,tbd->tad", W, jac)

        # Huber, as IRLS: cap the influence of any single over-large term.
        # Re-derived each LM iteration from the current residual, which is
        # what makes it a proper robust fit rather than a one-off clip.
        if _HUBER_DELTA is not None:
            mag = np.linalg.norm(res, axis=1)
            w = np.sqrt(np.minimum(1.0, _HUBER_DELTA / np.maximum(mag, 1e-9)))
            res = res * w[:, None]
            jac = jac * w[:, None, None]

        r_parts = [res.reshape(-1), _W_REG * (q - q_prev)]
        J_parts = [jac.reshape(-1, D), self._reg_block]

        # Abduction-flexion coupling (see _W_COUPLE): r = w * s(q_mcp) * q_abd,
        # with s a smoothstep over the MCP flexion. Exact Jacobian, including
        # the d/dq_mcp term, so LM sees the true cost surface.
        if self.w_couple > 0.0:
            a_i, m_i = self._couple_pairs[:, 0], self._couple_pairs[:, 1]
            span = _COUPLE_FLEX_HI - _COUPLE_FLEX_LO
            x = (q[m_i] - _COUPLE_FLEX_LO) / span
            xc = np.clip(x, 0.0, 1.0)
            s = xc * xc * (3.0 - 2.0 * xc)
            ds = np.where((x > 0.0) & (x < 1.0), 6.0 * xc * (1.0 - xc) / span, 0.0)
            rows = np.arange(len(a_i))
            J_c = np.zeros((len(a_i), D))
            J_c[rows, a_i] = self.w_couple * s
            J_c[rows, m_i] = self.w_couple * ds * q[a_i]
            r_parts.append(self.w_couple * s * q[a_i])
            J_parts.append(J_c)

        return np.concatenate(r_parts), np.concatenate(J_parts, axis=0)

    # -- solve -------------------------------------------------------------

    def solve(self, landmarks: Sequence[Sequence[float]], iters: int = _LM_ITERS) -> np.ndarray:
        """MediaPipe world landmarks -> solved URDF joint angles (radians).

        Projected Levenberg-Marquardt: repeatedly solve the damped normal
        equations (J^T J + lambda*diag(J^T J)) d = -J^T r, clip the step into
        the joint box, and accept it only if the cost went down (otherwise
        raise the damping and retry). Warm-started from the previous frame.
        """
        H, width = self._human_points(landmarks)
        seg_dirs, seg_w, pinch_vecs, pinch_w = self._targets_for_frame(H, width)

        # First frame after a reset there is no warm start, so the solve can
        # settle into a wrong local minimum -- and because the next frame
        # warm-starts from it, a bad one is sticky rather than self-correcting.
        # Trying a second seed at mid-range and keeping whichever lands lower
        # cuts the worst observed cold-start error from 88 deg to 27 deg. This
        # runs once per tracking session, not per frame.
        if getattr(self, "_cold", False):
            self._cold = False
            # Depth is down-weighted during tracking because it is noisy --
            # but noisy is not the same as uninformative, and depth is the
            # ONLY thing that says whether a finger is bent toward the camera
            # or away from it. With no warm start to resolve that ambiguity
            # by continuity, discounting depth here lets the first solve pick
            # the mirrored branch and then stay there (cold-start worst case
            # measured at 92 deg vs 21 deg). So the first frame uses every
            # bit of depth it has to choose the branch, and only afterwards
            # does the weighting drop for precision.
            self._depth_weight_now = 1.0
            best_q, best_cost = None, np.inf
            for seed in (self.q.copy(), 0.5 * (self._lo + self._hi)):
                self.q = np.clip(seed, self._lo, self._hi)
                cand = self._solve_from(self.q, seg_dirs, seg_w, pinch_vecs, pinch_w,
                                        self.q.copy(), max(iters, 12))
                if self.last_cost < best_cost:
                    best_cost, best_q = self.last_cost, cand
            self._depth_weight_now = _DEPTH_WEIGHT
            self.q, self.last_cost = best_q, best_cost
            return best_q

        self._depth_weight_now = _DEPTH_WEIGHT
        return self._solve_from(self.q.copy(), seg_dirs, seg_w, pinch_vecs, pinch_w,
                                self.q.copy(), iters)

    def _solve_from(self, q, seg_dirs, seg_w, pinch_vecs, pinch_w, q_prev, iters):
        q = q.copy()
        r, Jr = self._residuals(q, seg_dirs, seg_w, pinch_vecs, pinch_w, q_prev)
        cost = float(r @ r)
        lam = _LM_LAMBDA0

        for _ in range(iters):
            JTJ = Jr.T @ Jr
            g = Jr.T @ r
            diag = np.maximum(np.diag(JTJ), 1e-9)
            try:
                step = np.linalg.solve(JTJ + lam * np.diag(diag), -g)
            except np.linalg.LinAlgError:
                break
            q_try = np.clip(q + step, self._lo, self._hi)
            r_try, J_try = self._residuals(q_try, seg_dirs, seg_w, pinch_vecs, pinch_w, q_prev)
            cost_try = float(r_try @ r_try)
            if cost_try < cost:
                q, r, Jr, cost = q_try, r_try, J_try, cost_try
                lam = max(lam / 3.0, 1e-9)
                if np.max(np.abs(step)) < 1e-5:
                    break
            else:
                lam *= 6.0
                if lam > 1e6:
                    break

        self.q = q
        self.last_cost = cost
        return q

    # -- output ------------------------------------------------------------

    def to_firmware(self, q: Optional[np.ndarray] = None) -> List[float]:
        """Solved angles -> the 19-channel firmware array in joints.py order
        (degrees). The three wrist channels are 0."""
        q = self.q if q is None else q
        by_name = {j.name: 0.0 for j in JOINTS}
        for fw, value in zip(self._fw_names, q):
            sign = HRH1_URDF_SIGNS.get(fw, 1.0)
            by_name[fw] = clamp_deg(JOINTS_BY_NAME[fw], sign * float(np.degrees(value)))
        return [by_name[j.name] for j in JOINTS]

    def landmarks_to_angles(self, landmarks: Sequence[Sequence[float]]) -> List[float]:
        """Drop-in replacement for retarget.landmarks_to_angles: 21 world
        landmarks -> 19 firmware joint angles in degrees."""
        self.solve(landmarks)
        return self.to_firmware()

    # -- introspection (for tests and for generating synthetic landmarks) ---

    def robot_keypoints(self, q: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        """Robot keypoint name -> world position at the given pose."""
        q = self.q if q is None else q
        P, _ = self.model.point_jacobians(
            self._targets, self.dof, dict(zip(self.dof, q)), only_links=self._only_links
        )
        return {name: P[i] for name, i in self._kp.items()}

    def synthetic_landmarks(self, q: Sequence[float]) -> np.ndarray:
        """The 21 landmarks a human hand in the robot's pose `q` would give.

        Used to test the solver without a camera: pose the robot, generate the
        landmarks that pose implies, solve, and check the angles come back.
        Landmarks the robot has no counterpart for (the DIP joints) are
        interpolated, since MediaPipe would report them but the solver ignores
        them.
        """
        kp = self.robot_keypoints(np.asarray(q, dtype=float))
        L = np.zeros((21, 3))
        L[WRIST] = kp["wrist"]
        L[THUMB_CMC], L[THUMB_MCP] = kp["thumb_cmc"], kp["thumb_mcp"]
        L[THUMB_IP], L[THUMB_TIP] = kp["thumb_ip"], kp["thumb_tip"]
        for finger, (h_mcp, h_pip, h_tip) in FINGER_LANDMARKS.items():
            L[h_mcp], L[h_pip], L[h_tip] = kp[f"{finger}_mcp"], kp[f"{finger}_pip"], kp[f"{finger}_tip"]
            L[h_tip - 1] = 0.5 * (L[h_pip] + L[h_tip])  # stand-in DIP
        return L


_SINGLETON: Optional[HandRetargeter] = None


def get_retargeter(mirror: bool = False) -> HandRetargeter:
    """Shared retargeter; loading the URDF takes ~100 ms so it is done once."""
    global _SINGLETON
    if _SINGLETON is None or _SINGLETON.mirror != mirror:
        _SINGLETON = HandRetargeter(mirror=mirror)
    return _SINGLETON


def landmarks_to_angles(world_landmarks: Sequence[Sequence[float]]) -> List[float]:
    """Module-level drop-in for retarget.landmarks_to_angles."""
    return get_retargeter().landmarks_to_angles(world_landmarks)


# ---------------------------------------------------------------------------
# Self-test. `python -m hrh.ik_retarget` -- needs no camera, display or GPU.
#
# The central check is a round trip: pose the robot at a known q, generate the
# landmarks that pose implies via the model's own FK, solve, and require the
# angles back. That is a genuine test rather than a tautology because the
# forward path (link transforms -> keypoints) and the inverse path (residuals,
# analytic Jacobian, palm frames, LM, firmware sign mapping) share no code --
# a frame error, a sign flip or a bad index breaks it. It is also what caught
# the palm-frame bug during development: building the robot frame from the
# abduction pivots instead of the knuckles cost 5.5 deg of round-trip error.
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import time

    r = HandRetargeter()
    rng = np.random.default_rng(0)
    lo, hi = r._lo, r._hi

    def sweep(n, noise_mm=0.0, iters=_LM_ITERS, warm=False):
        errs = []
        if warm:
            r.reset()
            q_prev = lo + (hi - lo) * 0.3
        for _ in range(n):
            if warm:
                q_true = np.clip(q_prev + rng.normal(0, np.radians(2.5), len(lo)), lo, hi)
                q_prev = q_true
            else:
                q_true = lo + (hi - lo) * rng.uniform(0.05, 0.95, len(lo))
                r.reset()
            L = r.synthetic_landmarks(q_true)
            if noise_mm:
                L = L + rng.normal(0, noise_mm / 1000.0, L.shape) * np.array([1, 1, 3])
            errs.append(np.degrees(np.abs(r.solve(L, iters=iters) - q_true)))
        return np.array(errs)

    print(f"{len(r.dof)} DOF, {len(r._segments)} direction terms, "
          f"{len(r._pinches)} pinch terms, {len(r._only_links)}/{len(r.model.links)} links walked")

    e = sweep(30, warm=True)
    print(f"warm round trip (clean)   mean {e.mean():6.3f} deg   max {e.max():6.2f}")
    assert e.mean() < 2.0, "warm round trip should be well under 2 deg"

    e = sweep(20)
    print(f"cold round trip (clean)   mean {e.mean():6.3f} deg   max {e.max():6.2f}")
    assert e.mean() < 6.0, "cold round trip regressed"
    # Guards the depth-ambiguity trap: down-weighting depth during the COLD
    # solve lets the first frame settle on the mirrored branch (finger bent
    # away rather than toward the camera), and because the next frame warm-
    # starts from it, that error is sticky. This caught exactly that, at a
    # 92 deg worst case, when the depth weighting was first added.
    assert e.max() < 45.0, "cold round trip worst case regressed -- depth branch?"

    e = sweep(20, noise_mm=2.0)
    print(f"cold round trip (2mm)     mean {e.mean():6.3f} deg   max {e.max():6.2f}")

    # Open hand must read ~zero everywhere; a fist must read strongly flexed
    # with the correct SIGN (the thing that kept being wrong by hand).
    zero = np.clip(np.zeros(len(lo)), lo, hi)
    r.reset(); r.solve(r.synthetic_landmarks(zero), iters=12)
    fw_open = np.array(r.to_firmware())
    print(f"open hand: max |angle| = {np.abs(fw_open).max():.2f} deg")
    assert np.abs(fw_open).max() < 3.0, "an open hand must retarget to ~zero"

    fist = zero.copy()
    for i, name in enumerate(r.dof):
        if name.startswith("T"):
            continue
        if name.endswith("_MCP"):
            fist[i] = np.radians(85)
        elif name.endswith("_IP"):
            fist[i] = np.radians(110)
    r.reset(); r.solve(r.synthetic_landmarks(np.clip(fist, lo, hi)), iters=12)
    fw_fist = {n: v for n, v in zip([j.name for j in JOINTS], r.to_firmware())}
    print("fist:      " + "  ".join(f"{k}={fw_fist[k]:+6.1f}" for k in ("I_P", "I_IP", "M_P", "M_IP")))
    assert fw_fist["I_P"] > 70 and fw_fist["I_IP"] > 90, "a fist must flex POSITIVE"

    L = r.synthetic_landmarks(lo + (hi - lo) * 0.4)
    r.solve(L)
    t0 = time.perf_counter()
    for _ in range(200):
        r.solve(L)
    dt = (time.perf_counter() - t0) / 200
    print(f"solve: {dt * 1000:.2f} ms/frame -> {1 / dt:.0f} Hz")
    assert dt < 0.020, "must comfortably beat a 30 Hz camera"
    print("OK")
