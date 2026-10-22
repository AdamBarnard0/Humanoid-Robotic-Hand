"""Palm orientation -> HRH1 wrist joints (W_1 / W_2 / W_3), against the URDF.

Why this exists
---------------
The finger retargeter (ik_retarget.py) works entirely in the palm's own frame,
so it deliberately throws away how the hand is *oriented*. That is correct for
the fingers, but it left the robot's three wrist joints permanently at 0 under
hand tracking: a single hand's landmarks carry no forearm reference, so the
wrist angle itself is unobservable.

What IS observable is how the palm rotates relative to the camera. If the
operator's forearm stays roughly where it was -- the normal case when sitting
in front of a webcam -- that rotation is (to first order) wrist + forearm
rotation, which is exactly what HRH1's roll / pitch / yaw wrist is for. So this
module asks the operator for one reference ("neutral") pose, and from then on
drives the robot wrist so that its palm rotates, relative to the robot's own
neutral, the same way the human palm has rotated relative to the captured one.

How
---
Both sides use the SAME palm-frame construction as the finger solver
(ik_retarget.palm_frame: lateral / forward / normal from wrist + knuckles), so
"the same rotation" is well defined without any sign or axis guesswork:

    human:  Delta   = F_h0 @ F_h(t).T      (rotation, in neutral-palm coords)
    robot:  R_w(q)  = M_r0 @ Delta @ M_r0.T  is what the wrist must produce

where F are frames with axes as ROWS (as palm_frame returns them) and
M_r0 = F_r0.T. R_w(q) is the wrist chain's rotation in the robot base frame,
written as a product of exponentials over the joint axes read straight out of
the URDF at q = 0 -- so the chain order (W_1 -> W_3 -> W_2 in this export),
axis directions and signs all come from the CAD, not from constants here.
The self-test checks that product against the full URDF forward kinematics.

The 3-DOF solve is a damped Gauss-Newton on SO(3) with the joint limits
enforced, warm-started from the previous frame; it converges in 2-4 cheap
iterations (3x3 matrices only, no tree walk).

Limits of the idea (documented rather than hidden): the camera sees the hand,
not the forearm, so moving the whole arm also turns the palm and will move the
robot wrist; and W_3 (yaw) only has +-16.5 deg of travel, so larger side-to-side
swings saturate. It is off by default and has to be enabled from the GUI.
"""
from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np

from .joints import JOINTS_BY_NAME, clamp_deg

#: Robot wrist joints (firmware names == URDF names for these).
WRIST_JOINTS = ("W_1", "W_2", "W_3")

# Frames averaged when capturing the neutral pose. A single MediaPipe frame
# carries a degree or two of orientation noise; ~0.3 s of frames removes most
# of it and is short enough that the operator does not notice the wait.
NEUTRAL_FRAMES = 10

# The neutral is only accepted from a hand that is holding still: if a new
# frame's palm orientation is more than this from the mean of the frames
# captured so far, the capture starts over. Otherwise switching wrist tracking
# on while the hand is still moving into view would capture a pose in
# mid-swing. (Compared against the running mean rather than the previous
# frame, so ordinary landmark noise -- a degree or two per frame -- does not
# keep restarting it.)
NEUTRAL_STILL_DEG = 4.0

# Pull toward the previous solution (per radian). Keeps the solve on one
# continuous branch and damps it when the target is out of reach, without
# measurably slowing it when the target is reachable.
_W_REG = 0.05
_ITERS = 6


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rot_exp(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation matrix for `angle` radians about unit `axis` (Rodrigues)."""
    K = _skew(axis)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def rot_log(R: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle) of a rotation matrix. Robust near 0 and
    near pi (where the naive formula divides by ~0)."""
    cos_t = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    theta = float(np.arccos(cos_t))
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if theta < 1e-6:
        return 0.5 * w
    if np.pi - theta < 1e-4:
        # Near pi: take the axis from the symmetric part instead.
        B = 0.5 * (R + np.eye(3))
        axis = np.sqrt(np.clip(np.diag(B), 0.0, None))
        k = int(np.argmax(axis))
        axis = B[k] / max(axis[k], 1e-12)
        axis /= np.linalg.norm(axis)
        return axis * theta
    return w * (theta / (2.0 * np.sin(theta)))


def project_to_rotation(M: np.ndarray) -> np.ndarray:
    """Nearest rotation matrix (in Frobenius norm) to M -- used to average
    frames: the chordal mean of rotations is the projection of their sum."""
    U, _s, Vt = np.linalg.svd(M)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt


class WristRetargeter:
    """Solves W_1/W_2/W_3 from palm-frame orientation, relative to a neutral.

    `model` is the loaded URDFModel and `robot_palm_frame` the robot's rest
    palm frame (rows lateral/forward/normal, robot base coords) -- pass the
    finger retargeter's own so both use one definition of "the palm".
    """

    def __init__(self, model, robot_palm_frame: np.ndarray):
        self.model = model
        for j in WRIST_JOINTS:
            if model.joint_by_name(j) is None:
                raise ValueError(f"URDF has no wrist joint {j!r}")
        # Order base -> palm by how many movable joints sit above each one's
        # child link, rather than trusting any particular list order. The
        # palm is the child link of the last wrist joint in that chain.
        depth = {j: len(model.ancestor_movable_joints(model.joint_by_name(j).child))
                 for j in WRIST_JOINTS}
        self.chain: List[str] = sorted(WRIST_JOINTS, key=depth.__getitem__)
        palm = model.joint_by_name(self.chain[-1]).child
        above = model.ancestor_movable_joints(palm)
        if [j for j in above if j in WRIST_JOINTS] != self.chain:
            raise ValueError(f"wrist joints {self.chain} are not one serial chain to {palm!r}")
        frames = model.joint_world_frames({})
        self._axes = [np.asarray(frames[j][1], float) / np.linalg.norm(frames[j][1]) for j in self.chain]
        lo, hi = [], []
        for j in self.chain:
            uj, spec = model.joint_by_name(j), JOINTS_BY_NAME[j]
            # Tighter of URDF and firmware limits (wrist signs are +1).
            lo.append(max(uj.lower, np.radians(spec.min_deg)))
            hi.append(min(uj.upper, np.radians(spec.max_deg)))
        self._lo, self._hi = np.array(lo), np.array(hi)
        self._palm_link = palm
        self._M_r0 = np.asarray(robot_palm_frame, float).T  # columns = palm axes
        self.q = np.zeros(3)
        self.last_error_deg = 0.0
        self._neutral: Optional[np.ndarray] = None   # F_h0 (rows)
        self._capture: Optional[List[np.ndarray]] = None

    # -- neutral pose -------------------------------------------------------

    @property
    def has_neutral(self) -> bool:
        return self._neutral is not None

    @property
    def capturing(self) -> bool:
        return self._capture is not None

    def request_neutral(self) -> None:
        """Capture the next NEUTRAL_FRAMES palm frames as the new neutral."""
        self._capture = []

    def clear_neutral(self) -> None:
        self._neutral = None
        self._capture = None
        self.q = np.zeros(3)

    def set_neutral(self, human_palm_frame: np.ndarray) -> None:
        self._neutral = project_to_rotation(np.asarray(human_palm_frame, float))
        self._capture = None

    # -- kinematics ---------------------------------------------------------

    def chain_rotation(self, q: Sequence[float]) -> np.ndarray:
        """Wrist-chain rotation in the base frame (product of exponentials)."""
        R = np.eye(3)
        for axis, angle in zip(self._axes, q):
            R = R @ rot_exp(axis, float(angle))
        return R

    def _spatial_jacobian(self, q: np.ndarray) -> np.ndarray:
        J = np.zeros((3, 3))
        R = np.eye(3)
        for i, (axis, angle) in enumerate(zip(self._axes, q)):
            J[:, i] = R @ axis
            R = R @ rot_exp(axis, float(angle))
        return J

    def target_rotation(self, human_palm_frame: np.ndarray) -> np.ndarray:
        """R_w the robot wrist must produce for this human palm frame."""
        delta = self._neutral @ np.asarray(human_palm_frame, float).T
        return self._M_r0 @ delta @ self._M_r0.T

    def solve_rotation(self, R_target: np.ndarray, iters: int = _ITERS) -> np.ndarray:
        """Joint angles (rad, chain order) whose rotation best matches R_target."""
        q = self.q.copy()
        q_prev = self.q.copy()
        for _ in range(iters):
            e = rot_log(R_target @ self.chain_rotation(q).T)   # spatial error
            J = self._spatial_jacobian(q)
            A = J.T @ J + (_W_REG ** 2) * np.eye(3)
            g = J.T @ e - (_W_REG ** 2) * (q - q_prev)
            step = np.linalg.solve(A, g)
            q = np.clip(q + step, self._lo, self._hi)
            if np.max(np.abs(step)) < 1e-6:
                break
        self.q = q
        self.last_error_deg = float(np.degrees(np.linalg.norm(
            rot_log(R_target @ self.chain_rotation(q).T))))
        return q

    # -- per frame ----------------------------------------------------------

    def update(self, human_palm_frame: np.ndarray) -> Optional[dict]:
        """Feed one frame's human palm frame (rows, camera coords).

        Returns {firmware joint name: degrees}, or None while no neutral has
        been captured yet (the caller should then leave the wrist alone).
        """
        F = np.asarray(human_palm_frame, float)
        capture = self._capture   # one read: request_neutral() may swap it
        if capture is not None:
            if capture:
                mean = project_to_rotation(np.sum(capture, axis=0))
                if np.degrees(np.linalg.norm(rot_log(mean @ F.T))) > NEUTRAL_STILL_DEG:
                    capture.clear()   # moving: start over
            capture.append(F)
            if len(capture) >= NEUTRAL_FRAMES:
                self.set_neutral(np.sum(capture, axis=0))
                self.q = np.zeros(3)
            return None if self._neutral is None else self._to_firmware(self.q)
        if self._neutral is None:
            return None
        q = self.solve_rotation(self.target_rotation(F))
        return self._to_firmware(q)

    def _to_firmware(self, q: np.ndarray) -> dict:
        return {
            j: clamp_deg(JOINTS_BY_NAME[j], float(np.degrees(v)))
            for j, v in zip(self.chain, q)
        }


# ---------------------------------------------------------------------------
# Self-test: `python -m hrh.wrist_retarget` (no camera/display needed).
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    from .ik_retarget import HandRetargeter, palm_frame, FINGER_LANDMARKS, WRIST

    hr = HandRetargeter()
    wr = WristRetargeter(hr.model, hr._robot_R)
    print("wrist chain (base -> palm):", wr.chain)
    rng = np.random.default_rng(1)

    # 1. The product-of-exponentials rotation must equal the URDF's own FK.
    worst = 0.0
    palm = wr._palm_link
    R0 = hr.model.link_world_transforms({})[palm][:3, :3]
    for _ in range(50):
        q = rng.uniform(wr._lo, wr._hi)
        fk = hr.model.link_world_transforms(dict(zip(wr.chain, q)))[palm][:3, :3]
        worst = max(worst, np.abs(fk @ R0.T - wr.chain_rotation(q)).max())
    print(f"PoE vs URDF FK: max |dR| = {worst:.2e}")
    assert worst < 1e-9

    # 2. Round trip through landmarks. Pose the robot wrist, generate the palm
    #    landmarks the whole robot would show a camera (rotated rigidly by the
    #    wrist, then by an arbitrary camera orientation), and require the
    #    solver to recover the wrist angles from the palm frame alone.
    def palm_frame_of(L):
        return palm_frame(L[WRIST], L[FINGER_LANDMARKS["index"][0]],
                          L[FINGER_LANDMARKS["middle"][0]], L[FINGER_LANDMARKS["pinky"][0]])

    L0 = hr.synthetic_landmarks(np.clip(np.zeros(len(hr.dof)), hr._lo, hr._hi))
    cam = rot_exp(np.array([0.3, -0.8, 0.5]) / np.linalg.norm([0.3, -0.8, 0.5]), 1.1)

    def seen(q_w):
        R = wr.chain_rotation(q_w)
        return (L0 @ R.T) @ cam.T      # rigid rotation, then camera view

    wr.set_neutral(palm_frame_of(seen(np.zeros(3))))
    errs = []
    q_true = np.zeros(3)
    for _ in range(200):   # a continuous random walk, like a real session
        q_true = np.clip(q_true + rng.normal(0, np.radians(4), 3), wr._lo * 0.95, wr._hi * 0.95)
        out = wr.update(palm_frame_of(seen(q_true)))
        got = np.radians([out[j] for j in wr.chain])
        errs.append(np.degrees(np.abs(got - q_true)).max())
    errs = np.array(errs)
    print(f"wrist round trip: mean {errs.mean():.4f} deg, max {errs.max():.4f} deg")
    assert errs.max() < 0.05

    # 3. Neutral capture averages frames and holds the wrist at zero meanwhile.
    wr.clear_neutral(); wr.request_neutral()
    for k in range(NEUTRAL_FRAMES):
        # ~0.7 mm i.i.d. landmark noise gives ~1.3 deg of palm-orientation
        # jitter on this (small) palm -- about what a still hand shows on
        # real MediaPipe output (quietest 10-frame windows of the bench clip
        # stay within 1.55 deg of their mean).
        noisy = palm_frame_of(seen(np.zeros(3)) + rng.normal(0, 0.0007, L0.shape))
        out = wr.update(noisy)
    assert wr.has_neutral and not wr.capturing

    want = {"W_1": 30.0, "W_2": -20.0, "W_3": 10.0}
    out = wr.update(palm_frame_of(seen(np.radians([want[j] for j in wr.chain]))))
    print("after noisy neutral capture, pose", want, "->", {k: round(v, 1) for k, v in out.items()})
    assert all(abs(out[j] - want[j]) < 2.0 for j in want)

    # 3b. A hand that is still turning is not accepted as the neutral.
    wr.clear_neutral(); wr.request_neutral()
    for k in range(3 * NEUTRAL_FRAMES):
        wr.update(palm_frame_of(seen(np.radians([3.0 * k, 0.0, 0.0]))))   # 3 deg/frame roll
    assert not wr.has_neutral and wr.capturing, "captured a neutral from a moving hand"
    for k in range(NEUTRAL_FRAMES):
        wr.update(palm_frame_of(seen(np.zeros(3))))
    assert wr.has_neutral
    print("moving hand rejected as neutral; still hand accepted")

    # 4. Out-of-range targets saturate at the limit instead of misbehaving.
    want = {"W_1": 0.0, "W_2": 0.0, "W_3": 60.0}   # W_3 (yaw) travel is +-16.5
    out = wr.update(palm_frame_of(seen(np.radians([want[j] for j in wr.chain]))))
    print("yaw target 60 deg (limit 16.5) ->", {k: round(v, 1) for k, v in out.items()})
    assert abs(out["W_3"]) <= 16.5 + 1e-6 and abs(out["W_3"]) > 15.0

    import time
    F = palm_frame_of(seen(np.radians([10, 5, -3])))
    t0 = time.perf_counter()
    for _ in range(500):
        wr.update(F)
    print(f"solve: {(time.perf_counter() - t0) / 500 * 1000:.3f} ms/frame")
    print("OK")
