"""Canned gesture presets, interpolated PC-side (brief section 4/5).

Each entry lists the 16 finger+thumb angles in degrees, in firmware channel
order (see joints.JOINTS): P_MP, P_P, P_IP, R_MP, R_P, R_IP, M_MP, M_P,
M_IP, I_MP, I_P, I_IP, T_CMC, T_ABD, T_MCP, T_IP.

**v2:** the hand now has three wrist joints (W_1/W_2/W_3, indices 16-18).
The gesture rows below stay finger+thumb only for readability; `_clamped`
pads the wrist to neutral (0 deg) so every canned pose rests the wrist. Add
explicit wrist values to a row only if a gesture ever needs a posed wrist.

>>> PLACEHOLDER VALUES <<<
These are reasonable starting guesses chosen to be visually representative
and safely inside each joint's range -- not measured from the real hand.
Adam is providing exact tuned angles later; until then, edit the arrays
below directly (each row is commented with the joint name for editing).
"""

from __future__ import annotations

from .joints import JOINTS, clamp_deg


def _clamped(angles: list[float]) -> list[float]:
    # Pad any joints not given (the v2 wrist joints appended at the end)
    # with their neutral 0 deg, then clamp each to its range. This keeps a
    # gesture row as just the 16 finger+thumb values while still producing
    # a full 19-value pose.
    padded = list(angles) + [0.0] * (len(JOINTS) - len(angles))
    return [clamp_deg(joint, deg) for joint, deg in zip(JOINTS, padded)]


# fmt: off
GESTURES: dict[str, list[float]] = {
    # All joints at 0: fully open, straight fingers (per the brief, 0 deg
    # means fully extended for the flexion joints and neutral for
    # abduction / thumb column joints).
    "zero": _clamped([
        0, 0, 0,      # P_MP, P_P, P_IP
        0, 0, 0,      # R_MP, R_P, R_IP
        0, 0, 0,      # M_MP, M_P, M_IP
        0, 0, 0,      # I_MP, I_P, I_IP
        0, 0, 0, 0,   # T_CMC, T_ABD, T_MCP, T_IP
    ]),

    # Relaxed, slightly curled -- resting hand pose. Thumb angles tuned by
    # Adam against the MANO 3D view (manual sliders); finger angles are
    # still the original placeholder.
    "neutral": _clamped([
        0, 15, 15,
        0, 15, 15,
        0, 15, 15,
        0, 15, 15,
        62.8, 3.5, -12.5, 10,
    ]),

    # Full grip / fist, e.g. for grasping an object. Finger flexion set by
    # Adam to each joint's full range: X_P 90 deg and X_IP 116 deg, which are
    # exactly the v3 limits in joints.py, so nothing here is clamped. Thumb
    # is still the original placeholder.
    "fist": _clamped([
        0, 90, 116,   # P_MP, P_P, P_IP
        0, 90, 116,   # R_MP, R_P, R_IP
        0, 90, 116,   # M_MP, M_P, M_IP
        0, 90, 116,   # I_MP, I_P, I_IP
        35, 10, 40, 60,
    ]),

    # Peace sign: index + middle extended, ring + pinky curled, thumb
    # tucked over the curled fingers. Thumb angles tuned by Adam against
    # the MANO 3D view (manual sliders); finger angles are still the
    # original placeholder. See gui.py's THUMB_DELAYED_GESTURES -- the
    # thumb here needs ring/pinky to curl out of the way first or it has
    # nowhere clear to tuck into.
    "peace": _clamped([
        5, 80, 100,           # pinky curled
        -5, 80, 100,          # ring curled
        -8, 0, 0,             # middle extended, slight spread
        8, 0, 0,              # index extended, slight spread
        -11, 65, 27.8, 36.7,  # thumb tucked
    ]),

    # "Rock on": index + pinky extended, middle + ring curled, thumb
    # tucked or extended across the curled fingers.
    "rock": _clamped([
        0, 0, 0,        # pinky extended
        0, 80, 100,     # ring curled
        0, 80, 100,     # middle curled
        0, 0, 0,        # index extended
        20, 0, 30, 30,  # thumb tucked
    ]),
}
# fmt: on

GESTURE_NAMES: list[str] = list(GESTURES.keys())
