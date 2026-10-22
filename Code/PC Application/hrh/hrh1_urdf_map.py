"""Firmware joint names/signs <-> HRH1 URDF joint names, and where the URDF is.

Split out of hand_view3d_urdf.py so it can be imported without pulling in
pyvista/VTK: the retargeting optimiser (ik_retarget.py) runs on the vision
worker thread and needs this mapping, but must not drag a renderer along.
The 3D view imports the same names from here, so there is one copy.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

# --- HRH1: firmware joint -> this hand's URDF joint name ----------------
# The Fusion export names some joints slightly differently from the
# firmware (joints.py): finger abduction is `*_ABD` (except the index,
# exported as `I_MP`), finger proximal flexion is `*_MCP`, and the thumb
# tip joint is `T_DP`.
HRH1_URDF_NAMES: Dict[str, str] = {
    "P_MP": "P_ABD", "P_P": "P_MCP", "P_IP": "P_IP",
    "R_MP": "R_ABD", "R_P": "R_MCP", "R_IP": "R_IP",
    "M_MP": "M_ABD", "M_P": "M_MCP", "M_IP": "M_IP",
    "I_MP": "I_MP",  "I_P": "I_MCP", "I_IP": "I_IP",
    "T_CMC": "T_CMC", "T_ABD": "T_ABD", "T_MCP": "T_MCP", "T_IP": "T_DP",
    "W_1": "W_1", "W_2": "W_2", "W_3": "W_3",
}

# Per-joint direction sign (firmware + vs URDF +). urdf_deg = sign * firmware_deg,
# and because sign is +-1 the inverse is the same multiply.
#
# The three thumb joints are -1: the Fusion export gave them limits mirrored
# vs firmware (URDF T_MCP (-95,+65) vs firmware (-65,+95); T_CMC (-82,+37) vs
# (-37.5,+82); T_DP (-98,+63) vs T_IP (-63,+98.2)), i.e. the URDF +axis is
# the firmware -axis. So -1 maps firmware -> URDF correctly: firmware +95
# drives to URDF -95 (forward), firmware -65 to URDF +65 (back).
# (T_ABD is NOT mirrored -- URDF (-32,+70) vs firmware (-65,+65) -- so it
# stays +1; note its usable negative travel is smaller than firmware allows.)
HRH1_URDF_SIGNS: Dict[str, float] = {
    "T_CMC": -1.0,
    "T_MCP": -1.0,
    "T_IP": -1.0,   # -> URDF T_DP
}

# Firmware joints the retargeter actually solves for. The three wrist joints
# are excluded: a single hand's landmarks carry no forearm reference, so they
# are unobservable and stay at 0 (see README / PoseLandmarker note).
SOLVED_FIRMWARE_JOINTS = (
    "I_MP", "I_P", "I_IP",
    "M_MP", "M_P", "M_IP",
    "R_MP", "R_P", "R_IP",
    "P_MP", "P_P", "P_IP",
    "T_ABD", "T_CMC", "T_MCP", "T_IP",
)

_ASSEMBLY_XACRO = (
    Path(__file__).resolve().parent.parent
    / "URDFs" / "HRH1_URDF_Assembly_description" / "urdf" / "HRH1_URDF_Assembly.xacro"
)


def hrh1_assembly_xacro() -> Optional[Path]:
    """Path to the full-hand URDF export, or None if it isn't present."""
    return _ASSEMBLY_XACRO if _ASSEMBLY_XACRO.exists() else None


def hrh1_firmware_map() -> Dict[int, tuple]:
    """firmware joint index (joints.py order) -> (urdf joint name, sign)."""
    from .joints import JOINTS
    out: Dict[int, tuple] = {}
    for j in JOINTS:
        u = HRH1_URDF_NAMES.get(j.name)
        if u is not None:
            out[j.index] = (u, HRH1_URDF_SIGNS.get(j.name, 1.0))
    return out
