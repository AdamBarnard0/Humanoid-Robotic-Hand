"""HRH1 joint table -- HRH1 Serial Protocol v3 (ranges per PROTOCOL.md section 6).

Order is fixed by the firmware's JointId enum and is used positionally
everywhere in the wire protocol (T/S messages carry 19 values in this
order). Do not reorder -- see PROTOCOL.md section 3.

**v2 change (was 16 joints):** the hand now has a 3-DOF wrist, so frames
carry **19 joints / 38 numbers**, not 16 / 32. Three wrist joints (`W_1`,
`W_2`, `W_3`) are appended at indices 16-18. Several ranges of motion also
changed -- see the per-joint notes below and PROTOCOL.md section 11.

Zero convention (from the protocol's "Zero reference" column):
    "extended" -> 0 deg is the end of travel (also fully extended); the
                  joint only moves one way from there. The finger _P / _IP
                  flexion joints.
    "straight" -> 0 deg is the finger *physically straight*, a reference
                  point inside the range; the joint moves both ways from
                  it. The four abduction (_MP) joints and T_IP (negative =
                  over-extended past straight).
    "centre"   -> 0 deg is the neutral rest position, inside the range.
                  T_CMC / T_ABD / T_MCP and the three wrist joints.

Speed: each joint has its own maximum slew rate (deg/s). v1's single global
400 deg/s no longer applies -- the wrist axes swing the mass of the whole
hand and are capped lower (PROTOCOL.md section 4). A negative commanded
speed means "full speed for that joint", which is that joint's own ceiling.
"""

from dataclasses import dataclass

# Per-joint slew-rate ceilings (deg/s), PROTOCOL.md section 4.
MAX_SPEED = 400.0          # PROTO_MAX_SPEED -- fingers and thumb
WRIST_SPEED_SCALE = 0.6    # WRIST_SPEED_W_n in Config.h section 4c
WRIST_MAX_SPEED = MAX_SPEED * WRIST_SPEED_SCALE  # 240.0 -- wrist axes
MIN_SPEED = 1.0            # PROTO_MIN_SPEED


@dataclass(frozen=True)
class JointSpec:
    index: int          # position in the 19-value T/S arrays (0-18)
    name: str           # firmware joint name, e.g. "P_MP"
    zero_reference: str  # "extended", "straight", or "centre"
    min_deg: float
    max_deg: float
    max_speed_deg_s: float = MAX_SPEED  # per-joint slew ceiling

    @property
    def default_deg(self) -> float:
        """Safe startup position: 0 deg for every joint on this hand (an
        extended/straight/neutral pose depending on the joint)."""
        return 0.0


# Prefix: P=Pinky, R=Ring, M=Middle, I=Index, T=Thumb, W=Wrist.
# MP = abduction (side-to-side), P = proximal flexion,
# IP = fused distal+intermediate flexion (mechanically fused on this hand).
#
# NOTE (v2): the four abduction joints are NOT symmetric and NOT identical
# to each other -- the pinky in particular travels only +10 deg but -35
# (PROTOCOL.md section 3). T_IP is now centre-referenced (-63..+98.2), not
# the v1 one-directional 0..98.2. The wrist ROMs are PLACEHOLDERS pending
# measurement of the real mechanism.
JOINTS: list[JointSpec] = [
    # Limits from PROTOCOL.md v3 section 6 (= CAD/URDF export, firmware sign
    # convention). The firmware clamps to these and reports clamping per joint.
    JointSpec(0, "P_MP", "straight", -35.0, 3.0),
    JointSpec(1, "P_P", "extended", 0.0, 90.0),
    JointSpec(2, "P_IP", "extended", 0.0, 116.0),
    JointSpec(3, "R_MP", "straight", -24.0, 20.0),
    JointSpec(4, "R_P", "extended", 0.0, 90.0),
    JointSpec(5, "R_IP", "extended", 0.0, 116.0),
    JointSpec(6, "M_MP", "straight", -24.0, 24.0),
    JointSpec(7, "M_P", "extended", 0.0, 90.0),
    JointSpec(8, "M_IP", "extended", 0.0, 116.0),
    JointSpec(9, "I_MP", "straight", -20.0, 24.0),
    JointSpec(10, "I_P", "extended", 0.0, 90.0),
    JointSpec(11, "I_IP", "extended", 0.0, 116.0),
    JointSpec(12, "T_CMC", "centre", -37.0, 82.0),
    JointSpec(13, "T_ABD", "centre", -32.0, 70.0),
    JointSpec(14, "T_MCP", "centre", -65.0, 95.0),
    JointSpec(15, "T_IP", "straight", -63.0, 98.0),
    JointSpec(16, "W_1", "centre", -90.0, 90.0, WRIST_MAX_SPEED),   # roll
    JointSpec(17, "W_2", "centre", -55.0, 55.0, WRIST_MAX_SPEED),   # pitch
    JointSpec(18, "W_3", "centre", -16.5, 16.5, WRIST_MAX_SPEED),   # yaw
]

NUM_JOINTS = len(JOINTS)

assert [j.index for j in JOINTS] == list(range(NUM_JOINTS)), (
    "JOINTS must be listed in firmware channel order"
)

JOINTS_BY_NAME: dict[str, JointSpec] = {j.name: j for j in JOINTS}

# The three wrist joints (indices 16-18) are different hardware -- driven
# straight off ESP32 GPIOs, not homed by `C homeall`, and their INA219 mA
# is a shared whole-rail total (all three report the same number). See
# PROTOCOL.md section 3.
WRIST_JOINT_NAMES = frozenset({"W_1", "W_2", "W_3"})


def clamp_deg(joint: JointSpec, deg: float) -> float:
    """Clamp a commanded angle to the joint's mechanical range."""
    return max(joint.min_deg, min(joint.max_deg, deg))


def clamp_speed(joint: JointSpec, speed: float) -> float:
    """Clamp a commanded slew rate to this joint's range, mirroring the
    firmware (PROTOCOL.md section 4). A negative value is the "full speed,
    don't care" sentinel and is passed through unchanged. A positive value
    is bounded to [MIN_SPEED, joint.max_speed_deg_s]."""
    if speed < 0:
        return speed  # "don't care" sentinel -> firmware uses the joint ceiling
    return max(MIN_SPEED, min(joint.max_speed_deg_s, speed))
