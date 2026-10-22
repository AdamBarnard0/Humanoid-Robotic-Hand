"""HRH1 PC control app.

Single-window layout: control panel on the left (connect, presets,
manual-sliders toggle, hand-tracking toggle), an auxiliary panel in the
middle that shows *either* the manual slider array *or* the hand-tracking
webcam preview (never both, and hidden entirely when neither is active),
and a live 3D hand view on the right. The hand view has its own
Measured/Target toggle above it: *measured* shows the link's actual S-frame
feedback (default), *target* shows self.commanded -- what's currently
being sent, which also works with no link connected. Everything lives in
one QMainWindow -- no popup dialogs, no extra windows.

Streaming, presets, and MediaPipe hand tracking all funnel into one
`self.commanded` array of 16 angles that the 50 Hz stream timer sends as
ordinary T targets -- matching PROTOCOL.md's "latest-value-wins, no
queue" design. Whichever source last touched `self.commanded` (a slider
drag, a preset jump, or a tracked hand) wins; each one takes over from
whatever came before. Presets and manual moves are sent at speed -1
(firmware default) and left for the hand's own slew limiter to move
smoothly -- there's no PC-side interpolation.
"""

from __future__ import annotations

import colorsys
import sys
import time
from collections import deque

import pyqtgraph as pg
from PySide6.QtCore import QObject, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QImage, QKeySequence, QPixmap, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .gestures import GESTURE_NAMES, GESTURES
from .joints import JOINTS, NUM_JOINTS, JointSpec
from .theme import APP_STYLESHEET
from .widgets import MouseGlyph, ProgressButton, StatusLED, ToggleSwitch, VerticalTab

try:
    from .hand_view3d_urdf import URDFHandView, HRH1_URDF_JOINT_MAP, hrh1_assembly_xacro
except Exception:  # pragma: no cover
    URDFHandView = None
    HRH1_URDF_JOINT_MAP = {}
    def hrh1_assembly_xacro():
        return None
from .link import LINK_STALE_TIMEOUT_S, STREAM_HZ as LINK_STREAM_HZ, SerialLink, StatusMessage, list_available_ports
from .camera import DEFAULT_RESOLUTION, RESOLUTIONS
from .vision import VisionWorker

# Protocol v3 is lockstep and PC-clocked: one T every 33.3 ms, whether or not
# the target changed (an unchanged T is the heartbeat), each answered by one S.
STREAM_HZ = LINK_STREAM_HZ  # 30
STREAM_INTERVAL_MS = round(1000 / STREAM_HZ)
STATUS_POLL_INTERVAL_MS = 50  # measured-value / link-health refresh

# 3D-view redraw rate, on its own timer separate from telemetry/tracking.
# Originally 120 ms (~8 Hz) because the view rebuilt/re-uploaded heavy
# geometry every frame -- that made motion look jumpy. The URDF view now
# only re-transforms the links that actually moved (and skips the ~1.1M
# triangles of bearing meshes + decimates the rest), so a redraw is cheap
# and we can run it at a smooth rate. set_pose no-ops when the pose is
# unchanged, so a still hand costs nothing regardless of this interval.
HAND_VIEW_INTERVAL_MS = 33  # ~30 Hz

SLIDER_SCALE = 10  # slider steps of 0.1 deg, matching wire precision

MEASURED_BAR_HEIGHT = 6
MEASURED_BAR_STYLE = """
    QProgressBar { background-color: #3a3a3a; border: none; border-radius: 2px; }
    QProgressBar::chunk { background-color: #e0c020; border-radius: 2px; }
"""

# The control panel lays its buttons out two per row, so it is wider and
# about half as tall as the old single-column stack -- which is what got it
# to fit without scrolling. The hand is a tall narrow assembly, so the 3D
# view had horizontal space to spare and vertical space it wanted.
CONTROL_PANEL_WIDTH = 345
# Docked tab panel. Its widest page (the camera preview) needs ~320px and a
# joint row ~306px, so anything past this was just stretching the sliders
# into dead space and stealing width from the 3D view.
AUX_PANEL_WIDTH = 340
# Camera preview height in that panel: 4:3 at the panel width.
PREVIEW_HEIGHT = AUX_PANEL_WIDTH * 3 // 4
# The diagnostics readout gets its own column rather than a tab, so it can
# be watched while the tabs are in use. Narrow -- it is all label/value rows.
# Sized from measurement, not taste: at 59 rows, ~370 px puts every bounded
# value and ~90% of the names on one line (widest short value "other hand
# ignored" 130 px; 90th-percentile name 169 px; plus tab, scrollbar, margins).
# It was 290, which left a 100 px value column -- too narrow even for the
# unbreakable token "MediaFoundation" (108 px), and it doubled the panel's
# height with wrapping. The column starts collapsed, so this only costs the
# 3D view width while diagnostics is actually open.
DIAG_PANEL_WIDTH = 370
# How the diagnostics column's width is split between a row's name and its
# value. Both sides wrap rather than clip, so this only decides which one
# wraps first -- names are the more expendable, values get the larger share.
DIAG_NAME_STRETCH = 5
DIAG_VALUE_STRETCH = 4

# Named camera presets for the 3D view's toolbar, as (label, tooltip, key).
# The keys are interpreted by URDFHandView.set_named_view.
VIEW_PRESETS = (
    ("Iso", "Isometric view", "iso"),
    ("Front", "Look at the back of the hand", "front"),
    ("Back", "Look at the palm", "back"),
    ("Left", "Pinky side", "left"),
    ("Right", "Thumb side", "right"),
    ("Top", "Looking down onto the fingertips", "top"),
)

# The 3D view's mouse bindings (VTK trackball style), shown as a legend
# beside the view. Only a description -- the bindings themselves are VTK's
# and are not re-implemented here, so this cannot drift out of step by being
# wired to anything.
MOUSE_HELP = (
    ("left", "Rotate"),
    ("middle", "Pan"),
    ("right", "Zoom"),
    ("scroll", "Zoom"),
)

CURRENT_HISTORY_LEN = 400  # rolling sample window for the current-draw graph

PRESET_LABELS = {
    "zero": "Zero (Open)",
    "neutral": "Neutral Grip",
    "fist": "Grip / Fist",
    "peace": "Peace Sign",
    "rock": "Rock Sign",
}

# Gestures where the thumb should only start moving once the fingers have
# had a head start, so it settles on the *outside* of the closing fingers
# instead of risking closing in step with (or ahead of) them and getting
# caught between them mid-close (or, for "peace", tucking in before
# ring/pinky have curled out of the way, leaving it nowhere clear to go).
THUMB_DELAYED_GESTURES = {"fist", "peace"}
#: Positions of the thumb joints inside a full joint array, derived from
#: joints.py rather than hardcoded -- see _apply_preset for what hardcoding
#: them cost.
THUMB_JOINT_INDICES = tuple(
    i for i, j in enumerate(JOINTS) if j.name.startswith("T_")
)
THUMB_DELAY_MS = 400  # PROTOCOL.md's default speed is 400 deg/s (full
                       # speed) as of the latest amendment, so this is
                       # already generous headroom for a full-range finger
                       # close (~0.25s at most) to finish first -- tune
                       # freely, purely a feel knob

# Built-in firmware routines (PROTOCOL.md section 6), split across two
# control-panel blocks by how much feedback each one needs:
#
# - Demos are pure fire-and-forget entertainment -- no interactive input,
#   no need to read a log to know what's happening, just "is it running"
#   -- so they get their own lightweight ProgressButton (a fill sweep
#   timed against the routine's known approximate duration) instead of
#   opening the routine log panel at all.
# - Everything else (homing, feedback-channel discovery, servo-neutral
#   center) is calibration/setup-flavoured, benefits from seeing the
#   hand's own progress text, and one of them (manualhome) is
#   interactive -- these live in "Settings / Calibration" and open the
#   routine log panel the moment they're pressed.
#
# The firmware's `C zero` routine deliberately has no button of its own
# here -- per Adam, it's a UI-level duplicate of the "Zero (Open)" preset
# in the Presets block (T frame vs. C routine under the hood, but nothing
# a user needs two buttons to tell apart). `SerialLink.run_routine("zero")`
# still works if it's ever needed some other way; it's just not exposed.
# Two lines rather than one: the demo number on top, what it does beneath.
# Stacking them keeps both pieces of information without the width a single
# "Demo 1 · Movement" line cost -- that pair was the widest row in the
# control panel and set its minimum width single-handedly. Qt lays out a
# newline in button text natively, so this needs no rich text.
DEMO_LABELS = {
    "demo1": "Demo 1\nMovement",
    "demo2": "Demo 2\nThumb Touch",
}
# Approximate durations (PROTOCOL.md §6: "demo1 takes ~18s, demo2 ~5s"),
# used only to time each ProgressButton's fill sweep -- cosmetic, not a
# real deadline; see MainWindow._on_demo_started.
DEMO_DURATIONS = {"demo1": 18.0, "demo2": 5.0}
DEMO_NAMES = frozenset(DEMO_LABELS)

SETTINGS_ROUTINE_LABELS = {
    "fbfind": "Find Feedback",
    "homeall": "Home All",
    "center": "Center Servos",
    "test": "Connectivity Test",  # v2, PROTOCOL.md §6
}

# The parenthetical detail these labels used to carry inline ("(auto
# stall-detect)", "(servo neutral, 1500µs)", "(centre + sweep each)") moved
# here. Two reasons: the long labels forced the control panel far wider than
# the two-per-row grid needed -- "Connectivity Test (centre + sweep each)"
# alone wanted 294px, pushing the Settings group to 569px against a 430px
# panel -- and a tooltip is where that kind of detail belongs anyway.
ROUTINE_TOOLTIPS = {
    "fbfind": "Discover which feedback channel each joint's position sensor is on",
    "homeall": "Home every joint automatically, using stall detection to find the end stops",
    "center": "Drive every servo to its neutral 1500 µs pulse",
    "test": "Connectivity check: centre every joint, then sweep each one in turn",
    "demo1": "Full movement demo — takes about 18 s",
    "demo2": "Thumb touches each fingertip in turn — takes about 5 s",
}

# If K <name> hasn't arrived this long after a C command was sent, assume
# it's never coming (link.py's own guards -- PC-mode-not-confirmed,
# already-running -- report synchronously and don't need this; this is
# for "wrote to the port, got silence or an unrelated E line back", e.g.
# a firmware mismatch or a rejected command that doesn't cleanly map to
# E badcmd/E badjoint). Without this, RoutinePanel.set_active's optimistic
# disable-everything state has no way back to idle short of reconnecting.
# K should arrive near-instantly if the hand accepts the command at all,
# so this is a generous margin, not a race against real routine duration.
ROUTINE_START_TIMEOUT_MS = 2000


def _distinct_colors(n: int) -> list[str]:
    """n evenly-spaced hues -> hex colors, so 16 overlapping curves on
    the current-draw graph stay visually distinguishable."""
    colors = []
    for i in range(n):
        r, g, b = colorsys.hsv_to_rgb(i / n, 0.65, 0.95)
        colors.append(f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}")
    return colors


class JointRow(QWidget):
    """One joint: name, slider (commanded), measured readout.

    No enable/disable control here -- HRH1 Serial Protocol v1 has no
    motor enable/disable command reachable from PC mode (that only
    exists via the terminal-mode menu), so there's nothing to wire up.
    """

    def __init__(self, joint: JointSpec, on_slider_moved):
        super().__init__()
        self.joint = joint
        self._on_slider_moved = on_slider_moved
        self._suppress_callback = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)

        name_label = QLabel(joint.name)
        name_label.setFixedWidth(55)
        layout.addWidget(name_label)

        # Slider (commanded) with a thin yellow bar directly underneath
        # it showing the live measured position, same width, same
        # min/max scale -- a fill you can compare against the slider's
        # handle at a glance without reading the numeric column.
        slider_column = QVBoxLayout()
        slider_column.setSpacing(1)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(round(joint.min_deg * SLIDER_SCALE))
        self.slider.setMaximum(round(joint.max_deg * SLIDER_SCALE))
        self.slider.setValue(round(joint.default_deg * SLIDER_SCALE))
        self.slider.valueChanged.connect(self._on_slider_changed)
        self._default_focus = self.slider.focusPolicy()
        slider_column.addWidget(self.slider)

        self.measured_bar = QProgressBar()
        self.measured_bar.setMinimum(round(joint.min_deg * SLIDER_SCALE))
        self.measured_bar.setMaximum(round(joint.max_deg * SLIDER_SCALE))
        self.measured_bar.setValue(round(joint.min_deg * SLIDER_SCALE))
        self.measured_bar.setTextVisible(False)
        self.measured_bar.setFixedHeight(MEASURED_BAR_HEIGHT)
        self.measured_bar.setStyleSheet(MEASURED_BAR_STYLE)
        slider_column.addWidget(self.measured_bar)

        layout.addLayout(slider_column, stretch=1)

        self.commanded_label = QLabel(self._format_deg(joint.default_deg))
        self.commanded_label.setFixedWidth(65)
        self.commanded_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self.commanded_label)

        self.measured_label = QLabel("--")
        self.measured_label.setFixedWidth(65)
        self.measured_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self.measured_label)

    @staticmethod
    def _format_deg(value: float) -> str:
        return f"{value:6.1f}"

    def _on_slider_changed(self, raw_value: int) -> None:
        deg = raw_value / SLIDER_SCALE
        self.commanded_label.setText(self._format_deg(deg))
        if not self._suppress_callback:
            self._on_slider_moved(self.joint.index, deg)

    def set_commanded_deg(self, value: float) -> None:
        """Move the slider to reflect an externally-set commanded value
        (a preset jump, a tracked hand) without re-triggering the
        slider-moved callback."""
        self._suppress_callback = True
        self.slider.setValue(round(value * SLIDER_SCALE))
        self._suppress_callback = False

    def set_measured_deg(self, value: float) -> None:
        self.measured_label.setText(self._format_deg(value))
        raw = round(value * SLIDER_SCALE)
        raw = max(self.measured_bar.minimum(), min(self.measured_bar.maximum(), raw))
        self.measured_bar.setValue(raw)

    def clear_measured(self) -> None:
        self.measured_label.setText("--")
        self.measured_bar.setValue(self.measured_bar.minimum())

    def set_read_only(self, read_only: bool) -> None:
        """Read-only = the handle still moves and stays full-color (so it
        reads as a live target readout, not a greyed-out disabled
        control), but can't be grabbed by mouse, wheel, or keyboard.
        Used while hand tracking drives the commanded values -- see
        MainWindow._toggle_slider_panel. Transparent-for-mouse-events
        (rather than setEnabled(False)) is what keeps it looking live;
        dropping focus stops keyboard arrows/page-keys from nudging it.

        The handle itself is hidden via a dynamic property the stylesheet
        keys off (see theme.py): the groove keeps its accent fill, so the
        row still reads as a live target bar, but the grabbable-looking dot
        goes away rather than inviting a drag that would be ignored. Qt
        does not re-evaluate property selectors on its own, hence the
        unpolish/polish."""
        self.slider.setAttribute(Qt.WA_TransparentForMouseEvents, read_only)
        self.slider.setFocusPolicy(Qt.NoFocus if read_only else self._default_focus)
        self.slider.setProperty("readOnly", "true" if read_only else "false")
        style = self.slider.style()
        style.unpolish(self.slider)
        style.polish(self.slider)


class SliderPanel(QWidget):
    """Embedded manual-slider array (not a popup -- one of the pages in
    the main window's auxiliary panel)."""

    def __init__(self, on_slider_moved, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        header = QLabel("Manual Joint Sliders")
        header.setObjectName("panelHeader")
        outer.addWidget(header)

        # Shown only in read-only mode (while hand tracking is driving the
        # commanded values) -- see set_read_only / MainWindow.
        self.readonly_note = QLabel(
            "Read-only: showing live hand-tracking target. "
            "Stop tracking to take manual control."
        )
        self.readonly_note.setObjectName("panelNote")
        self.readonly_note.setWordWrap(True)
        self.readonly_note.hide()
        outer.addWidget(self.readonly_note)

        legend = QHBoxLayout()
        legend.addWidget(QLabel("     "))  # spacer matching name column
        legend.addStretch(1)
        cmd_hdr = QLabel("cmd")
        cmd_hdr.setFixedWidth(65)
        cmd_hdr.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        legend.addWidget(cmd_hdr)
        meas_hdr = QLabel("meas.")
        meas_hdr.setFixedWidth(65)
        meas_hdr.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        legend.addWidget(meas_hdr)
        outer.addLayout(legend)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll, stretch=1)

        rows_container = QWidget()
        rows_layout = QVBoxLayout(rows_container)
        self.rows: list[JointRow] = []
        for joint in JOINTS:
            row = JointRow(joint, on_slider_moved=on_slider_moved)
            rows_layout.addWidget(row)
            self.rows.append(row)
        rows_layout.addStretch(1)
        scroll.setWidget(rows_container)

    def sync_commanded(self, angles_deg) -> None:
        for row, deg in zip(self.rows, angles_deg):
            row.set_commanded_deg(deg)

    def update_measured(self, positions_deg) -> None:
        for row, deg in zip(self.rows, positions_deg):
            row.set_measured_deg(deg)

    def clear_measured(self) -> None:
        for row in self.rows:
            row.clear_measured()

    def set_read_only(self, read_only: bool) -> None:
        """Toggle every joint row's slider between interactive and
        read-only, and surface the explanatory note. See
        JointRow.set_read_only and MainWindow._toggle_slider_panel."""
        self.readonly_note.setVisible(read_only)
        for row in self.rows:
            row.set_read_only(read_only)


class CameraPanel(QWidget):
    """Embedded webcam preview (not a popup) -- the other page in the
    auxiliary panel, shown while hand tracking is active."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        header = QLabel("Hand Tracking Preview")
        header.setObjectName("panelHeader")
        layout.addWidget(header)

        # Tracking status ("RIGHT hand", "No hand", "Confirming...") as a
        # real label above the preview. It used to be drawn into the camera
        # image, which is shrunk to fit this narrow panel -- taking the text
        # down with it to a few pixels tall. Here it stays at full UI size
        # regardless of the preview's scale, and wraps rather than clipping
        # the longer messages. Colour follows the state (see theme.py).
        self.status_label = QLabel("Starting camera...")
        self.status_label.setObjectName("trackingStatus")
        self.status_label.setWordWrap(True)
        self.status_label.setProperty("state", "idle")
        layout.addWidget(self.status_label)

        self.image_label = QLabel("Starting camera...")
        # Top-anchored so the picture sits directly under the status line.
        # Centred, it floated halfway down this tall panel, leaving a large
        # gap between the status text and the image it describes.
        self.image_label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        # Fixed 4:3 height for the panel width, so the switches below sit
        # right under the picture instead of at the bottom of the tab.
        self.image_label.setMinimumWidth(320)
        self.image_label.setFixedHeight(PREVIEW_HEIGHT)
        layout.addWidget(self.image_label)

        # Filtering on/off, directly under the preview -- lets you A/B the
        # landmark-averaging + One Euro smoothing (see vision.py) live
        # against the raw retargeted angles, e.g. to check whether a
        # jittery/wiggling joint is noise (filtering fixes it) or a real
        # retargeting/geometry issue (filtering doesn't).
        filter_row = QHBoxLayout()
        filter_row.addWidget(QLabel("Filtering"))
        self.filter_switch = ToggleSwitch(checked=True)
        filter_row.addWidget(self.filter_switch)
        filter_row.addStretch(1)
        layout.addLayout(filter_row)

        # Fingertip snapping (DexPilot pinch projection, ik_retarget.py): when
        # your thumb and a fingertip come within ~30 mm, the robot is asked to
        # close that gap fully instead of copying the noisy measured one. On
        # by default -- it is what makes pinches close -- but switch it off
        # when pinching real objects: commanding a fingertip well past where
        # the object stops it (> 20 deg) trips the firmware's protection and
        # that finger lets go.
        pinch_row = QHBoxLayout()
        pinch_row.addWidget(QLabel("Snap fingertips together"))
        self.pinch_switch = ToggleSwitch(checked=True)
        self.pinch_switch.setToolTip(
            "On: when your thumb and a fingertip come close, the robot closes "
            "that pinch fully so the tips actually meet.\n"
            "Off: the robot copies the gap it sees, no snapping.\n"
            "Switch off when pinching a real object -- driving a finger more "
            "than 20 deg past where the object stops it trips the joint "
            "protection and the finger lets go."
        )
        pinch_row.addWidget(self.pinch_switch)
        pinch_row.addStretch(1)
        layout.addLayout(pinch_row)
        layout.addStretch(1)

    def set_frame(self, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.image_label.setPixmap(scaled)

    def set_status(self, text: str, state: str = "idle") -> None:
        """Show the tracking status line. `state` is the tracker's own state
        name ("tracking", "confirming", "wrong_hand", ...) and only picks the
        colour; anything unrecognised falls back to the neutral style."""
        style = {"tracking": "good", "confirming": "pending",
                 "wrong_hand": "warn"}.get(state, "idle")
        self.status_label.setText(text)
        if self.status_label.property("state") != style:
            self.status_label.setProperty("state", style)
            self.status_label.style().unpolish(self.status_label)
            self.status_label.style().polish(self.status_label)

    def reset(self) -> None:
        self.image_label.setText("Starting camera...")
        self.image_label.setPixmap(QPixmap())
        self.set_status("Starting camera...", "idle")


class CurrentGraphPanel(QWidget):
    """Embedded rolling graph of per-joint motor current draw (mA) --
    another page in the auxiliary panel, opened via a control-panel
    button. Not a popup; same single-window rule as everything else.

    Per PROTOCOL.md section 5: current is sampled round-robin across all
    16 channels (one LTC2497 conversion in flight at a time, ~160 ms
    each), so any single joint's value only steps roughly every ~2.6 s.
    That staleness is firmware behaviour, not a bug in this graph -- the
    flat stretches between steps are expected and are left visible
    rather than smoothed over, so you can actually see the real update
    cadence.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        header = QLabel("Motor Current Draw")
        header.setObjectName("panelHeader")
        layout.addWidget(header)

        note = QLabel(
            "Each joint updates on its own ~2.6 s round-robin cycle "
            "(PROTOCOL.md §5) — flat stretches between steps "
            "are expected, not a dropped reading."
        )
        note.setWordWrap(True)
        note.setObjectName("panelNote")
        layout.addWidget(note)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#20242c")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.setLabel("left", "Current", units="mA")
        self.plot_widget.setLabel("bottom", "samples ago")
        self.plot_widget.addLegend(offset=(5, 5))
        layout.addWidget(self.plot_widget, stretch=1)

        self._x = list(range(-CURRENT_HISTORY_LEN + 1, 1))
        self._buffers = [
            deque([0.0] * CURRENT_HISTORY_LEN, maxlen=CURRENT_HISTORY_LEN) for _ in range(NUM_JOINTS)
        ]
        colors = _distinct_colors(NUM_JOINTS)
        self._curves = [
            self.plot_widget.plot(pen=pg.mkPen(color=colors[i], width=1.5), name=joint.name)
            for i, joint in enumerate(JOINTS)
        ]

    def add_sample(self, currents_ma) -> None:
        for buf, val in zip(self._buffers, currents_ma):
            buf.append(val)
        if not self.isVisible():
            return  # keep recording history; skip the redraw when hidden
        for curve, buf in zip(self._curves, self._buffers):
            curve.setData(self._x, list(buf))


class DiagnosticsPanel(QWidget):
    """Live performance readout: link traffic, vision, render, power.

    Everything shown is a *measured* rate rather than the configured one --
    the point of the panel is to catch the gap between them. The serial
    stream is nominally 50 Hz and the 3D view 30 Hz, so seeing what they
    actually achieve under load is what tells you whether the hand is
    keeping up.

    Rates come from differencing monotonic counters between refreshes rather
    than from timers on the hot paths, so nothing here costs anything when
    the tab isn't visible beyond an integer increment per event.
    """

    REFRESH_MS = 500

    #: (section, key, label, formatter). Laid out in this order.
    _ROWS = (
        ("Link", "link_state", "State", None),
        ("Link", "tx_rate", "T frames sent", "{:.1f} /s"),
        ("Link", "rx_rate", "S replies received", "{:.1f} /s"),
        ("Link", "lost", "Replies lost", "{:d}"),
        ("Link", "rtt", "Round trip (avg / max)", None),
        ("Link", "s_jitter", "S interval jitter", "±{:.1f} ms"),
        ("Link", "age", "Since last S frame", "{:.0f} ms"),
        ("Link", "tx_tput", "TX throughput", "{:.1f} kB/s"),
        ("Link", "rx_tput", "RX throughput", "{:.1f} kB/s"),
        ("Link", "rx_errors", "E replies (rejected)", "{:d}"),
        ("Link", "rx_error_kind", "Commonest E", None),
        ("Link", "rx_badcs", "Bad-checksum lines in", "{:d}"),
        ("Link", "rx_malformed", "Malformed S frames", "{:d}"),
        ("Link", "tx_blocked", "Held (command running)", "{:d}"),
        ("Link", "tx_skipped", "Skipped (backpressure)", "{:d}"),
        ("Link", "tx_dropped", "Dropped (write stalled)", "{:d}"),
        ("Link", "tx_failed", "Failed writes", "{:d}"),
        ("Link", "rx_backlog", "RX backlog (now / peak)", None),
        ("Link", "rehandshakes", "PC mode re-acquired", "{:d}"),
        ("Link", "resyncs", "Pose resyncs", "{:d}"),

        # The hand's own account (protocol v3 `C stats` -> `I` line, polled
        # by the link every 2 s; counters restart whenever PC mode is entered)
        # plus what its S replies' flags and masks say.
        ("Hand (firmware view)", "hand_age", "Report age", "{:.1f} s"),
        ("Hand (firmware view)", "hand_rx", "Lines received", "{:d}"),
        ("Hand (firmware view)", "hand_ok", "Accepted", "{:d}"),
        ("Hand (firmware view)", "hand_cs", "Checksum rejects", "{:d}"),
        ("Hand (firmware view)", "hand_fmt", "Format rejects", "{:d}"),
        ("Hand (firmware view)", "hand_long", "Too long", "{:d}"),
        ("Hand (firmware view)", "hand_unk", "Unknown type", "{:d}"),
        ("Hand (firmware view)", "hand_wd", "Watchdog trips", "{:d}"),
        ("Hand (firmware view)", "hand_loop", "Slowest loop pass", "{:.1f} ms"),
        ("Hand (firmware view)", "gap", "GAP flags (frame lost)", "{:d}"),
        ("Hand (firmware view)", "wdog", "WDOG flags (PC stalled)", "{:d}"),
        ("Hand (firmware view)", "nodrv", "PWM driver", None),
        ("Hand (firmware view)", "estop", "E-stop", None),
        ("Hand (firmware view)", "faults", "Joint protection", None),
        ("Hand (firmware view)", "fb_count", "Trusted feedback", None),
        ("Hand (firmware view)", "clamped", "Clamped now", None),

        ("Hand tracking", "tracking_state", "State", None),
        ("Hand tracking", "tracked_hand", "Tracked hand", None),
        ("Hand tracking", "capture", "Capture", None),
        ("Hand tracking", "camera_backend", "Camera backend", None),
        ("Hand tracking", "camera_mode", "Camera mode", None),
        ("Hand tracking", "camera_timing", "Per frame: camera wait / work", None),
        ("Hand tracking", "camera_fps", "Camera", "{:.1f} fps"),
        ("Hand tracking", "detect_rate", "Hand detected", "{:.0f}%"),
        ("Hand tracking", "angle_rate", "Poses retargeted", "{:.1f} /s"),
        ("Hand tracking", "solve_ms_avg", "Solve time (avg)", "{:.2f} ms"),
        ("Hand tracking", "solve_ms_max", "Solve time (peak)", "{:.2f} ms"),
        ("Hand tracking", "latency", "Frame read -> target (avg / p95)", None),
        ("Hand tracking", "stale_skipped", "Frames skipped (solver busy)", "{:d}"),
        ("Hand tracking", "solver_residual", "Solver residual", "{:.4f}"),
        ("Hand tracking", "frame_errors", "Dropped frames", "{:d}"),
        ("Hand tracking", "acquisitions", "Hand (re)acquired", "{:d}x"),
        ("Hand tracking", "flicker_suppressed", "Handedness flickers ignored", "{:d}"),
        ("Hand tracking", "extra_hands", "Frames with a 2nd hand", "{:d}"),
        ("Hand tracking", "wrist", "Wrist tracking", None),

        ("Application", "view_fps", "3D view", "{:.1f} fps"),
        ("Application", "stream_hz", "Target stream", "{:.1f} Hz"),
        # Of those sends, how many carried a NEW pose. 30 Hz of sends with ~0
        # here is just the heartbeat -- nothing is asking the hand to move.
        ("Application", "change_hz", "  ...with a new pose", "{:.1f} Hz"),
        ("Application", "poll_hz", "Status poll", "{:.1f} Hz"),

        ("Power", "current_total", "Total current", "{:.0f} mA"),
        ("Power", "current_peak", "Peak total (session)", "{:.0f} mA"),
        ("Power", "current_max_joint", "Highest joint", None),
        ("Power", "current_active", "Joints drawing >50 mA", "{:d}"),
    )

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        header = QLabel("Diagnostics")
        header.setObjectName("panelHeader")
        layout.addWidget(header)
        note = QLabel("Measured rates, not configured ones — the gap is the point.")
        note.setObjectName("panelNote")
        note.setWordWrap(True)
        layout.addWidget(note)

        # One grid for every row, rather than a separate HBox per row.
        #
        # The per-row HBox gave each value only whatever width its label left
        # over, and inside a scroll area with horizontal scrolling off, a row
        # too long to fit is simply clipped at the right edge -- which is
        # exactly where the values are. It went unnoticed at 27 rows; at 59,
        # with free-text values like "capturing neutral (hold still)" and a
        # comma-joined list of clamped joints (unbounded length), no column
        # width can guarantee a fit.
        #
        # So: two columns on a fixed name:value split, and BOTH sides wrap
        # onto extra lines when they run out of room. The size policies are
        # Ignored horizontally so the columns are sized purely by the stretch
        # split, never by how long any one string happens to be -- that is
        # what makes clipping impossible rather than merely unlikely. A bonus
        # is that every value now lines up on one right-hand edge.
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(0)
        grid.setColumnStretch(0, DIAG_NAME_STRETCH)
        grid.setColumnStretch(1, DIAG_VALUE_STRETCH)

        self._values: dict[str, QLabel] = {}
        current_section = None
        r = 0
        for section, key, label, _fmt in self._ROWS:
            if section != current_section:
                current_section = section
                caption = QLabel(section.upper())
                caption.setObjectName("diagSection")
                grid.addWidget(caption, r, 0, 1, 2)
                r += 1
            name = QLabel(label)
            name.setObjectName("diagLabel")
            name.setWordWrap(True)
            name.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            value = QLabel("--")
            value.setObjectName("diagValue")
            value.setWordWrap(True)
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            grid.addWidget(name, r, 0)
            grid.addWidget(value, r, 1)
            self._values[key] = value
            r += 1
        layout.addLayout(grid)

        layout.addStretch(1)

        self._peak_current = 0.0
        self._vision_stats: dict = {}
        self._last_snapshot: Optional[tuple] = None  # (time, counters dict)

    def set_vision_stats(self, stats: dict) -> None:
        self._vision_stats = stats

    def clear_vision_stats(self) -> None:
        self._vision_stats = {}

    def reset_peaks(self) -> None:
        self._peak_current = 0.0

    def update_metrics(self, link, status, counters: dict) -> None:
        """Refresh every readout. `counters` holds the GUI-side event tallies
        (view redraws, stream sends, status polls); `link` supplies the
        serial traffic totals and `status` the latest S frame."""
        now = time.monotonic()
        out: dict = {}

        link_stats = link.stats() if link is not None and hasattr(link, "stats") else {}
        merged = dict(link_stats)
        merged.update(counters)
        hand = link_stats.get("hand") or {}
        for k, v in hand.items():  # flattened so rate() can difference them
            merged[f"hand_{k}"] = v

        prev = self._last_snapshot
        self._last_snapshot = (now, merged)
        dt = (now - prev[0]) if prev else 0.0

        def rate(key: str) -> Optional[float]:
            """Per-second rate from two successive counter readings.

            A *decrease* means the counter was reset under us -- reconnecting
            builds a fresh SerialLink whose totals start at zero -- so that
            sample is discarded rather than reported as a large negative
            spike. The next refresh re-baselines and reads correctly."""
            if not prev or dt <= 0 or key not in merged or key not in prev[1]:
                return None
            delta = merged[key] - prev[1][key]
            if delta < 0:
                return None
            return delta / dt

        if link is None or not getattr(link, "is_open", False):
            out["link_state"] = "disconnected"
        elif not link_stats.get("pc_mode", True):
            out["link_state"] = "handshaking"
        elif link.is_link_stale(LINK_STALE_TIMEOUT_S):
            out["link_state"] = "STALE"
        else:
            out["link_state"] = "streaming"

        for src, dst in (("tx_frames", "tx_rate"), ("rx_status", "rx_rate")):
            v = rate(src)
            if v is not None:
                out[dst] = v
        for src, dst in (("tx_bytes", "tx_tput"), ("rx_bytes", "rx_tput")):
            v = rate(src)
            if v is not None:
                out[dst] = v / 1000.0
        for k in ("rx_errors", "rx_malformed", "tx_failed", "tx_skipped", "tx_dropped",
                  "rehandshakes"):
            if k in link_stats:
                out[k] = link_stats[k]
        if "rx_backlog" in link_stats:
            out["rx_backlog"] = f"{link_stats['rx_backlog']} / {link_stats.get('rx_backlog_max', 0)} B"

        for k in ("lost", "tx_blocked", "rx_badcs", "resyncs", "gap", "wdog"):
            if k in link_stats:
                out[k] = link_stats[k]
        if "rtt_mean" in link_stats:
            out["rtt"] = f"{link_stats['rtt_mean'] * 1000:.1f} / {link_stats['rtt_max'] * 1000:.1f} ms"
        if status is not None:
            out["nodrv"] = "NOT DETECTED" if status.flags & 0x04 else "ok"
            out["estop"] = ("ACTIVE" if status.estop else "off") + \
                f" ({link_stats.get('estops_sent', 0)} sent, {link_stats.get('resets', 0)} resets)"
            faults = link_stats.get("faults") or {}
            tripped = [f"{k} {v}" for k, v in faults.items() if k != "estop"]
            out["faults"] = ("FAULT: " + ", ".join(tripped) if tripped else "FAULT") if status.fault else "ok"
            out["fb_count"] = f"{bin(status.fb_mask).count('1')} / {NUM_JOINTS} joints"
            names = [JOINTS[i].name for i in range(NUM_JOINTS) if status.clamp_mask >> i & 1]
            out["clamped"] = ", ".join(names) if names else "none"

        # Firmware counters. Only shown while the report is fresh -- a stale
        # one would read as "all fine" exactly when the hand has gone quiet.
        age = link_stats.get("hand_age")
        if hand and age is not None:
            out["hand_age"] = age
            for k in ("rx", "ok", "cs", "fmt", "long", "unk", "wd", "loop"):
                if k not in hand:
                    continue
                if age >= 6.0:
                    out[f"hand_{k}"] = "stale"
                elif k == "loop":
                    out["hand_loop"] = hand["loop"] / 1000.0
                else:
                    out[f"hand_{k}"] = hand[k]

        vs = self._vision_stats
        out["tracking_state"] = "running" if vs else "off"
        if vs:
            cap = vs.get("capture")
            if cap:
                out["capture"] = f"{cap[0]}x{cap[1]}, {cap[2]:.0f} fps measured at start"
            out["camera_fps"] = vs.get("camera_fps", 0.0)
            out["detect_rate"] = vs.get("detect_rate", 0.0) * 100.0
            out["solve_ms_avg"] = vs.get("solve_ms_avg", 0.0)
            out["solve_ms_max"] = vs.get("solve_ms_max", 0.0)
            out["frame_errors"] = vs.get("frame_errors", 0)
            state = vs.get("track_state")
            hand = vs.get("tracked_hand")
            if state == "tracking" and hand:
                out["tracked_hand"] = hand + (" (mirrored)" if hand == "Left" else "")
            elif state == "wrong_hand":
                out["tracked_hand"] = "other hand ignored"
            elif state == "confirming":
                out["tracked_hand"] = "confirming..."
            else:
                out["tracked_hand"] = "none"
            timing = vs.get("camera_timing")
            if timing:
                wait, work = timing
                who = "camera-limited" if wait >= work else "PC-limited"
                out["camera_timing"] = f"{wait:.0f} / {work:.0f} ms ({who})"
            for key in ("camera_backend", "camera_mode", "wrist"):
                if vs.get(key):
                    out[key] = vs[key]
            lat = vs.get("latency_ms")
            if lat:
                out["latency"] = f"{lat[0]:.0f} / {lat[1]:.0f} ms"
            for key in ("acquisitions", "flicker_suppressed", "extra_hands", "stale_skipped"):
                if key in vs:
                    out[key] = vs[key]
            if "solver_residual" in vs:
                out["solver_residual"] = vs["solver_residual"]
            v = rate("angles")
            if v is not None:
                out["angle_rate"] = v

        for src, dst in (("view_frames", "view_fps"), ("stream_sends", "stream_hz"),
                         ("stream_changes", "change_hz"), ("status_polls", "poll_hz")):
            v = rate(src)
            if v is not None:
                out[dst] = v

        if status is not None and status.currents_ma:
            currents = list(status.currents_ma)
            total = sum(currents)
            out["current_total"] = total
            self._peak_current = max(self._peak_current, total)
            out["current_peak"] = self._peak_current
            i = max(range(len(currents)), key=lambda n: currents[n])
            if i < len(JOINTS):
                out["current_max_joint"] = f"{JOINTS[i].name}  {currents[i]:.0f} mA"
            out["current_active"] = sum(1 for c in currents if c > 50.0)

        self._apply(out)

    def _apply(self, out: dict) -> None:
        for _section, key, _label, fmt in self._ROWS:
            widget = self._values[key]
            if key not in out:
                continue
            value = out[key]
            try:
                widget.setText(fmt.format(value) if fmt else str(value))
            except (ValueError, TypeError):
                widget.setText(str(value))
        # Colour the two state rows so the panel reads at a glance.
        for key, good in (("link_state", "streaming"), ("tracking_state", "running")):
            if key in out:
                ok = out[key] == good
                self._values[key].setProperty("state", "good" if ok else "idle")
                st = self._values[key].style()
                st.unpolish(self._values[key])
                st.polish(self._values[key])


class _HandViewPlaceholder(QLabel):
    """Stands in for the 3D view when it cannot be created (no URDF export,
    no OpenGL). Accepts poses and ignores them, so callers need no checks."""

    def __init__(self, reason: str):
        super().__init__(f"3D hand view unavailable\n({reason})")
        self.setAlignment(Qt.AlignCenter)
        self.setObjectName("mutedLabel")
        self.setWordWrap(True)

    def set_pose(self, angles_deg) -> None:
        pass


class LinkSignals(QObject):
    """Bridges SerialLink's callbacks (invoked from its background reader
    thread, see link.py) onto Qt signals so their slots run on the GUI
    thread via an explicit QueuedConnection -- the same class of risk
    (and the same fix) as vision.py's VisionWorker signals; see that
    module's docstring. All four routine_* signals do more than a single
    tolerant .setText() (repeatedly append to a QTextEdit, toggle several
    widgets' enabled state), so they get the fuller treatment rather than
    being called directly from the reader thread. link_error started out
    as a "simple .setText(), direct call is fine" case per the original
    comment here, but _on_link_error now also touches the routine panel
    when a hand error arrives mid-request (see its docstring) -- routed
    through a signal too now for the same reason."""

    routine_started = Signal(str)
    routine_log = Signal(str)
    routine_done = Signal(str)
    routine_error = Signal(str)
    link_error = Signal(str)
    faults = Signal(object)   # dict from `C faults` ({} after a reset)


class RoutinePanel(QWidget):
    """Embedded panel (not a popup) showing live feedback for PROTOCOL.md
    section 6's built-in firmware routines that warrant it: homing,
    feedback-channel discovery, zero, and servo-neutral center. Routines
    block the *entire* link while active (no telemetry, no T frames
    accepted) -- MainWindow pauses the stream timer and hand tracking for
    exactly that reason, see MainWindow._on_routine_started.

    The two "demo" routines deliberately don't use this panel at all --
    they get their own ProgressButton directly in the control panel's
    Demos block instead (see DEMO_NAMES), since neither needs interactive
    input or benefits enough from log text to be worth opening a side
    panel for.

    `manualhome` is the one interactive routine here: it prompts per-joint
    on the same link and waits indefinitely for a typed reply (a jog
    amount in microseconds, or `d` to advance). The input row below is
    only enabled while it's the active routine. Every trigger button for
    the routines this panel *does* cover lives in the control panel's
    "Settings / Calibration" group, not here -- pressing one opens this
    panel automatically (MainWindow._start_routine), so this class only
    needs to own the log/status/interactive-input display, not the
    triggering.
    """

    def __init__(self, on_send_input, parent=None):
        super().__init__(parent)
        self._on_send_input = on_send_input

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        header = QLabel("Routine Log")
        header.setObjectName("panelHeader")
        layout.addWidget(header)

        note = QLabel(
            "Runs on the hand's own firmware. Blocks the link completely "
            "while active -- no live position/current elsewhere in the "
            "app until it finishes."
        )
        note.setWordWrap(True)
        note.setObjectName("panelNote")
        layout.addWidget(note)

        self.status_label = QLabel("Idle")
        self.status_label.setObjectName("routineStatus")
        layout.addWidget(self.status_label)

        layout.addWidget(QLabel("Log"))
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setObjectName("logView")
        layout.addWidget(self.log_view, stretch=1)

        input_row = QHBoxLayout()
        self.input_edit = QLineEdit()
        self.input_edit.setPlaceholderText("jog amount (µs) or 'd' -- manualhome only")
        self.input_edit.setEnabled(False)
        self.input_edit.returnPressed.connect(self._send_input)
        input_row.addWidget(self.input_edit, stretch=1)
        self.send_btn = QPushButton("Send")
        self.send_btn.setEnabled(False)
        self.send_btn.clicked.connect(self._send_input)
        input_row.addWidget(self.send_btn)
        layout.addLayout(input_row)

    def _send_input(self) -> None:
        text = self.input_edit.text().strip()
        if not text:
            return
        self._on_send_input(text)
        self.append_log(f"> {text}")
        self.input_edit.clear()

    def append_log(self, line: str) -> None:
        self.log_view.append(line)
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def clear_log(self) -> None:
        self.log_view.clear()

    def set_active(self, name: str | None) -> None:
        """name=None means idle. Triggering/disabling the buttons that
        would start a second routine on top of this one is now
        MainWindow._set_routine_controls_enabled's job (those buttons
        live in the control panel, not here) -- this only updates what
        this panel itself displays: the status line, and whether the
        interactive input row is live (manualhome only)."""
        interactive = name == "manualhome"
        self.input_edit.setEnabled(interactive)
        self.send_btn.setEnabled(interactive)
        self.status_label.setText("Idle" if name is None else f"Running: {name}")
        # Dynamic property + re-polish so QSS's routineStatus[running="true"]
        # selector picks up the state change -- Qt doesn't re-evaluate
        # property selectors on setText() alone.
        self.status_label.setProperty("running", name is not None)
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HRH1 Control")

        self.link: SerialLink | None = None
        self.commanded: list[float] = [0.0] * NUM_JOINTS

        self.vision_thread: QThread | None = None
        self.vision_worker: VisionWorker | None = None
        self.tracking_active = False


        self._aux_mode: str | None = None  # None | "sliders" | "camera" | "current" | "routine"
        self.hand_view_mode = "measured"  # "measured" | "target"
        self._preset_stage_token = 0  # invalidates a pending staged-thumb callback, see _apply_preset

        # Bridges SerialLink's routine callbacks (reader thread) onto Qt
        # signals -- see LinkSignals' docstring. Connected with an explicit
        # QueuedConnection for the same reason as vision_worker's signals
        # below: don't trust AutoConnection's thread-detection heuristic.
        self._link_signals = LinkSignals(self)
        self._link_signals.routine_started.connect(self._on_routine_started, Qt.QueuedConnection)
        self._link_signals.routine_log.connect(self._on_routine_log, Qt.QueuedConnection)
        self._link_signals.routine_done.connect(self._on_routine_done, Qt.QueuedConnection)
        self._link_signals.routine_error.connect(self._on_routine_error, Qt.QueuedConnection)
        self._link_signals.link_error.connect(self._on_link_error, Qt.QueuedConnection)
        self._link_signals.faults.connect(self._on_faults, Qt.QueuedConnection)

        # Protection state as last shown (see _update_safety_panel). The
        # E-stop itself lives on the hand; these only drive the UI and the
        # target hand-over after a reset.
        self._estop_shown = False
        self._fault_shown = False
        self._faults: dict = {}
        # Targets as they were when the E-stop latched. Joints whose feedback
        # is not trusted are not re-seeded by the post-reset resync, so they
        # fall back to these rather than to whatever sliders/tracking left in
        # `commanded` while the hand was stopped.
        self._commanded_at_estop: Optional[list] = None
        self._reseed_from_estop = False


        # Invalidates a pending _check_routine_started timeout the same
        # way _preset_stage_token invalidates a pending staged-thumb
        # callback -- bumped whenever K/D/an error resolves the request
        # for real, so a stale timeout can't undo a state that's already
        # correct.
        self._routine_request_token = 0
        # True from _start_routine until K / an error / the timeout
        # resolves it -- lets _on_link_error recognize a generic hand
        # error (E unknown / E badframe / E timeout, as opposed to the
        # unambiguous E badcmd / E badjoint) as probably routine-related
        # when one arrives in this window, and surface it in the routine
        # log immediately instead of making the user wait out the full
        # timeout for a "no response" message that isn't quite accurate.
        self._routine_awaiting_start = False
        # Name of the routine most recently requested via _start_routine,
        # cleared once it resolves (K/D/an error/the timeout). Set once,
        # read from every resolution path to decide whether a demo
        # button's progress fill needs resetting or the routine panel's
        # status does -- see DEMO_NAMES and _set_routine_controls_enabled.
        self._pending_routine_name: str | None = None
        # {name: QTimer} for demo buttons currently animating their
        # progress fill -- see _on_demo_started / _reset_demo_button.
        self._demo_progress_timers: dict[str, QTimer] = {}
        # Populated by _build_demos_group / _build_settings_group;
        # disabled as a group by _set_routine_controls_enabled while any
        # one routine is requested/running, since the firmware only ever
        # runs one at a time.
        self._routine_trigger_buttons: list[QPushButton] = []
        self._demo_buttons: dict[str, ProgressButton] = {}

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # Control | docked side panels | 3D view (takes all the slack).
        # Both fixed-width columns sit together on the left, so everything
        # you operate is in one place and the 3D view gets one uninterrupted
        # region on the right -- with its own camera toolbar down the far
        # edge, where a CAD package puts it.
        root.addWidget(self._build_control_panel(), stretch=0)
        root.addWidget(self._build_aux_panel(), stretch=0)
        root.addWidget(self._build_diagnostics_dock(), stretch=0)
        root.addWidget(self._build_hand_view_panel(), stretch=1)

        self.status_bar_label = QLabel("Disconnected")
        self.statusBar().addWidget(self.status_bar_label)

        self.stream_timer = QTimer(self)
        # PreciseTimer: the default CoarseTimer may be off by 5 % of the
        # interval, and Windows' default timer resolution is ~15.6 ms.
        self.stream_timer.setTimerType(Qt.PreciseTimer)
        self.stream_timer.timeout.connect(self._stream_targets)

        # Precise for the same reason as the stream timer above, and it
        # mattered more than it looks. A CoarseTimer on Windows snaps to the
        # 15.625 ms system tick, so these two ran slow by a whole tick:
        #   33 ms view timer -> 3 ticks = 46.9 ms -> 21.3 Hz, not 30
        #   50 ms poll timer -> 4 ticks = 62.5 ms -> 16.0 Hz, not 20
        # Those are exactly the "normal" rates observed with nothing
        # rendering at all -- the ceiling was the timer, never the renderer.
        self.poll_timer = QTimer(self)
        self.poll_timer.setTimerType(Qt.PreciseTimer)
        self.poll_timer.timeout.connect(self._poll_status)
        self.poll_timer.start(STATUS_POLL_INTERVAL_MS)

        self.hand_view_timer = QTimer(self)
        self.hand_view_timer.setTimerType(Qt.PreciseTimer)
        self.hand_view_timer.timeout.connect(self._refresh_hand_view)
        self.hand_view_timer.start(HAND_VIEW_INTERVAL_MS)

        # GUI-side event tallies for the diagnostics panel. Monotonic counts
        # only -- the panel differences them, so incrementing costs nothing.
        self._diag_counters = {
            "view_frames": 0, "stream_sends": 0, "stream_changes": 0, "status_polls": 0,
            "angles": 0,
        }
        # Last pose actually put on the wire, for the send-on-change check in
        # _stream_targets. None means "nothing sent yet", so the first tick
        # after connecting always transmits.
        self._last_sent: Optional[list] = None
        self.diag_timer = QTimer(self)
        self.diag_timer.timeout.connect(self._refresh_diagnostics)
        self.diag_timer.start(DiagnosticsPanel.REFRESH_MS)

        self._refresh_ports()

    # -- hand view (right) ---------------------------------------------------

    def _build_hand_view_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        header = QHBoxLayout()
        title = QLabel("3D Hand View")
        title.setObjectName("panelHeader")
        header.addWidget(title)
        header.addStretch(1)

        # Switch between the *measured* pose (from the link's S frames --
        # what the hand is actually doing) and the *target* pose
        # (self.commanded -- what's currently being sent as a T frame).
        # Flanking labels rather than dynamic switch text -- simpler than
        # drawing text inside the knob track.
        header.addWidget(QLabel("Measured"))
        self.hand_view_mode_switch = ToggleSwitch(checked=False)
        self.hand_view_mode_switch.toggled.connect(self._on_hand_view_mode_toggled)
        header.addWidget(self.hand_view_mode_switch)
        header.addWidget(QLabel("Target"))
        layout.addLayout(header)

        # The 3D view with a vertical camera toolbar down its right edge --
        # the same place a CAD package puts its view cube, and directly
        # under the interactive orientation gizmo the view draws in that
        # corner. Buttons as well as the gizmo because the gizmo is small
        # and fiddly to hit precisely, and a named view is one click.
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(8)

        self.hand_view = self._build_hand_view()
        body.addWidget(self.hand_view, stretch=1)
        body.addWidget(self._build_view_toolbar(), stretch=0)
        layout.addLayout(body, stretch=1)

        return panel

    def _build_view_toolbar(self) -> QWidget:
        bar = QWidget()
        bar.setObjectName("viewToolbar")
        bar.setFixedWidth(88)  # wide enough for the mouse legend's labels
        col = QVBoxLayout(bar)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(6)

        # Mouse legend first: it is reference material you read once while
        # learning the view, so it belongs at the top where the eye lands,
        # above the buttons you press repeatedly.
        col.addWidget(self._build_mouse_help())
        col.addSpacing(10)

        caption = QLabel("VIEW")
        caption.setObjectName("toolbarCaption")
        caption.setAlignment(Qt.AlignCenter)
        col.addWidget(caption)

        for label, tip, key in VIEW_PRESETS:
            btn = QPushButton(label)
            btn.setObjectName("viewButton")
            btn.setToolTip(tip)
            btn.clicked.connect(lambda checked=False, k=key: self._set_hand_view_camera(k))
            col.addWidget(btn)

        col.addSpacing(6)
        fit = QPushButton("Fit")
        fit.setObjectName("viewButton")
        fit.setToolTip("Zoom to fit the whole hand")
        fit.clicked.connect(lambda: self._set_hand_view_camera("fit"))
        col.addWidget(fit)

        col.addStretch(1)
        return bar

    def _build_mouse_help(self) -> QWidget:
        """Legend for the 3D view's mouse bindings, under the view buttons.

        Drawn mouse glyphs rather than text like "RMB", because a picture of
        the button you are meant to press needs no decoding. Nothing here is
        connected to anything: the bindings belong to VTK's trackball
        interactor, and this only documents them."""
        box = QGroupBox("Mouse")
        box.setObjectName("mouseHelp")
        grid = QGridLayout(box)
        grid.setContentsMargins(6, 4, 6, 6)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(5)
        for row, (part, action) in enumerate(MOUSE_HELP):
            glyph = MouseGlyph(part)
            glyph.setToolTip(f"{part.title()} mouse button — {action.lower()}")
            grid.addWidget(glyph, row, 0, Qt.AlignHCenter)
            label = QLabel(action)
            label.setObjectName("mouseHelpLabel")
            grid.addWidget(label, row, 1, Qt.AlignVCenter | Qt.AlignLeft)
        grid.setColumnStretch(1, 1)
        return box

    def _set_hand_view_camera(self, key: str) -> None:
        """Ask the hand view for a named camera. The placeholder shown when
        the 3D view is unavailable has no such method, so this is a no-op."""
        setter = getattr(self.hand_view, "set_named_view", None)
        if callable(setter):
            setter(key)

    def _build_hand_view(self) -> QWidget:
        """The CAD hand (hand_view3d_urdf.URDFHandView), driven by the Fusion
        URDF export with a 1:1 firmware->joint mapping. Any failure (missing
        URDF export, pyvista/OpenGL problem, ...) must never crash the app:
        a text placeholder is shown instead and everything else still works."""
        xacro = hrh1_assembly_xacro()
        reason = "URDF export not found"
        if URDFHandView is not None and xacro is not None:
            try:
                return URDFHandView(str(xacro), firmware_map=HRH1_URDF_JOINT_MAP)
            except Exception as exc:
                reason = str(exc)
        elif URDFHandView is None:
            reason = "pyvista / pyvistaqt not available"
        print(f"3D hand view unavailable: {reason}")
        return _HandViewPlaceholder(reason)

    def _set_commanded(self, angles) -> None:
        """Replace the commanded pose, refusing anything the wrong length.

        `commanded` is the single value the 50 Hz stream timer sends, so a
        wrong-length array here is not a local mistake -- it breaks every
        subsequent frame until something overwrites it, which is exactly how
        the 16-vs-19 preset bug managed to freeze the hand. Catching it at
        the assignment names the culprit instead of blaming send_targets."""
        angles = list(angles)
        if len(angles) != NUM_JOINTS:
            self.status_bar_label.setText(
                f"Internal error: refused a {len(angles)}-joint pose "
                f"(expected {NUM_JOINTS}); keeping the previous target"
            )
            return
        self.commanded = angles
        # Always keep the sliders in step, visible or not. This used to be
        # gated on `_aux_mode == "sliders"` -- an optimisation from when the
        # slider panel was hidden most of the time -- and a stale flag was
        # enough to leave the sliders showing an old pose while the hand did
        # something else. A sync costs ~0.07 ms and suppresses the slider
        # callbacks (JointRow.set_commanded_deg), so it cannot feed back into
        # _on_slider_moved or stop tracking; it isn't worth being clever about.
        self.slider_panel.sync_commanded(self.commanded)

    def _refresh_diagnostics(self) -> None:
        """Drive the diagnostics panel. Runs regardless of which tab is
        showing, because the rates it reports are averages over the refresh
        interval -- pausing it while hidden would leave a misleading spike
        on the first update after switching back."""
        status = self.link.last_status if (self.link is not None and self.link.is_open) else None
        try:
            self.diagnostics_panel.update_metrics(self.link, status, self._diag_counters)
        except Exception as exc:  # noqa: BLE001 - a readout is never worth a crash
            print(f"diagnostics update failed: {exc}")

    def _on_hand_view_mode_toggled(self, checked: bool) -> None:
        self.hand_view_mode = "target" if checked else "measured"
        self._refresh_hand_view()  # update immediately, don't wait for the next tick

    # -- control panel (left) ----------------------------------------------

    def _build_control_panel(self) -> QWidget:
        """Connection / Presets / Demos / Tracking / Settings, in one column
        of two-button-wide groups.

        This used to be a single column of full-width buttons, which made
        the panel taller than most screens -- so it lived in a QScrollArea
        and you had to scroll to reach Settings. Laying the buttons out two
        per row instead roughly halves the height at the cost of width, and
        width is the one thing this window has to spare: the hand is a tall,
        narrow assembly, so the 3D view never wanted the horizontal space.
        The QScrollArea is kept as a safety net for genuinely short windows,
        but at CONTROL_PANEL_WIDTH the content now fits without it.
        """
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setSpacing(10)

        title = QLabel("HRH1")
        title.setObjectName("appTitle")
        subtitle = QLabel("Humanoid Robotic Hand · control")
        subtitle.setObjectName("appSubtitle")
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addSpacing(2)

        # E-stop first, above everything else: it must be findable without
        # looking for it.
        layout.addWidget(self._build_safety_group())
        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_presets_group())
        layout.addWidget(self._build_demos_group())

        # No Home button here: HRH1 Serial Protocol v1 has no home
        # command reachable from PC mode outside the C routines below
        # (homeall / manualhome, in Settings / Calibration). "Zero (Open)"
        # in the Presets group above sends every joint to 0 deg
        # (anatomical zero) via an ordinary T frame -- the firmware's own
        # `C zero` routine does the same thing a different way, but isn't
        # exposed as its own button, see SETTINGS_ROUTINE_LABELS' comment.

        layout.addWidget(self._build_tracking_group())
        layout.addWidget(self._build_settings_group())

        # Starts collapsed: diagnostics is something you open when you are
        # chasing a problem, not the default view, and the column is ~300px
        # the 3D hand would otherwise have.
        self.diag_btn = QPushButton("Show Diagnostics")
        self.diag_btn.setCheckable(True)
        self.diag_btn.setChecked(False)
        self.diag_btn.setToolTip(
            "Show or hide the live diagnostics column beside the tabs"
        )
        self.diag_btn.toggled.connect(self._toggle_diagnostics)
        layout.addWidget(self.diag_btn)

        # The old "Show Manual Sliders" / "Show Current Graph" / "Show
        # Routine Log" buttons are gone: those panels are now permanently
        # docked as tabs on the right (see _build_aux_panel), so a button
        # whose only job was to reveal them would be duplicating a tab.

        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidget(inner)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(CONTROL_PANEL_WIDTH)
        scroll.setObjectName("controlScroll")
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        return scroll

    @staticmethod
    def _grid(group: QWidget, columns: int = 2) -> QGridLayout:
        """A tidy N-column grid on `group`, with equal-width columns so
        buttons line up across rows regardless of label length."""
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        for c in range(columns):
            grid.setColumnStretch(c, 1)
        return grid

    def _build_safety_group(self) -> QWidget:
        """Digital E-stop, its state, and Reset (PROTOCOL.md §10.5).

        The button sends the bare `!` line straight away, whatever else is
        going on -- mid-routine, mid-resync, while a command is outstanding
        (SerialLink.estop bypasses the T/C slots). Esc does the same from
        anywhere in the window. The status line below follows the ESTOP and
        FAULT flags of the hand's own S replies, so it shows what the hand
        reports, not what was clicked."""
        box = QWidget()
        box.setObjectName("safetyBox")
        col = QVBoxLayout(box)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(6)

        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setObjectName("estopButton")
        self.estop_btn.setToolTip(
            "Stop the hand now (Esc). Works at any time, also during demos and "
            "calibration routines. The hand keeps reporting positions but "
            "ignores every target until Reset."
        )
        self.estop_btn.clicked.connect(self._on_estop_clicked)
        col.addWidget(self.estop_btn)

        row = QHBoxLayout()
        row.setSpacing(8)
        self.safety_label = QLabel("Not connected")
        self.safety_label.setObjectName("safetyStatus")
        self.safety_label.setWordWrap(True)
        self.safety_label.setProperty("state", "idle")
        row.addWidget(self.safety_label, stretch=1)
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.setToolTip(
            "Clear the E-stop and any joint faults. The hand stays where it is "
            "and the app takes its measured pose as the new target before "
            "anything moves at full speed."
        )
        self.reset_btn.setEnabled(False)
        self.reset_btn.clicked.connect(self._on_reset_clicked)
        row.addWidget(self.reset_btn)
        col.addLayout(row)

        # Esc from anywhere in the window -- including while a text field
        # (routine input) has focus, hence the application-wide context.
        self.estop_shortcut = QShortcut(QKeySequence(Qt.Key_Escape), self)
        self.estop_shortcut.setContext(Qt.ApplicationShortcut)
        self.estop_shortcut.activated.connect(self._on_estop_clicked)
        return box

    def _build_connection_group(self) -> QWidget:
        group = QGroupBox("Connection")
        layout = QVBoxLayout(group)

        port_row = QHBoxLayout()
        self.port_combo = QComboBox()
        port_row.addWidget(self.port_combo, stretch=1)
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(refresh_btn)
        layout.addLayout(port_row)

        # LED answers "is the port open at all" at a glance -- orange
        # (idle) until _connect() succeeds, green while connected. Kept
        # separate from the "Link: OK/STALE" label below, which instead
        # reflects S-frame telemetry freshness and has nothing meaningful
        # to say before the first S frame arrives.
        connect_row = QHBoxLayout()
        self.connect_led = StatusLED(color="#e0a020")
        connect_row.addWidget(self.connect_led)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("primaryButton")
        self.connect_btn.clicked.connect(self._toggle_connection)
        connect_row.addWidget(self.connect_btn, stretch=1)
        layout.addLayout(connect_row)

        self.link_health_label = QLabel("Link: --")
        layout.addWidget(self.link_health_label)

        return group

    def _build_presets_group(self) -> QWidget:
        group = QGroupBox("Presets")
        grid = self._grid(group)
        for i, name in enumerate(GESTURE_NAMES):
            label = PRESET_LABELS.get(name, name.title())
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked=False, n=name: self._apply_preset(n))
            # An odd preset count leaves a gap; give the last one the whole
            # row rather than stranding a half-width button beside empty space.
            last_alone = (i == len(GESTURE_NAMES) - 1) and (len(GESTURE_NAMES) % 2 == 1)
            if last_alone:
                grid.addWidget(btn, i // 2, 0, 1, 2)
            else:
                grid.addWidget(btn, i // 2, i % 2)
        return group

    def _build_demos_group(self) -> QWidget:
        """Firmware demo routines (PROTOCOL.md §6): pure fire-and-forget,
        no interactive input, no log needed to know what's happening --
        each button doubles as its own progress indicator (a green sweep
        timed against the routine's known approximate duration) instead
        of opening the routine log panel. See DEMO_NAMES."""
        group = QGroupBox("Demos")
        grid = self._grid(group)
        for i, (name, label) in enumerate(DEMO_LABELS.items()):
            btn = ProgressButton(label)
            # Two-line label: theme.py gives #demoButton a two-line
            # min-height. The generic one-line QPushButton min-height was
            # what the layout sized these to, cropping the text top and
            # bottom (a QSS min-height overrides setMinimumHeight, so it has
            # to be fixed in the stylesheet, not here).
            btn.setObjectName("demoButton")
            btn.setToolTip(ROUTINE_TOOLTIPS.get(name, ""))
            btn.clicked.connect(lambda checked=False, n=name: self._start_routine(n))
            grid.addWidget(btn, i // 2, i % 2)
            self._demo_buttons[name] = btn
            self._routine_trigger_buttons.append(btn)
        return group

    def _build_settings_group(self) -> QWidget:
        """Homing, feedback-channel calibration, zero, and servo-neutral
        center (PROTOCOL.md §6) -- calibration/setup-flavoured routines
        where seeing progress text (or, for manualhome, replying to
        prompts) actually matters. Pressing any of these opens the
        Routine Log panel automatically -- see _start_routine."""
        group = QGroupBox("Settings / Calibration")
        layout = QVBoxLayout(group)

        routines = QWidget()
        grid = self._grid(routines)
        grid.setContentsMargins(0, 0, 0, 0)
        for i, (name, label) in enumerate(SETTINGS_ROUTINE_LABELS.items()):
            btn = QPushButton(label)
            btn.setToolTip(ROUTINE_TOOLTIPS.get(name, ""))
            btn.clicked.connect(lambda checked=False, n=name: self._start_routine(n))
            grid.addWidget(btn, i // 2, i % 2)
            self._routine_trigger_buttons.append(btn)
        layout.addWidget(routines)

        home_row = QHBoxLayout()
        self.manualhome_joint_combo = QComboBox()
        self.manualhome_joint_combo.addItem("All joints", None)
        for joint in JOINTS:
            self.manualhome_joint_combo.addItem(joint.name, joint.name)
        home_row.addWidget(self.manualhome_joint_combo, stretch=1)
        self.manualhome_btn = QPushButton("Manual Home")
        self.manualhome_btn.clicked.connect(self._start_manualhome)
        home_row.addWidget(self.manualhome_btn)
        layout.addLayout(home_row)
        self._routine_trigger_buttons.append(self.manualhome_btn)

        # Sweep one joint through its full homed range (v2 `C sweep_r <j>`,
        # PROTOCOL.md §6) -- a blocking routine like the rest, with a joint
        # picker like Manual Home. Handy for eyeballing a single joint's
        # ROM after the v2 range changes.
        sweep_row = QHBoxLayout()
        self.sweep_joint_combo = QComboBox()
        for joint in JOINTS:
            self.sweep_joint_combo.addItem(joint.name, joint.name)
        sweep_row.addWidget(self.sweep_joint_combo, stretch=1)
        self.sweep_btn = QPushButton("Sweep Joint")
        self.sweep_btn.clicked.connect(self._start_sweep)
        sweep_row.addWidget(self.sweep_btn)
        layout.addLayout(sweep_row)
        self._routine_trigger_buttons.append(self.sweep_btn)

        return group

    def _build_tracking_group(self) -> QWidget:
        group = QGroupBox("Hand Tracking (MediaPipe)")
        outer = QVBoxLayout(group)
        row = QHBoxLayout()
        self.tracking_btn = QPushButton("Start Hand Tracking")
        self.tracking_btn.clicked.connect(self._toggle_tracking)
        row.addWidget(self.tracking_btn, stretch=1)
        self.tracking_led = StatusLED(color="#5b6270")
        row.addWidget(self.tracking_led)
        self.tracking_status_label = QLabel("off")
        self.tracking_status_label.setObjectName("mutedLabel")
        row.addWidget(self.tracking_status_label)
        outer.addLayout(row)

        # Which hand to follow, and which camera. The hand choice applies
        # live; the camera is picked up the next time tracking starts.
        opts = QHBoxLayout()
        opts.addWidget(QLabel("Hand"))
        self.tracking_hand_combo = QComboBox()
        for label, key in (("Auto", "auto"), ("Right", "right"), ("Left", "left")):
            self.tracking_hand_combo.addItem(label, key)
        self.tracking_hand_combo.setToolTip(
            "Which hand to follow. Auto follows whichever hand is found; a left "
            "hand is mirrored onto this right robot hand. Right/Left only track "
            "that hand. Only one hand is detected at a time, so keep the other "
            "hand out of view."
        )
        self.tracking_hand_combo.currentIndexChanged.connect(self._on_tracking_hand_changed)
        opts.addWidget(self.tracking_hand_combo, stretch=1)
        opts.addWidget(QLabel("Camera"))
        self.tracking_camera_combo = QComboBox()
        for i in range(4):
            self.tracking_camera_combo.addItem(str(i), i)
        self.tracking_camera_combo.setToolTip(
            "Webcam index (0 = the system default camera). Takes effect the "
            "next time tracking is started."
        )
        opts.addWidget(self.tracking_camera_combo)
        outer.addLayout(opts)

        # Capture resolution. 640x480 is the default because the camera's
        # frame rate is the biggest single term in tracking latency and
        # almost every webcam manages 30 fps there (see camera.RESOLUTIONS).
        # Applies live: the camera is reopened in the new mode.
        resrow = QHBoxLayout()
        resrow.addWidget(QLabel("Resolution"))
        self.tracking_res_combo = QComboBox()
        for key, (label, _modes) in RESOLUTIONS.items():
            self.tracking_res_combo.addItem(label, key)
        self.tracking_res_combo.setCurrentIndex(
            self.tracking_res_combo.findData(DEFAULT_RESOLUTION))
        self.tracking_res_combo.setToolTip(
            "640x480 (fast): highest frame rate, lowest latency -- recommended.\n"
            "1280x720 (detail): more pixels on the hand when it is far from the "
            "camera, but many webcams only manage 10-15 fps at this size.\n"
            "Auto: 1280x720 if the camera keeps up at 30 fps, else 640x480.\n"
            "Changes apply immediately (the camera is reopened)."
        )
        self.tracking_res_combo.currentIndexChanged.connect(self._on_tracking_res_changed)
        resrow.addWidget(self.tracking_res_combo, stretch=1)
        outer.addLayout(resrow)

        # Wrist: off by default -- it follows the palm's rotation relative to
        # a captured neutral pose, so the operator has to set that pose.
        wrist = QHBoxLayout()
        wrist.addWidget(QLabel("Track wrist"))
        self.tracking_wrist_switch = ToggleSwitch(checked=False)
        self.tracking_wrist_switch.setToolTip(
            "Drive the robot wrist (W_1-W_3) from how your palm rotates relative "
            "to a neutral pose. Turning this on captures the neutral from your "
            "hand as it is at that moment -- hold it in the pose that should "
            "mean 'wrist straight'. Keep your forearm still: the camera sees the "
            "hand, not the arm."
        )
        self.tracking_wrist_switch.toggled.connect(self._on_tracking_wrist_toggled)
        wrist.addWidget(self.tracking_wrist_switch)
        wrist.addStretch(1)
        self.tracking_neutral_btn = QPushButton("Set Wrist Neutral")
        self.tracking_neutral_btn.setToolTip(
            "Re-capture the wrist neutral from your hand's current orientation."
        )
        self.tracking_neutral_btn.setEnabled(False)
        self.tracking_neutral_btn.clicked.connect(self._on_tracking_neutral_clicked)
        wrist.addWidget(self.tracking_neutral_btn)
        outer.addLayout(wrist)

        # Camera speed. The frame rate is the biggest single term in tracking
        # latency (see camera.py), and in dim light most webcams slow
        # themselves down to lengthen the exposure. Short exposure pins it
        # short (darker picture, full frame rate); Camera Settings opens the
        # driver's own page for low-light compensation, gain, anti-flicker.
        camrow = QHBoxLayout()
        camrow.addWidget(QLabel("Short exposure"))
        self.tracking_exposure_switch = ToggleSwitch(checked=False)
        self.tracking_exposure_switch.setToolTip(
            "Fix the camera's exposure short (1/64 s) so it cannot slow down "
            "in dim light. The picture gets darker -- add light if MediaPipe "
            "starts losing the hand. Use this if Diagnostics shows the camera "
            "well below 30 fps."
        )
        self.tracking_exposure_switch.toggled.connect(self._on_tracking_exposure_toggled)
        camrow.addWidget(self.tracking_exposure_switch)
        camrow.addStretch(1)
        self.tracking_camsettings_btn = QPushButton("Camera Settings")
        self.tracking_camsettings_btn.setToolTip(
            "Open the webcam driver's own settings (while tracking). Turn off "
            "'Low light compensation' / 'RightLight' and set anti-flicker to "
            "50 Hz: both can hold the camera at 10-15 fps."
        )
        self.tracking_camsettings_btn.setEnabled(False)
        self.tracking_camsettings_btn.clicked.connect(self._on_tracking_camsettings_clicked)
        camrow.addWidget(self.tracking_camsettings_btn)
        outer.addLayout(camrow)
        return group

    # -- auxiliary panel (middle): sliders OR camera preview, never a
    #    separate window -------------------------------------------------

    def _build_aux_panel(self) -> QWidget:
        """The four side panels, permanently docked on the right as tabs.

        These used to share a QStackedWidget that was hidden until one of
        three "Show ..." buttons revealed it, which meant the whole window
        re-laid-out whenever a panel appeared, and the buttons duplicated
        what a tab bar does natively. They are always visible now: the hand
        is narrow enough that the space was going spare anyway, and a fixed
        layout is calmer to work in than one that jumps.

        _aux_mode and _set_aux_panel keep their old meaning ("which panel is
        showing"), so every existing caller -- starting tracking switches to
        the camera, starting a routine switches to the log -- still works;
        they now switch a tab instead of revealing a panel.
        """
        self.aux_container = QWidget()
        self.aux_container.setFixedWidth(AUX_PANEL_WIDTH)
        outer = QVBoxLayout(self.aux_container)
        outer.setContentsMargins(0, 0, 0, 0)

        self.aux_tabs = QTabWidget()
        self.aux_tabs.setObjectName("auxTabs")
        self.aux_tabs.setDocumentMode(True)
        outer.addWidget(self.aux_tabs)

        self.slider_panel = SliderPanel(on_slider_moved=self._on_slider_moved)
        self.camera_panel = CameraPanel()
        self.camera_panel.filter_switch.toggled.connect(self._on_filter_switch_toggled)
        self.camera_panel.pinch_switch.toggled.connect(self._on_pinch_switch_toggled)
        self.current_graph_panel = CurrentGraphPanel()
        self.routine_panel = RoutinePanel(on_send_input=self._send_routine_input)
        self.diagnostics_panel = DiagnosticsPanel()

        # Diagnostics is deliberately NOT one of these tabs. It is a monitor
        # rather than a thing you switch to: you want to watch the frame
        # rates and current draw *while* dragging a slider or running a
        # routine, which a tab makes impossible. It gets its own column
        # instead -- see _build_diagnostics_dock.
        self._aux_pages = {
            "sliders": self.slider_panel,
            "camera": self.camera_panel,
            "current": self.current_graph_panel,
            "routine": self.routine_panel,
        }
        for label, key in (("Joints", "sliders"), ("Camera", "camera"),
                           ("Current", "current"), ("Log", "routine")):
            self.aux_tabs.addTab(self._aux_pages[key], label)

        self.aux_tabs.currentChanged.connect(self._on_aux_tab_changed)
        # The first addTab above already made Joints the current tab and
        # emitted currentChanged -- before this handler was connected, so
        # nothing heard it and _aux_mode stayed None while the sliders were
        # plainly on screen. Every `_aux_mode == "sliders"` guard then skipped
        # the sync, so pressing a preset moved the hand but not the sliders,
        # until you happened to click another tab and back. Initialise from
        # whatever tab is actually showing.
        self._on_aux_tab_changed(self.aux_tabs.currentIndex())
        return self.aux_container

    def _build_diagnostics_dock(self) -> QWidget:
        """The diagnostics readout as its own column, beside the tabs.

        Deliberately a separate column rather than a fifth tab: its whole
        value is being readable *while* you do something else -- drag a
        slider and watch the current climb, start a routine and watch the
        frame rates -- which a tab rules out by construction. It stays
        inside the one window, like everything else here; the toggle in the
        control panel hides it when the 3D view wants the space back.
        """
        self._diag_container = QWidget()
        row = QHBoxLayout(self._diag_container)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)

        # The handle lives on the LEFT edge, between the tabs and the panel,
        # so collapsing pulls the panel rightward toward the 3D view it is
        # giving space back to. It is never hidden -- collapsed, it is the
        # only way to get the panel back.
        self.diag_tab = VerticalTab("DIAGNOSTICS")
        self.diag_tab.toggled.connect(self._set_diagnostics_open)
        row.addWidget(self.diag_tab)

        # Scrolled, because the readout grows as metrics are added and it is
        # already taller than a 1080p window once the title bar and status
        # bar are taken out. A scrollbar is better than either dropping rows
        # or letting the bottom ones get clipped silently.
        self._diag_scroll = QScrollArea()
        self._diag_scroll.setWidget(self.diagnostics_panel)
        self._diag_scroll.setWidgetResizable(True)
        self._diag_scroll.setFrameShape(QFrame.NoFrame)
        self._diag_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        row.addWidget(self._diag_scroll, stretch=1)

        self._diag_container.setFixedWidth(DIAG_PANEL_WIDTH)
        # Collapsed on startup. Done through the same path the handle and the
        # button use, so the container width, the panel visibility and both
        # controls' captions all start consistent rather than being set up
        # three times in three places.
        self._set_diagnostics_open(False)
        return self._diag_container

    def _set_diagnostics_open(self, open_: bool) -> None:
        """Collapse the diagnostics column down to just its handle.

        Both the handle and the control-panel button drive this, so each is
        re-synced with signals blocked -- otherwise toggling one would echo
        back through the other and fight it."""
        self._diag_scroll.setVisible(open_)
        self._diag_container.setFixedWidth(
            DIAG_PANEL_WIDTH if open_ else self.diag_tab.width()
        )
        for widget, setter in ((self.diag_tab, "setChecked"), (self.diag_btn, "setChecked")):
            blocked = widget.blockSignals(True)
            getattr(widget, setter)(open_)
            widget.blockSignals(blocked)
        self.diag_btn.setText("Hide Diagnostics" if open_ else "Show Diagnostics")
        self.diag_tab.update()

    def _toggle_diagnostics(self, checked: bool) -> None:
        self._set_diagnostics_open(checked)

    def _on_aux_tab_changed(self, index: int) -> None:
        """Keep _aux_mode in step with the tab bar, and refresh the sliders
        when they come back into view.

        The sliders are only kept in sync while they are the visible panel
        (the `_aux_mode == "sliders"` guards around sync_commanded), so
        without this they would show a stale pose after presets or tracking
        moved the hand while another tab was up."""
        widget = self.aux_tabs.widget(index)
        for key, page in self._aux_pages.items():
            if page is widget:
                self._aux_mode = key
                break
        if self._aux_mode == "sliders":
            self.slider_panel.set_read_only(self.tracking_active)
            self.slider_panel.sync_commanded(self.commanded)

    def _set_aux_panel(self, mode: str | None) -> None:
        """Bring a side panel to the front. `None` now means "leave whatever
        is showing alone" rather than "hide the dock": the dock is permanent,
        so there is nothing to hide, and yanking the user's tab away when a
        routine finishes would be worse than leaving it put."""
        page = self._aux_pages.get(mode) if mode else None
        if page is not None:
            self.aux_tabs.setCurrentWidget(page)  # fires _on_aux_tab_changed
        elif mode is None:
            pass

    def _set_tracking_status(self, text: str, color: str) -> None:
        self.tracking_status_label.setText(text)
        self.tracking_led.set_color(color)

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_available_ports()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    # -- connection lifecycle -----------------------------------------------

    def _toggle_connection(self) -> None:
        if self.link is not None and self.link.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "No port", "Select a serial port first.")
            return
        self.link = SerialLink(
            port,
            num_joints=NUM_JOINTS,
            on_error=lambda message: self._link_signals.link_error.emit(message),
            # These four fire from the link's background reader thread
            # (PROTOCOL.md section 6 K/D markers, opaque log text, and
            # E badcmd/badjoint) -- routed through Qt signals rather than
            # touching widgets directly, see LinkSignals.
            on_routine_started=lambda name: self._link_signals.routine_started.emit(name),
            on_routine_log=lambda line: self._link_signals.routine_log.emit(line),
            on_routine_done=lambda name: self._link_signals.routine_done.emit(name),
            on_routine_error=lambda message: self._link_signals.routine_error.emit(message),
            on_faults=lambda faults: self._link_signals.faults.emit(faults),
        )
        try:
            self.link.open()
        except Exception as exc:
            QMessageBox.critical(self, "Connection failed", str(exc))
            self.link = None
            return

        self.connect_btn.setText("Disconnect")
        self.connect_led.set_color("#2ecc71")  # green -- connected
        self.status_bar_label.setText(f"Connected on {port}")
        self.stream_timer.start(STREAM_INTERVAL_MS)

    def _disconnect(self) -> None:
        self.stream_timer.stop()
        if self.link is not None:
            self.link.close()
        self.link = None
        self.connect_btn.setText("Connect")
        self.connect_led.set_color("#e0a020")  # orange -- idle
        self.status_bar_label.setText("Disconnected")
        self.link_health_label.setText("Link: --")
        self.slider_panel.clear_measured()
        self._faults = {}
        self._reseed_from_estop = False
        self._update_safety_panel()
        self._routine_awaiting_start = False
        self._routine_request_token += 1  # invalidate any pending _check_routine_started
        pending = self._resolve_pending_routine()
        if pending in DEMO_NAMES:
            self._reset_demo_button(pending, completed=False)
        self.routine_panel.set_active(None)

    def _on_link_error(self, message: str) -> None:
        self.status_bar_label.setText(f"Link error: {message}")
        if self._routine_awaiting_start:
            # link.py only routes E badcmd / E badjoint through
            # on_routine_error -- those are unambiguously "the routine
            # request was rejected". A generic E line (E unknown,
            # E badframe, E timeout) arriving in this same window is
            # *probably* the hand's actual response to the C command
            # we just sent (e.g. firmware that doesn't recognise `C` at
            # all yet replies E unknown, not a routine-specific error),
            # but link.py can't say that for certain, so it's inferred
            # here from timing instead. Surface it right away rather
            # than leaving things showing "running" until
            # _check_routine_started's timeout catches up.
            self._routine_request_token += 1  # invalidate the pending timeout
            self._routine_awaiting_start = False
            pending = self._resolve_pending_routine()
            if pending in DEMO_NAMES:
                self._reset_demo_button(pending, completed=False)
            else:
                self.routine_panel.append_log(f"-- hand error while starting: {message} --")
                self.routine_panel.set_active(None)

    # -- streaming / measured polling ---------------------------------------

    def _stream_targets(self) -> None:
        """One tick of the 30 Hz lockstep loop (PROTOCOL.md §4): always send.

        Protocol v3 is PC-clocked -- the hand only ever speaks in reply to a
        T, and its 250 ms watchdog freezes it if T frames stop -- so the
        current target goes out on every tick whether or not it changed.

        The link runs a short resync after connecting, after a hand reset and
        after any routine (see link.py): it holds the hand at 1 deg/s, probes
        its real pose, and publishes it as `pending_resync`. Adopting it here,
        in the send tick itself, means the very next T already carries the
        measured pose, so nothing jumps when full speed is restored. Joints
        whose feedback is not trusted keep their current target.
        """
        if self.link is None or not self.link.is_open:
            return
        try:
            probe = self.link.pending_resync
            if probe is not None:
                # After an E-stop reset, start from the targets as they were
                # when the hand stopped: anything sliders or tracking wrote
                # into `commanded` meanwhile was never acted on, and must not
                # leak through on the joints the probe can't re-seed.
                base = self._commanded_at_estop if (self._reseed_from_estop and self._commanded_at_estop) else self.commanded
                if self._reseed_from_estop:
                    self._reseed_from_estop = False
                    self._commanded_at_estop = None
                pose = list(base)
                for i in range(NUM_JOINTS):
                    if probe.trusted(i):
                        pose[i] = probe.positions_deg[i]
                self._set_commanded(pose)
                self.link.ack_resync()
            changed = self.commanded != self._last_sent
            if self.link.send_targets(self.commanded):
                self._diag_counters["stream_sends"] += 1
                if changed:
                    self._diag_counters["stream_changes"] += 1
                self._last_sent = list(self.commanded)
        except Exception as exc:
            self.status_bar_label.setText(f"Send failed: {exc}")

    def _poll_status(self) -> None:
        self._diag_counters["status_polls"] += 1
        if self.link is None or not self.link.is_open:
            return

        status: StatusMessage | None = self.link.last_status
        if status is not None:
            self.slider_panel.update_measured(status.positions_deg)
            self.current_graph_panel.add_sample(status.currents_ma)


        if not self.link.is_pc_mode_confirmed:
            self.link_health_label.setText("Link: HANDSHAKING")
        elif self.link.is_link_stale(LINK_STALE_TIMEOUT_S):
            self.link_health_label.setText("Link: STALE")
        else:
            self.link_health_label.setText("Link: OK")
        self._update_safety_panel()

    # -- protection: E-stop / reset / faults ----------------------------------

    def _on_estop_clicked(self) -> None:
        """E-stop button / Esc. Sends `!` immediately and stops everything on
        this side that would otherwise keep producing targets, so nothing is
        queued up to move the moment the stop is cleared."""
        self._preset_stage_token += 1          # drop a pending staged-thumb move
        if self.tracking_active:
            self._stop_tracking()
        if self.link is None or not self.link.is_open:
            self.status_bar_label.setText("E-stop: not connected -- nothing to stop")
            return
        if self._commanded_at_estop is None:
            self._commanded_at_estop = list(self.commanded)
        if self.link.estop():
            self.status_bar_label.setText("E-STOP sent -- the hand ignores all targets until Reset")
        else:
            self.status_bar_label.setText("E-stop could not be written to the port -- cut the power!")
        # Keep (or get) the 30 Hz loop running: the hand reports the ESTOP flag
        # only in S replies to T frames. During a routine the timer is
        # stopped; ticking it lets the link release a routine whose D never
        # comes (send_targets stays blocked until then, so nothing extra is sent).
        if not self.stream_timer.isActive():
            self.stream_timer.start(STREAM_INTERVAL_MS)
        self._update_safety_panel()

    def _on_reset_clicked(self) -> None:
        if self.link is None or not self.link.is_open:
            return
        if self.tracking_active:
            # The resync after the reset hands control back at the measured
            # pose; live tracking would immediately drive it elsewhere.
            self._stop_tracking()
        self._reseed_from_estop = self._commanded_at_estop is not None
        # Point the targets at where the hand actually is *before* the reset
        # goes out, so even the resync's probe frame (sent at 1 deg/s) asks
        # for no motion. The probe's reply then re-seeds them exactly.
        status = self.link.last_status
        pose = list(self._commanded_at_estop or self.commanded)
        if status is not None:
            for i in range(NUM_JOINTS):
                if status.trusted(i):
                    pose[i] = status.positions_deg[i]
        self._preset_stage_token += 1
        self._set_commanded(pose)
        self.link.reset_protection()
        self.status_bar_label.setText(
            "Reset sent -- the hand holds its pose; targets re-seeded from its measured position")

    def _on_faults(self, faults: dict) -> None:
        self._faults = dict(faults or {})
        self._update_safety_panel()
        joints = self._fault_joints()
        if joints:
            self.status_bar_label.setText(
                "Joint protection stopped " + ", ".join(f"{j} ({r})" for j, r in joints) +
                " -- press Reset to clear")

    def _fault_joints(self) -> list:
        """(joint, reason) pairs from the last `C faults` reply; the reply's
        non-joint keys (estop=...) are left out."""
        names = {j.name for j in JOINTS}
        return [(k, v) for k, v in self._faults.items() if k in names]

    def _update_safety_panel(self) -> None:
        """E-stop/fault indicator, Reset enable, and routine buttons -- driven
        by the flags in the hand's own S replies (polled at 20 Hz)."""
        link = self.link if (self.link is not None and self.link.is_open) else None
        estop = bool(link and link.estop_active)
        fault = bool(link and link.fault_active)
        pending = bool(link and link.estop_pending)

        if link is None:
            text, state = "Not connected", "idle"
        elif estop:
            text, state = "E-STOP ACTIVE -- targets ignored. Press Reset to resume.", "estop"
        elif pending:
            text, state = "E-stop sent, waiting for the hand to confirm...", "estop"
        elif fault:
            joints = self._fault_joints()
            what = ", ".join(f"{j} ({r})" for j, r in joints) if joints else "a joint"
            text, state = f"FAULT: protection stopped {what}.", "fault"
        elif not link.is_pc_mode_confirmed:
            text, state = "Connecting...", "idle"
        else:
            text, state = "Running", "ok"
        if self.safety_label.text() != text:
            self.safety_label.setText(text)
        if self.safety_label.property("state") != state:
            self.safety_label.setProperty("state", state)
            self.safety_label.style().unpolish(self.safety_label)
            self.safety_label.style().polish(self.safety_label)
        self.reset_btn.setEnabled(bool(link) and (estop or fault or pending))
        active = estop or pending
        if self.estop_btn.property("active") != active:
            self.estop_btn.setProperty("active", active)
            self.estop_btn.style().unpolish(self.estop_btn)
            self.estop_btn.style().polish(self.estop_btn)

        # Edges. ESTOP on: routines would be refused (E <seq> estop), so grey
        # them out. ESTOP off (after Reset): forget the snapshot once the
        # resync has used it, and give the buttons back.
        if estop != self._estop_shown:
            self._estop_shown = estop
            if estop:
                if self._commanded_at_estop is None:     # stopped by someone else / on connect
                    self._commanded_at_estop = list(self.commanded)
                if self.tracking_active:
                    self._stop_tracking()
            elif not self._reseed_from_estop:
                self._commanded_at_estop = None
            if self._pending_routine_name is None:
                self._set_routine_controls_enabled(not estop)
        if fault != self._fault_shown:
            self._fault_shown = fault
            if not fault and not estop:
                self._faults = {}

    def _refresh_hand_view(self) -> None:
        # Deliberately on its own slower timer -- see HAND_VIEW_INTERVAL_MS.
        self._diag_counters["view_frames"] += 1
        if self.hand_view_mode == "target":
            # self.commanded is always available locally (slider drag,
            # preset jump, tracked hand), even with no link connected --
            # useful for previewing a pose before a hand is plugged in.
            self.hand_view.set_pose(self.commanded)
            return
        if self.link is None or not self.link.is_open:
            return
        status: StatusMessage | None = self.link.last_status
        if status is not None:
            self.hand_view.set_pose(status.positions_deg)

    # -- manual sliders -------------------------------------------------------

    def _toggle_slider_panel(self) -> None:
        if self._aux_mode == "sliders":
            self._set_aux_panel(None)
            return
        # While tracking, show the sliders *read-only* -- a live view of
        # the target positions the tracker is driving -- rather than
        # stopping tracking to take manual control. (Grabbing a slider is
        # exactly what hands control back to you, so interaction is
        # disabled here; the sliders auto-become interactive again the
        # moment tracking stops -- see _on_vision_stopped.)
        self.slider_panel.set_read_only(self.tracking_active)
        self.slider_panel.sync_commanded(self.commanded)
        self._set_aux_panel("sliders")

    def _toggle_current_panel(self) -> None:
        # Deliberately does NOT stop hand tracking -- watching current
        # draw while the hand is moving (manually or tracked) is exactly
        # when it's most useful, so tracking is left running underneath.
        if self._aux_mode == "current":
            self._set_aux_panel(None)
        else:
            self._set_aux_panel("current")

    def _on_slider_moved(self, joint_index: int, deg: float) -> None:
        if self.tracking_active:
            self._stop_tracking()
        self._preset_stage_token += 1  # cancel any pending staged-thumb apply, see _apply_preset
        self.commanded[joint_index] = deg

    # -- presets ------------------------------------------------------------

    def _apply_preset(self, name: str) -> None:
        """Jump straight to the gesture's target angles in one T frame (or
        two, for THUMB_DELAYED_GESTURES -- see below), at default speed
        (-1 -> firmware default). No PC-side interpolation of the motion
        itself: the firmware's own slew limiter does that, which is both
        simpler and avoids fighting a second interpolator on top of the
        one that already runs on the hand."""
        if self.tracking_active:
            self._stop_tracking()
        self._preset_stage_token += 1
        target = list(GESTURES[name])

        if name not in THUMB_DELAYED_GESTURES:
            self._set_commanded(target)
            return

        # Send the fingers now, hold the thumb where it currently is, and
        # only bring the thumb to its target after THUMB_DELAY_MS -- gives
        # the fingers a head start closing in, so the thumb lands on the
        # *outside* of the fist instead of risking closing in step with
        # (or ahead of) them and getting trapped between them.
        # Take the whole target, then put the thumb back where it is now.
        #
        # This used to be `target[:12] + thumb_hold`, which silently dropped
        # W_1/W_2/W_3: it was written when the hand had 16 joints and the
        # thumb was the last block, so slicing to 12 and appending 4 landed
        # exactly on the end. v2 added three wrist joints after the thumb, so
        # the same expression produced a 16-entry array against a 19-joint
        # protocol. send_targets then raised on EVERY subsequent frame
        # ("expected 19 angles, got 16") and the hand simply stopped
        # receiving targets -- pressing Grip/Fist or Peace Sign froze it
        # until another preset happened to overwrite `commanded` wholesale.
        # Indices come from JOINTS now, so adding joints cannot break it again.
        staged = list(target)
        for i in THUMB_JOINT_INDICES:
            staged[i] = self.commanded[i]
        self._set_commanded(staged)

        token = self._preset_stage_token
        QTimer.singleShot(THUMB_DELAY_MS, lambda: self._finish_staged_preset(target, token))

    def _finish_staged_preset(self, target: list[float], token: int) -> None:
        if token != self._preset_stage_token:
            return  # superseded by a slider move / another preset / tracking since
        self._set_commanded(target)

    # -- hand tracking (MediaPipe) --------------------------------------------

    def _toggle_tracking(self) -> None:
        if self.tracking_active:
            self._stop_tracking()
        else:
            self._start_tracking()

    def _start_tracking(self) -> None:
        self.camera_panel.reset()
        self._set_aux_panel("camera")

        self.vision_thread = QThread(self)
        self.vision_worker = VisionWorker(
            camera_index=int(self.tracking_camera_combo.currentData() or 0),
            filtering_enabled=self.camera_panel.filter_switch.isChecked(),
            hand_preference=self.tracking_hand_combo.currentData() or "auto",
            short_exposure=self.tracking_exposure_switch.isChecked(),
            resolution=self.tracking_res_combo.currentData() or DEFAULT_RESOLUTION,
            wrist_enabled=self.tracking_wrist_switch.isChecked(),
            pinch_snap=self.camera_panel.pinch_switch.isChecked(),
        )
        self.vision_worker.moveToThread(self.vision_thread)

        self.vision_thread.started.connect(self.vision_worker.run)
        # Force Qt.QueuedConnection explicitly rather than relying on
        # Qt.AutoConnection's thread-detection heuristic. angles_ready in
        # particular is emitted from inside MediaPipe's own result
        # callback, which the Tasks API invokes from a C++ thread-pool
        # thread it manages -- not necessarily `vision_thread` itself, and
        # not a thread Qt created via QThread. If AutoConnection ever
        # misjudges that as "same thread" and calls a Qt-widget-touching
        # slot directly instead of queuing it, that's a classic source of
        # intermittent GUI crashes. QueuedConnection removes the ambiguity.
        self.vision_worker.frame_ready.connect(self._on_vision_frame, Qt.QueuedConnection)
        self.vision_worker.camera_notice.connect(
            self.status_bar_label.setText, Qt.QueuedConnection
        )
        self.vision_worker.status_changed.connect(
            self.camera_panel.set_status, Qt.QueuedConnection
        )
        self.vision_worker.angles_ready.connect(self._on_vision_angles, Qt.QueuedConnection)
        self.vision_worker.error.connect(self._on_vision_error, Qt.QueuedConnection)
        self.vision_worker.stopped.connect(self._on_vision_stopped, Qt.QueuedConnection)
        self.vision_worker.stats_ready.connect(
            self.diagnostics_panel.set_vision_stats, Qt.QueuedConnection
        )

        self.vision_thread.start()
        self.tracking_active = True
        self.tracking_neutral_btn.setEnabled(self.tracking_wrist_switch.isChecked())
        self.tracking_camera_combo.setEnabled(False)
        self.tracking_camsettings_btn.setEnabled(True)
        self.tracking_btn.setText("Stop Hand Tracking")
        self._set_tracking_status("starting...", "#e0a020")

    def _stop_tracking(self) -> None:
        if self.vision_worker is not None:
            self.vision_worker.stop()
        self._set_tracking_status("stopping...", "#e0a020")
        # Actual thread teardown happens in _on_vision_stopped, once the
        # worker's capture loop notices the stop flag and exits.

    def _on_vision_frame_status(self) -> None:
        """First frame through means the camera really opened -- flip the
        LED green then, rather than optimistically at button-press."""
        if self.tracking_active and self.tracking_status_label.text() != "on":
            self._set_tracking_status("on", "#2ecc71")

    def _on_vision_frame(self, image: QImage) -> None:
        self.camera_panel.set_frame(image)
        self._on_vision_frame_status()

    def _on_vision_angles(self, angles: list[float]) -> None:
        self._diag_counters["angles"] += 1
        self._preset_stage_token += 1  # cancel any pending staged-thumb apply, see _apply_preset
        self._set_commanded(angles)

    def _on_filter_switch_toggled(self, checked: bool) -> None:
        # The switch persists across tracking sessions; the worker doesn't
        # (a fresh one is created each _start_tracking). Only forward the
        # change if a worker actually exists right now -- flipping the
        # switch while tracking is off just sets the initial state the
        # next VisionWorker(...) will be constructed with.
        if self.vision_worker is not None:
            self.vision_worker.set_filtering_enabled(checked)

    def _on_pinch_switch_toggled(self, checked: bool) -> None:
        # Same pattern as the filter switch: live if tracking, else read by
        # the next VisionWorker.
        if self.vision_worker is not None:
            self.vision_worker.set_pinch_snap(checked)

    def _on_tracking_hand_changed(self, _index: int) -> None:
        # Same pattern as the filter switch: forward live if tracking, else
        # it is simply read when the next worker is constructed.
        if self.vision_worker is not None:
            self.vision_worker.set_hand_preference(self.tracking_hand_combo.currentData() or "auto")

    def _on_tracking_wrist_toggled(self, checked: bool) -> None:
        self.tracking_neutral_btn.setEnabled(checked and self.tracking_active)
        if self.vision_worker is not None:
            self.vision_worker.set_wrist_enabled(checked)

    def _on_tracking_res_changed(self, _index: int) -> None:
        key = self.tracking_res_combo.currentData() or DEFAULT_RESOLUTION
        if self.vision_worker is not None:
            self.vision_worker.set_resolution(key)
            self.status_bar_label.setText(
                f"Switching camera to {self.tracking_res_combo.currentText()}...")

    def _on_tracking_exposure_toggled(self, checked: bool) -> None:
        if self.vision_worker is not None:
            self.vision_worker.set_short_exposure(checked)

    def _on_tracking_camsettings_clicked(self) -> None:
        if self.vision_worker is not None:
            self.vision_worker.request_camera_settings()

    def _on_tracking_neutral_clicked(self) -> None:
        if self.vision_worker is not None:
            self.vision_worker.request_wrist_neutral()
            self.status_bar_label.setText("Capturing wrist neutral -- hold your hand still")

    def _on_vision_error(self, message: str) -> None:
        self.status_bar_label.setText(f"Tracking error: {message}")

    def _on_vision_stopped(self) -> None:
        if self.vision_thread is not None:
            self.vision_thread.quit()
            self.vision_thread.wait(2000)
        self.vision_thread = None
        self.vision_worker = None
        self.tracking_active = False
        self.tracking_neutral_btn.setEnabled(False)
        self.tracking_camera_combo.setEnabled(True)
        self.tracking_camsettings_btn.setEnabled(False)
        self.tracking_btn.setText("Start Hand Tracking")
        self._set_tracking_status("off", "#5b6270")
        self.diagnostics_panel.clear_vision_stats()
        self.camera_panel.set_status("Tracking off", "idle")
        if self._aux_mode == "camera":
            self._set_aux_panel(None)
        elif self._aux_mode == "sliders":
            # Was showing the read-only live-target view -- tracking's over,
            # so hand the sliders back to manual control (they stay on
            # screen, just interactive again). See _toggle_slider_panel.
            self.slider_panel.set_read_only(False)

    # -- built-in firmware routines (PROTOCOL.md section 6) -----------------

    def _toggle_routine_panel(self) -> None:
        if self._aux_mode == "routine":
            self._set_aux_panel(None)
        else:
            self._set_aux_panel("routine")

    def _start_manualhome(self) -> None:
        joint = self.manualhome_joint_combo.currentData()
        self._start_routine("manualhome", joint)

    def _start_sweep(self) -> None:
        joint = self.sweep_joint_combo.currentData()
        self._start_routine("sweep_r", joint)

    def _set_routine_controls_enabled(self, enabled: bool) -> None:
        """Every button that can start a routine -- both Demos buttons
        and everything in Settings / Calibration -- toggled together,
        since the firmware only ever runs one routine at a time
        regardless of which button started it."""
        for btn in self._routine_trigger_buttons:
            btn.setEnabled(enabled)

    def _resolve_pending_routine(self) -> str | None:
        """Clears pending-routine tracking and re-enables every routine-
        trigger button. Called from every path that resolves a request
        (K/D, an error, or _check_routine_started's timeout). Returns the
        name that *was* pending so the caller can decide whether to reset
        a Demos ProgressButton or the Routine Log panel -- see
        DEMO_NAMES."""
        name = self._pending_routine_name
        self._pending_routine_name = None
        # Stay greyed out while the E-stop is latched: the hand would refuse.
        self._set_routine_controls_enabled(not self._estop_shown)
        return name

    def _start_routine(self, name: str, arg: str | None = None) -> None:
        if self.link is None or not self.link.is_open:
            if name in DEMO_NAMES:
                self.status_bar_label.setText("Cannot start demo: not connected")
            else:
                self.routine_panel.append_log("cannot start routine: not connected")
            return

        self._pending_routine_name = name
        self._set_routine_controls_enabled(False)
        if name not in DEMO_NAMES:
            # Optimistic: opens the panel and shows "Running: <name>"
            # immediately rather than waiting for the K marker round-trip.
            # Corrected back to idle by _on_routine_error if the hand
            # rejects the command cleanly (E badcmd / E badjoint), or by
            # _check_routine_started's timeout if K never arrives at all.
            # Demos skip all of this -- no log, no panel, just the
            # ProgressButton itself; see _on_demo_started, which only
            # starts the actual fill sweep once K confirms it.
            self.routine_panel.set_active(name)
            cmd = f"C {name}" + (f" {arg}" if arg else "")
            self.routine_panel.append_log(f"-- requesting: {cmd} --")
            self._set_aux_panel("routine")
        self.link.run_routine(name, arg)

        self._routine_awaiting_start = True
        self._routine_request_token += 1
        token = self._routine_request_token
        QTimer.singleShot(ROUTINE_START_TIMEOUT_MS, lambda t=token: self._check_routine_started(t))

    def _check_routine_started(self, token: int) -> None:
        if token != self._routine_request_token:
            return  # superseded by K / D / an explicit error already
        self._routine_awaiting_start = False
        pending = self._resolve_pending_routine()
        if pending in DEMO_NAMES:
            self._reset_demo_button(pending, completed=False)
            self.status_bar_label.setText(
                f"No response starting {pending!r} -- assuming it didn't start"
            )
        else:
            self.routine_panel.append_log(
                "-- no response from the hand at all (no K, no E line either) -- "
                "assuming it didn't start --"
            )
            self.routine_panel.set_active(None)

    def _send_routine_input(self, text: str) -> None:
        if self.link is None or not self.link.is_open:
            return
        self.link.send_routine_input(text)

    def _on_demo_started(self, name: str) -> None:
        """K confirmed a demo actually started -- begin its progress
        button's fill sweep, timed against DEMO_DURATIONS' estimate
        (cosmetic only; _on_routine_done snaps it to 100% and resets on
        the real D marker regardless of how the estimate compares)."""
        btn = self._demo_buttons.get(name)
        if btn is None:
            return
        duration = DEMO_DURATIONS.get(name, 10.0)
        start_time = time.monotonic()
        timer = QTimer(self)
        timer.setInterval(80)
        timer.timeout.connect(
            lambda: btn.set_progress((time.monotonic() - start_time) / duration)
        )
        timer.start()
        self._demo_progress_timers[name] = timer

    def _reset_demo_button(self, name: str, *, completed: bool) -> None:
        timer = self._demo_progress_timers.pop(name, None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
        btn = self._demo_buttons.get(name)
        if btn is None:
            return
        if completed:
            # Briefly show a full green bar on a real D marker before
            # resetting, rather than snapping straight back to empty --
            # a clearer "done" signal than the button just going blank.
            btn.set_progress(1.0)
            QTimer.singleShot(250, btn.reset)
        else:
            btn.reset()

    def _on_routine_started(self, name: str) -> None:
        self._routine_request_token += 1  # invalidate _check_routine_started's timeout
        self._routine_awaiting_start = False
        # Confirmed via the K marker (as opposed to _start_routine's
        # optimistic guess, which may not even include e.g. the manualhome
        # joint arg) -- this is the authoritative state.
        if name in DEMO_NAMES:
            self._on_demo_started(name)
        else:
            self.routine_panel.set_active(name)
            self.routine_panel.append_log(f"-- started: {name} --")
            if self._aux_mode != "routine":
                self._set_aux_panel("routine")
        # PROTOCOL.md section 6: routines block the link completely -- stop
        # our own T-frame streaming for the duration. is_link_stale() and
        # send_targets() both already guard on active_routine as a
        # backstop, but the send timer should still not be firing at all.
        self.stream_timer.stop()

    def _on_routine_log(self, line: str) -> None:
        # Opaque log/prompt text while a routine runs -- includes
        # manualhome's per-joint jog prompts, which the panel's input row
        # (enabled by set_active("manualhome")) replies to. Harmless to
        # always append even during a demo (which doesn't otherwise use
        # this panel) -- it just won't be visible unless the user opens
        # the Routine Log panel manually while one happens to be running.
        self.routine_panel.append_log(line)

    def _on_routine_done(self, name: str) -> None:
        self._routine_request_token += 1  # invalidate _check_routine_started's timeout
        self._routine_awaiting_start = False
        self._resolve_pending_routine()
        stopped = self.link is not None and (self.link.estop_pending or self.link.estop_active)
        if name in DEMO_NAMES:
            self._reset_demo_button(name, completed=not stopped)
        else:
            self.routine_panel.set_active(None)
            self.routine_panel.append_log(f"-- {'stopped by E-stop' if stopped else 'done'}: {name} --")
        if self.link is not None and self.link.is_open:
            # Resume the heartbeat straight away. It must not wait for an S
            # frame first: in v3 no S comes back unless a T is sent. The link
            # resyncs (1 deg/s probe, then _stream_targets adopts the measured
            # pose) before full speed, so the stale target can't yank the hand.
            self.stream_timer.start(STREAM_INTERVAL_MS)

    def _on_routine_error(self, message: str) -> None:
        self._routine_request_token += 1  # invalidate _check_routine_started's timeout
        self._routine_awaiting_start = False
        pending = self._resolve_pending_routine()
        if pending in DEMO_NAMES:
            self._reset_demo_button(pending, completed=False)
        else:
            self.routine_panel.append_log(f"ERROR: {message}")
            self.routine_panel.set_active(None)
        self.status_bar_label.setText(f"Routine error: {message}")
        if self.link is not None and self.link.is_open and not self.stream_timer.isActive():
            # The routine was rejected, or had started (streaming stopped for
            # it) and then died without a D -- e.g. the hand reset mid-routine.
            # Resume the heartbeat; the link resyncs before full speed.
            self.stream_timer.start(STREAM_INTERVAL_MS)

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        if self.tracking_active:
            self._stop_tracking()
            if self.vision_thread is not None:
                self.vision_thread.wait(2000)
        self._disconnect()
        super().closeEvent(event)


def _install_crash_diagnostics() -> None:
    """Print uncaught exceptions instead of letting them vanish silently.

    PySide6 slots invoked via Qt's C++ signal machinery (e.g. a QThread's
    `started` signal calling a worker's `run()`) don't always surface a
    Python traceback the normal way if something inside raises -- the app
    can just disappear. This makes sure *something* gets printed to the
    terminal, which is the only way to tell "it crashed because of X" from
    "it crashed for no visible reason" from the outside.
    """
    import traceback

    def _hook(exc_type, exc_value, exc_tb):
        print("=== Uncaught exception (see traceback below) ===", file=sys.stderr)
        traceback.print_exception(exc_type, exc_value, exc_tb)

    sys.excepthook = _hook


def main() -> None:
    _install_crash_diagnostics()
    app = QApplication(sys.argv)
    # Fusion renders custom QSS consistently (native styles like Windows'
    # "windowsvista" handle stylesheets on things like QComboBox/QSlider
    # inconsistently) -- see theme.py's module docstring.
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)
    window = MainWindow()
    window.resize(1200, 780)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
