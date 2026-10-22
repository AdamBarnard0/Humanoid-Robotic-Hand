"""Small reusable Qt widgets shared across gui.py.

Kept separate from gui.py (which is already the largest file) so these
stay easy to find and reuse: a toggle switch (Measured/Target, hand-
tracking filtering), a small circular status LED (connection state), and
a progress-filling button (the Demos block).
"""
from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QAbstractButton, QPushButton, QSizePolicy, QWidget


class VerticalTab(QAbstractButton):
    """A thin vertical tab that collapses/expands the panel beside it.

    Checkable: checked = the panel is open. It stays visible when the panel
    is collapsed -- that is the whole point, since it is then the only way
    back. Drawn rather than styled because Qt cannot rotate the text of an
    ordinary QPushButton, and a sideways label is what makes a 24px-wide
    handle readable at all.

    The chevron points the way the panel will move: toward the panel when
    open (click to close it), away when closed (click to reopen).
    """

    _WIDTH = 24

    def __init__(self, text: str, parent=None, accent: str = "#4f8cff"):
        super().__init__(parent)
        self._text = text
        self._accent = QColor(accent)
        self.setCheckable(True)
        self.setChecked(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedWidth(self._WIDTH)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        self.setToolTip("Collapse or expand the diagnostics panel")

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        w, h = self.width(), self.height()
        hovered = self.underMouse()

        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#242932" if hovered else "#20242c"))
        p.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), 5, 5)
        # Accent edge along the panel side, brightest when open.
        p.setBrush(self._accent if self.isChecked() else QColor("#3a4150"))
        p.drawRoundedRect(QRectF(w - 3.0, 6, 2.0, max(h - 12, 1)), 1, 1)

        font = p.font()
        font.setPointSizeF(max(font.pointSizeF() - 1.0, 6.0))
        font.setBold(True)
        p.setFont(font)
        p.setPen(QColor("#e4e7ec") if (hovered or self.isChecked()) else QColor("#8b93a1"))

        # Rotate about the widget's centre and draw the label along it.
        p.save()
        p.translate(w / 2.0, h / 2.0)
        p.rotate(-90)
        chevron = "›" if self.isChecked() else "‹"  # > when open, < when closed
        label = f"{chevron}  {self._text}"
        p.drawText(QRectF(-h / 2.0, -w / 2.0, h, w), Qt.AlignCenter, label)
        p.restore()
        p.end()

    def enterEvent(self, event) -> None:  # noqa: N802
        self.update()
        super().enterEvent(event)

    def leaveEvent(self, event) -> None:  # noqa: N802
        self.update()
        super().leaveEvent(event)

    def sizeHint(self):  # noqa: N802
        from PySide6.QtCore import QSize
        return QSize(self._WIDTH, 220)


class MouseGlyph(QWidget):
    """A small drawn mouse with one button (or the wheel) highlighted.

    Drawn with QPainter rather than shipped as icon files: it is a handful
    of rounded rectangles, it inherits the theme's colours directly, and it
    stays crisp at any DPI without needing @2x assets. Purely decorative --
    it explains the 3D view's mouse bindings and is not interactive, so it
    deliberately has no mouse handling of its own.

    `part` is one of "left", "right", "middle", "scroll".
    """

    _W, _H = 20, 30

    def __init__(self, part: str, parent=None, accent: str = "#4f8cff",
                 outline: str = "#5b6270"):
        super().__init__(parent)
        self._part = part
        self._accent = QColor(accent)
        self._outline = QColor(outline)
        self.setFixedSize(self._W, self._H)

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        body = QRectF(2.5, 1.5, self._W - 5, self._H - 3)
        split = body.top() + body.height() * 0.42   # button/body divider
        mid_x = body.center().x()

        # Highlight the active button's region first, clipped to the body's
        # rounded outline so the fill follows the mouse's shape.
        clip = QPainterPath()
        clip.addRoundedRect(body, 7, 7)
        p.save()
        p.setClipPath(clip)
        p.setPen(Qt.NoPen)
        p.setBrush(self._accent)
        if self._part == "left":
            p.drawRect(QRectF(body.left(), body.top(), body.width() / 2, split - body.top()))
        elif self._part == "right":
            p.drawRect(QRectF(mid_x, body.top(), body.width() / 2, split - body.top()))
        p.restore()

        # Body outline + the divider between the two buttons.
        pen = QPen(self._outline, 1.2)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(body, 7, 7)
        p.drawLine(QPointF(body.left(), split), QPointF(body.right(), split))
        p.drawLine(QPointF(mid_x, body.top() + 1.5), QPointF(mid_x, split))

        # Scroll wheel, filled when it is the part being described. The
        # wheel is drawn shorter for "scroll" to leave room for a chevron
        # above AND below it -- there is only ~11 px between the top of the
        # body and the button divider, so a full-height wheel crowds them
        # out and only the top one lands.
        scrolling = self._part == "scroll"
        wheel_h = 4.4 if scrolling else 7.0
        wheel = QRectF(mid_x - 1.9, body.top() + (5.0 if scrolling else 4.0), 3.8, wheel_h)
        wheel_on = self._part in ("middle", "scroll")
        p.setBrush(self._accent if wheel_on else Qt.NoBrush)
        p.setPen(QPen(self._accent if wheel_on else self._outline, 1.1))
        p.drawRoundedRect(wheel, 1.9, 1.9)

        if scrolling:
            p.setBrush(self._accent)
            p.setPen(Qt.NoPen)
            for base, tip in ((wheel.top() - 1.1, -1.9), (wheel.bottom() + 1.1, 1.9)):
                p.drawPolygon(QPolygonF([
                    QPointF(mid_x, base + tip),
                    QPointF(mid_x - 2.1, base),
                    QPointF(mid_x + 2.1, base),
                ]))
        p.end()


class ToggleSwitch(QAbstractButton):
    """A small iOS-style sliding switch. Checkable QAbstractButton with a
    custom paintEvent instead of a styled QCheckBox -- Qt's stylesheet
    system can fake a switch look but not the sliding-knob animation
    feel, and this is little enough code to just own directly.

    Usage is identical to any other checkable button: `setChecked()`,
    `isChecked()`, `toggled` signal.
    """

    def __init__(self, parent=None, checked: bool = False):
        super().__init__(parent)
        self.setCheckable(True)
        self.setChecked(checked)
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setFixedSize(44, 24)

    def sizeHint(self):
        return self.size()

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)

        rect = QRectF(0, 0, self.width(), self.height())
        radius = rect.height() / 2

        on = self.isChecked()
        track_color = QColor("#4caf50") if on else QColor("#5a5a5a")
        if not self.isEnabled():
            track_color = track_color.darker(150)
        painter.setBrush(track_color)
        painter.drawRoundedRect(rect, radius, radius)

        knob_d = rect.height() - 4
        knob_x = rect.width() - knob_d - 2 if on else 2
        painter.setBrush(QColor("#f0f0f0"))
        painter.drawEllipse(QRectF(knob_x, 2, knob_d, knob_d))


class StatusLED(QWidget):
    """A small circular status indicator, e.g. next to the Connect button
    for at-a-glance connection state (green = connected, orange =
    idle/disconnected). Deliberately separate from the more detailed
    "Link: OK/STALE" label elsewhere in the connection group -- that one
    reflects S-frame telemetry freshness once connected; this one just
    answers "is the port open at all", which stays meaningful even before
    the first S frame ever arrives."""

    def __init__(self, parent=None, color: str = "#e0a020"):
        super().__init__(parent)
        self._color = QColor(color)
        self.setFixedSize(14, 14)

    def set_color(self, color: str) -> None:
        self._color = QColor(color)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._color)
        d = min(self.width(), self.height()) - 2
        painter.drawEllipse(QRectF(1, 1, d, d))


class ProgressButton(QPushButton):
    """A QPushButton that can also show 0..1 fill progress as a green
    sweep from left to right -- via a stylesheet-only qlineargradient
    trick rather than a custom paintEvent, so it keeps the app's normal
    QPushButton hover/pressed/disabled chrome (from the global
    stylesheet) instead of having to reimplement all of that too.

    Used for the Demos block: gives at-a-glance progress for a firmware
    demo routine without needing to open the routine log, which demos
    deliberately don't use (see gui.py's DEMO_NAMES)."""

    _FILL_COLOR = "#2ecc71"

    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self._progress = 0.0

    def set_progress(self, fraction: float) -> None:
        self._progress = max(0.0, min(1.0, fraction))
        if self._progress <= 0.0:
            self.setStyleSheet("")
        elif self._progress >= 1.0:
            self.setStyleSheet(f"QPushButton {{ background-color: {self._FILL_COLOR}; }}")
        else:
            # Hard-edged two-color gradient, not a smooth blend: a solid
            # sweep reads as "progress" at a glance, a blur wouldn't.
            stop = f"{self._progress:.4f}"
            self.setStyleSheet(
                "QPushButton { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
                f"stop:0 {self._FILL_COLOR}, stop:{stop} {self._FILL_COLOR}, "
                f"stop:{stop} transparent, stop:1 transparent); }}"
            )

    def reset(self) -> None:
        """Back to 0 progress. Does not touch enabled state -- callers
        that disabled this button for the duration of a routine request
        (see gui.py's _set_routine_controls_enabled) re-enable it
        separately, alongside every other routine-trigger button at
        once, since only one routine can run on the hand at a time."""
        self.set_progress(0.0)
