"""App-wide dark theme, applied once via `app.setStyleSheet(APP_STYLESHEET)`
in `gui.main()`.

Kept as a single QSS string rather than scattered inline `setStyleSheet()`
calls per-widget (a few of those still exist for one-off cases, e.g. the
measured-position bar's yellow fill in gui.py, which is specific enough
to that one widget that it isn't worth a global selector) so the overall
look stays consistent and is editable in one place.

`gui.main()` also sets `QApplication.setStyle("Fusion")` before applying
this -- custom QSS on top of a native platform style (e.g. Windows'
"windowsvista") renders inconsistently for things like QComboBox drop-
downs and QSlider handles; Fusion respects stylesheets predictably across
platforms, which is what actually matters here since this only ever runs
on Windows in practice but the inconsistency risk is real either way.

Not visually verified against a real render -- this dev sandbox has no
display (see README's hardware-testing notes elsewhere in this project
for the same caveat applied to the routine feature). Colors and spacing
here are a reasonable-by-construction dark theme, not something checked
pixel-by-pixel; if something looks off (contrast, alignment, a control
that doesn't pick up the stylesheet the way it's expected to), it's
worth a look on a real screen before assuming the intent below is wrong.
"""
from __future__ import annotations

# -- palette --------------------------------------------------------------
# Named here once so the QSS below reads as "what" rather than a wall of
# hex codes; not exposed as a public API, just local documentation.
_BG = "#1b1e24"          # window background
_PANEL_BG = "#20242c"    # group boxes, panels, text views (matches the
                          # current-draw graph's existing plot background,
                          # chosen to match rather than introduce a third
                          # dark shade)
_INPUT_BG = "#171a20"    # sunken fields: combo boxes, line edits, text edits
_BORDER = "#343a46"      # default border / divider color
_BORDER_SOFT = "#2a2f38"
_TEXT = "#e4e7ec"
_TEXT_MUTED = "#8b93a1"
_TEXT_DIM = "#5b6270"
_ACCENT = "#4f8cff"      # primary interactive accent (focus, slider, links)
_ACCENT_HOVER = "#6ba0ff"
_ACCENT_PRESSED = "#3d72db"
_SUCCESS = "#2ecc71"     # matches ProgressButton._FILL_COLOR / ToggleSwitch's "on" green
_WARNING = "#e0a020"     # matches StatusLED's idle color / measured-bar yellow family
_DANGER = "#e0544c"

APP_STYLESHEET = f"""
* {{
    font-family: "Segoe UI", "Inter", "Helvetica Neue", sans-serif;
    font-size: 13px;
}}

QMainWindow, QWidget {{
    background-color: {_BG};
    color: {_TEXT};
}}

QToolTip {{
    background-color: {_PANEL_BG};
    color: {_TEXT};
    border: 1px solid {_BORDER};
    padding: 4px 6px;
}}

/* -- group boxes: the main structural unit of the control panel -- */
QGroupBox {{
    background-color: {_PANEL_BG};
    border: 1px solid {_BORDER};
    border-radius: 8px;
    margin-top: 16px;
    padding: 12px 8px 8px 8px;
    font-weight: 600;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    top: 2px;
    padding: 0 6px;
    color: {_TEXT_MUTED};
}}

/* -- buttons -- */
/* min-height (on top of padding) works around a Qt/Fusion quirk where
   QPushButton's auto sizeHint under a QSS border-radius+padding combo
   doesn't always reserve enough room for a font's descenders -- without
   it, text like "Zero (Open)" or "Grip / Fist" can render with its
   bottom edge clipped against the button's rounded border. Padding alone
   wasn't enough; min-height forces the actual widget rect to be tall
   enough regardless. */
QPushButton {{
    background-color: #2a2f38;
    border: 1px solid {_BORDER};
    border-radius: 6px;
    padding: 8px 14px;
    min-height: 20px;
    color: {_TEXT};
}}
/* The Demo buttons carry a two-line label ("Demo 1" over "Movement"). The
   one-line min-height above is what the control column sized them to,
   clipping both lines; this reserves two lines of 13 px text. */
QPushButton#demoButton {{
    min-height: 36px;
}}
QPushButton:hover {{
    background-color: #333a45;
    border-color: {_ACCENT};
}}
QPushButton:pressed {{
    background-color: #22262e;
}}
QPushButton:disabled {{
    color: {_TEXT_DIM};
    background-color: #202329;
    border-color: {_BORDER_SOFT};
}}
QPushButton:checked {{
    background-color: #2e3a4d;
    border-color: {_ACCENT};
}}

/* Primary action -- currently just Connect/Disconnect, set via
   objectName in gui.py so it stands out as the one button you press
   first. */
QPushButton#primaryButton {{
    background-color: {_ACCENT};
    border-color: {_ACCENT};
    color: #0c111b;
    font-weight: 600;
}}
QPushButton#primaryButton:hover {{ background-color: {_ACCENT_HOVER}; }}
QPushButton#primaryButton:pressed {{ background-color: {_ACCENT_PRESSED}; }}

/* -- inputs -- */
/* min-height for the same reason as QPushButton above -- QComboBox in
   particular (the port and joint dropdowns) is prone to the same
   descender-clipping quirk. */
QComboBox, QLineEdit {{
    background-color: {_INPUT_BG};
    border: 1px solid {_BORDER};
    border-radius: 6px;
    padding: 5px 8px;
    min-height: 18px;
    color: {_TEXT};
    selection-background-color: {_ACCENT};
    selection-color: #0c111b;
}}
QTextEdit {{
    background-color: {_INPUT_BG};
    border: 1px solid {_BORDER};
    border-radius: 6px;
    padding: 5px 8px;
    color: {_TEXT};
    selection-background-color: {_ACCENT};
    selection-color: #0c111b;
}}
QComboBox:focus, QLineEdit:focus, QTextEdit:focus {{
    border-color: {_ACCENT};
}}
QComboBox::drop-down {{
    border: none;
    width: 22px;
}}
QComboBox QAbstractItemView {{
    background-color: {_INPUT_BG};
    border: 1px solid {_BORDER};
    selection-background-color: {_ACCENT};
    selection-color: #0c111b;
    outline: none;
}}
QLineEdit:disabled, QTextEdit:disabled {{
    color: {_TEXT_DIM};
    background-color: #1a1d23;
}}

/* -- labels -- */
QLabel {{
    color: {_TEXT};
    background: transparent;
}}
QLabel#panelHeader {{
    font-weight: 700;
    font-size: 14px;
    color: #f2f4f7;
    padding: 4px 2px;
}}
QLabel#panelNote {{
    color: {_TEXT_MUTED};
    font-size: 11px;
    padding: 0 2px 6px 2px;
}}
QLabel#routineStatus {{
    font-weight: 600;
    padding: 0 2px 6px 2px;
    color: {_TEXT_MUTED};
}}
QLabel#routineStatus[running="true"] {{
    color: {_WARNING};
}}

/* -- slider (manual joint control) -- */
/* The handle is drawn larger than the groove via a negative margin (a
   standard Qt technique for a chunky, easy-to-grab handle on a thin
   track) -- but that only works if the QSlider widget itself is tall
   enough to contain the overhang; Qt clips child painting to a widget's
   own rect, it doesn't grow the widget to fit an oversized handle drawn
   via QSS. min-height below is what actually prevents the clipping;
   the groove/handle/margin numbers are kept in exact proportion
   (handle 12px, groove 4px, margin -4px = (12-4)/2, so the handle sits
   exactly centered on the groove with no left-over rounding) so there's
   no accidental extra overhang on one side to begin with. */
QSlider {{
    min-height: 20px;
}}
QSlider::groove:horizontal {{
    height: 4px;
    background: {_BORDER};
    border-radius: 2px;
}}
QSlider::sub-page:horizontal {{
    background: {_ACCENT};
    border-radius: 2px;
}}
QSlider::add-page:horizontal {{
    background: {_BORDER};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {_ACCENT};
    width: 12px;
    height: 12px;
    margin: -4px 0;
    border-radius: 6px;
}}
QSlider::handle:horizontal:hover {{
    background: {_ACCENT_HOVER};
}}

/* While hand tracking drives the sliders they are read-only, and a visible
   handle is a lie: it says "grab me" for a control that ignores the mouse.
   Hiding just the handle leaves the accent-filled groove, so the row still
   reads as a live target bar rather than a dead one. Geometry is kept
   (width/margin unchanged) so the fill lands in exactly the same place --
   only the dot disappears. Driven by a dynamic property, see
   JointRow.set_read_only. */
QSlider[readOnly="true"]::handle:horizontal,
QSlider[readOnly="true"]::handle:horizontal:hover {{
    background: transparent;
    border: none;
}}

/* -- scrollbars: thin, unobtrusive -- */
QScrollBar:vertical {{
    background: transparent;
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: #3a4048;
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{
    background: #4a5058;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QScrollBar:horizontal {{
    background: transparent;
    height: 10px;
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background: #3a4048;
    border-radius: 4px;
    min-width: 24px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0;
}}

QScrollArea {{
    border: none;
    background: transparent;
}}

/* -- status bar -- */
QStatusBar {{
    background-color: {_INPUT_BG};
    color: {_TEXT_MUTED};
    border-top: 1px solid {_BORDER};
}}

/* -- log / text views -- */
QTextEdit#logView {{
    font-family: Consolas, "Cascadia Mono", monospace;
    font-size: 11px;
    background-color: {_INPUT_BG};
}}

QProgressBar {{
    border: none;
    border-radius: 2px;
    background-color: #3a3a3a;
}}

/* -- docked side panels (right-hand tabs) --
   documentMode is on in gui.py, so the tab bar draws as a flat strip and
   the styling here only has to describe the tabs themselves. The selected
   tab is marked with an accent underline rather than a raised "folder"
   shape, which reads better against a flat dark panel. */
QTabWidget#auxTabs::pane {{
    border: 1px solid {_BORDER};
    border-radius: 8px;
    background-color: {_PANEL_BG};
    top: -1px;
}}
QTabWidget#auxTabs > QTabBar {{
    qproperty-drawBase: 0;
}}
/* Horizontal padding is tight on purpose: five tabs have to fit inside
   AUX_PANEL_WIDTH, and at 14px they overflowed by 25px, which made Qt add
   scroll arrows and push the first tab out of view. */
QTabWidget#auxTabs > QTabBar::tab {{
    background: transparent;
    color: {_TEXT_MUTED};
    border: none;
    border-bottom: 2px solid transparent;
    padding: 7px 8px;
    margin-right: 1px;
    font-weight: 600;
}}
QTabWidget#auxTabs > QTabBar::tab:hover {{
    color: {_TEXT};
}}
QTabWidget#auxTabs > QTabBar::tab:selected {{
    color: {_TEXT};
    border-bottom: 2px solid {_ACCENT};
}}

/* -- 3D view camera toolbar --
   Compact square-ish buttons down the right edge of the hand view, under
   the interactive orientation gizmo. Deliberately quieter than the
   control-panel buttons: these are navigation, not actions, so they
   shouldn't compete with Connect or the presets for attention. */
QWidget#viewToolbar {{
    background: transparent;
}}
/* Mouse-binding legend under the view buttons. Quieter than a normal
   group box -- it is a reference card, not a control. */
QGroupBox#mouseHelp {{
    background-color: rgba(32, 36, 44, 0.75);
    border: 1px solid {_BORDER_SOFT};
    border-radius: 6px;
    margin-top: 14px;
    padding: 4px 2px 2px 2px;
    font-size: 10px;
    font-weight: 700;
}}
QGroupBox#mouseHelp::title {{
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 0 4px;
    color: {_TEXT_DIM};
}}
QLabel#mouseHelpLabel {{
    color: {_TEXT_MUTED};
    font-size: 10px;
}}

QLabel#toolbarCaption {{
    color: {_TEXT_DIM};
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
    padding-bottom: 2px;
}}
QPushButton#viewButton {{
    background-color: rgba(42, 47, 56, 0.75);
    border: 1px solid {_BORDER_SOFT};
    border-radius: 5px;
    padding: 6px 4px;
    min-height: 16px;
    font-size: 11px;
    color: {_TEXT_MUTED};
}}
QPushButton#viewButton:hover {{
    color: {_TEXT};
    border-color: {_ACCENT};
    background-color: #2f3640;
}}
QPushButton#viewButton:pressed {{
    background-color: #22262e;
}}

QLabel#mutedLabel {{
    color: {_TEXT_MUTED};
}}

/* -- hand-tracking status line above the camera preview --
   Deliberately large and bold: it replaces text that used to be burned into
   the preview image and shrank with it to near-illegibility. The left bar
   and colour carry the state, so it reads at a glance without parsing. */
QLabel#trackingStatus {{
    font-size: 15px;
    font-weight: 700;
    padding: 6px 10px;
    margin: 2px 0 4px 0;
    border-radius: 6px;
    border-left: 4px solid {_TEXT_DIM};
    background-color: {_PANEL_BG};
    color: {_TEXT_MUTED};
}}
QLabel#trackingStatus[state="good"] {{
    border-left-color: {_SUCCESS};
    color: {_TEXT};
}}
QLabel#trackingStatus[state="pending"] {{
    border-left-color: {_WARNING};
    color: {_TEXT};
}}
QLabel#trackingStatus[state="warn"] {{
    border-left-color: {_DANGER};
    color: {_TEXT};
}}

/* -- E-stop and protection status (top of the control panel) --
   Always red, large and bold so it is found without looking for it. While
   the hand reports the E-stop latched ([active="true"]) it turns solid dark
   red with a bright border, so a stopped hand is unmistakable. */
QPushButton#estopButton {{
    background-color: #c62828;
    border: 2px solid #ff5252;
    border-radius: 8px;
    color: #ffffff;
    font-size: 18px;
    font-weight: 800;
    letter-spacing: 2px;
    min-height: 44px;
    padding: 6px;
}}
QPushButton#estopButton:hover {{ background-color: #d32f2f; }}
QPushButton#estopButton:pressed {{ background-color: #8e0000; }}
QPushButton#estopButton[active="true"] {{
    background-color: #7f0000;
    border: 3px solid #ffeb3b;
}}
QLabel#safetyStatus {{
    font-weight: 700;
    padding: 4px 8px;
    border-radius: 6px;
    border-left: 4px solid {_TEXT_DIM};
    background-color: {_PANEL_BG};
    color: {_TEXT_MUTED};
}}
QLabel#safetyStatus[state="ok"] {{
    border-left-color: {_SUCCESS};
    color: {_TEXT};
}}
QLabel#safetyStatus[state="fault"] {{
    border-left-color: {_WARNING};
    color: {_WARNING};
}}
QLabel#safetyStatus[state="estop"] {{
    border-left-color: #ff5252;
    background-color: #3a1214;
    color: #ff8a80;
}}

/* -- diagnostics readout --
   Values are tabular figures in a monospace face so digits do not jitter
   sideways as they change; at a 2 Hz refresh, proportional digits shifting
   under each other is genuinely distracting to read. */
QLabel#diagSection {{
    color: {_TEXT_DIM};
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
    padding: 10px 2px 2px 2px;
    border-bottom: 1px solid {_BORDER_SOFT};
}}
QLabel#diagLabel {{
    color: {_TEXT_MUTED};
    padding: 3px 2px;
}}
QLabel#diagValue {{
    color: {_TEXT};
    font-family: Consolas, "Cascadia Mono", monospace;
    font-size: 12px;
    padding: 3px 2px;
}}
QLabel#diagValue[state="good"] {{
    color: {_SUCCESS};
    font-weight: 600;
}}
QLabel#diagValue[state="idle"] {{
    color: {_TEXT_DIM};
}}

/* -- control-panel masthead -- */
QLabel#appTitle {{
    font-size: 40px;
    font-weight: 800;
    color: #f2f4f7;
    letter-spacing: 3px;
    padding: 2px 2px 0 2px;
}}
QLabel#appSubtitle {{
    font-size: 11px;
    color: {_TEXT_DIM};
    letter-spacing: 0.5px;
    padding: 0 3px 6px 3px;
}}

/* Breathing room inside the docked tab pages, so panel content isn't
   flush against the pane border. */
QTabWidget#auxTabs > QStackedWidget > QWidget {{
    padding: 2px;
}}

QScrollArea#controlScroll {{
    background: transparent;
    border: none;
}}
"""
