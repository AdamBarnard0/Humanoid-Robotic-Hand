"""Serial transport to the HRH1 hand -- HRH1 Serial Protocol **v3**.

PROTOCOL.md is the wire specification; it is taken from the firmware as built
(`src/Protocol.cpp`, `src/main.cpp`, `include/Config.h` section 11). What this
module implements, briefly:

- **921600 baud**, 8N1, ASCII, one frame per line. Hand->PC lines end `\\r\\n`.
- **Handshake.** The firmware boots into TERMINAL mode (a human menu). `pc\\n`
  is sent every 500 ms until the exact line `MODE PC` arrives. `pc` is
  idempotent and resets the hand's session. It is also how we recover after
  the hand resets (brown-out, EN button, DTR glitch) -- see REHANDSHAKE_AFTER_S.
- **Lockstep, PC-clocked.** The hand never transmits unprompted. Every `T` we
  send is answered by exactly one `S` (or an `E`). The caller must therefore
  call send_targets() on a fixed 30 Hz schedule *whether or not the targets
  changed* -- an unchanged `T` is the heartbeat. Stop sending and the hand's
  250 ms watchdog freezes it, and no `S` comes back at all.
- **Framing.** `<type> <seq> <fields...>*<XX>`: fields separated by exactly one
  space, XOR checksum of everything before `*` as two hex digits. Every `T`
  and `C` carries a `seq` 0..255 from one shared counter; replies echo it.
  Lines that fail the checksum are dropped (and counted).
- **`T <seq> <a0..a18>`**: 19 angles as integers in tenths of a degree. No
  per-joint speed any more (that is `C speed`).
- **`S <seq> <flags> <clamp> <fb> <a0> <m0> ... <a18> <m18>`**: measured angle
  (tenths) and current (mA) per joint, plus WDOG/GAP/NODRV flags, a per-joint
  "target was clamped" mask and a per-joint "feedback is trusted" mask.
- **`C <seq> <name> [args]`** runs a command: answered `K`, then optional log
  text, then `D` -- or a single `E` if rejected. No `T` may be sent from the
  moment a `C` goes out until its `D` (a `T` arriving mid-demo would abort the
  demo, and anything sent during a routine is discarded anyway).
- **`C stats`** returns an `I` line with the hand's link counters. This
  module polls it every STATS_POLL_S by itself, invisibly to the caller.

Protection and E-stop (firmware protection update, PROTOCOL.md §8.1, §10.5)
--------------------------------------------------------------------------
- **`!\\n`** is the digital E-stop. It is a bare line (no seq, no checksum),
  works at any time -- also mid-routine -- and gets no reply of its own; the
  next `S` carries the ESTOP flag (0x08). estop() writes it immediately,
  bypassing the T/C slot logic, so nothing (a routine, a pending command, a
  resync) can delay it.
- While the E-stop is active the hand still answers every `T` with an `S`
  (positions and currents) but ignores the targets, so the 30 Hz loop keeps
  running unchanged. Any command that would move the hand is refused with
  `E <seq> estop`.
- **FAULT** (0x10): at least one joint was stopped by the current/position
  protection -- for example a finger commanded more than 20 deg past where
  an object stopped it, which trips and lets go. On the rising edge of
  FAULT this module asks `C faults` once by itself; request_faults() asks
  again. The reply's `I` line (`I estop=1 P_P=deviation`) is published via
  on_faults and `last_faults`.
- **`C reset`** (reset_protection()) clears the E-stop and all faults. The
  hand holds where it is, so the usual resync follows (slow, probe, publish
  `pending_resync`): the caller re-seeds its targets from the measured pose
  before full speed is restored, exactly as after a routine.

Safety: resync after anything that moved the hand behind our back
------------------------------------------------------------------
v3 has no read-only query -- the only way to learn where the joints are is the
`S` that answers a `T`, and that `T` is itself a motion command. So after
entering PC mode (connect, or re-handshake after a reset) and after every
routine that moves joints, this module runs a short resync: `C speed all 1`
(1 deg/s), one probe `T` (whatever the caller is sending -- harmless at
1 deg/s), then it publishes that `S` as `pending_resync` and keeps the hand
slow until the caller has adopted those positions as its targets and called
ack_resync(). Only then is `C speed all 400` sent (the firmware clamps each
joint to its own ceiling). Without this the first `T` after a routine or reset
would drive the hand at full speed back to a stale pose.

Other design notes:
- Reading is done in bulk (`in_waiting` bytes per read, lines split here).
  pyserial's readline() reads one byte per call; under the GIL contention of
  hand tracking that starved the reader to ~1 line/s.
- All writes go through one lock (GUI thread sends T/C, the reader thread
  sends `pc` during a re-handshake).
- A link log (E/I/K/D/MODE lines, events, and PC counters next to every `I`)
  goes to <app>/logs by default; HRH_LINK_LOG=0 disables it.
"""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence

import serial  # pyserial

from .joints import NUM_JOINTS

BAUD_RATE = 921600  # must match SERIAL_BAUD in the firmware

# Fixed send rate the caller should drive send_targets() at (PROTOCOL.md §4).
STREAM_HZ = 30

# How long a single serial write may block before the frame is abandoned.
# send_targets runs on the GUI thread, so this bounds a UI stall.
WRITE_TIMEOUT_S = 0.05

# Skip a frame if this much is still queued for transmission (~2 T frames).
TX_BACKLOG_LIMIT_BYTES = 256

# Minimum spacing between error reports pushed to the UI.
ERROR_REPORT_INTERVAL_S = 0.25

# No S frame for this long -> the link is "stale" in the UI. At 30 Hz that is
# ~9 missed replies.
LINK_STALE_TIMEOUT_S = 0.3

PC_MODE_ACK = "MODE PC"
PC_MODE_RETRY_INTERVAL_S = 0.5
PC_MODE_WARN_AFTER_S = 10.0

# We have been sending for this long without a single valid reply (outside a
# routine): assume the hand has left PC mode -- almost always a reset -- and
# redo the handshake.
REHANDSHAKE_AFTER_S = 1.0

# Lines the firmware prints only when it (re)boots or leaves PC mode.
_LEFT_PC_MODE_MARKERS = ("MODE TERM", "HRH1 hand - firmware starting")

# A C command that has not been answered with K or E within this long is
# presumed lost, so it cannot block T frames forever.
CMD_START_TIMEOUT_S = 2.0

# `C stats` poll interval (instant command, costs one round trip).
STATS_POLL_S = 2.0

# Resync speeds (deg/s): crawl while probing, then back to full. The firmware
# clamps every joint to its own ceiling (400 fingers/thumb, 240 wrist).
RESYNC_SLOW_DPS = 1
RESYNC_FULL_DPS = 400
RESYNC_PROBE_TIMEOUT_S = 0.5

# If this many lines that look like v2 telemetry (`S <float> ...` without a
# checksum) arrive, the hand is running old firmware -- say so plainly.
_V2_DETECT_LINES = 10

RX_PARTIAL_LINE_MAX = 4096

LINK_LOG_ENV = "HRH_LINK_LOG"
LINK_LOG_DIR = Path(__file__).resolve().parent.parent / "logs"

# S flags (PROTOCOL.md §8.1)
FLAG_WDOG = 0x01
FLAG_GAP = 0x02
FLAG_NODRV = 0x04
FLAG_ESTOP = 0x08    # digital E-stop latched: targets are ignored until `C reset`
FLAG_FAULT = 0x10    # >= 1 joint stopped by current/position protection

# The E-stop line: bare, no seq, no checksum (PROTOCOL.md §10.5).
ESTOP_LINE = b"!\n"

# After `!` mid-routine the hand ends the routine (D or E follows). If neither
# arrives within this long, stop waiting so T frames -- and with them the
# S replies that report the ESTOP flag -- resume.
ESTOP_ROUTINE_RELEASE_S = 1.5

# An internal command refused with `E <seq> estop` is not retried for this
# long, so a refused resync step cannot take every T slot (which would
# starve the hand of T frames and trip its watchdog).
INTERNAL_RETRY_BACKOFF_S = 0.5

# Commands that do not move joints: no resync needed after them.
_INSTANT_COMMANDS = frozenset({"speed", "stats", "estop", "faults"})



def checksum(body: str) -> str:
    x = 0
    for b in body.encode("ascii"):
        x ^= b
    return f"{x:02X}"


def frame(body: str) -> bytes:
    return f"{body}*{checksum(body)}\n".encode("ascii")


def parse_frame(line: str):
    """-> (type, [fields]) for a checksum-valid frame, else None."""
    star = line.rfind("*")
    if star < 1 or len(line) - star != 3:
        return None
    body, cs = line[:star], line[star + 1:]
    if checksum(body) != cs.upper():
        return None
    parts = body.split(" ")
    return parts[0], parts[1:]


@dataclass
class StatusMessage:
    positions_deg: list[float]  # measured angle per joint, degrees
    currents_ma: list[float]    # per joint, mA (joints 0-15 up to ~2.6 s old; 16-18 = wrist rail total)
    received_at: float          # time.monotonic()
    seq: int = -1
    flags: int = 0              # FLAG_WDOG | FLAG_GAP | FLAG_NODRV
    clamp_mask: int = 0         # bit n: joint n's target in that T was clamped
    fb_mask: int = 0            # bit n: joint n's angle is a live, trusted measurement
    rtt_s: Optional[float] = None  # T sent -> this S received

    @property
    def estop(self) -> bool:
        return bool(self.flags & FLAG_ESTOP)

    @property
    def fault(self) -> bool:
        return bool(self.flags & FLAG_FAULT)

    def trusted(self, i: int) -> bool:
        return bool(self.fb_mask >> i & 1)

    def clamped(self, i: int) -> bool:
        return bool(self.clamp_mask >> i & 1)


class SerialLink:
    def __init__(
        self,
        port: str,
        baud: int = BAUD_RATE,
        num_joints: int = NUM_JOINTS,
        on_status: Optional[Callable[[StatusMessage], None]] = None,
        on_error: Optional[Callable[[str], None]] = None,
        on_routine_started: Optional[Callable[[str], None]] = None,
        on_routine_log: Optional[Callable[[str], None]] = None,
        on_routine_done: Optional[Callable[[str], None]] = None,
        on_routine_error: Optional[Callable[[str], None]] = None,
        on_faults: Optional[Callable[[dict], None]] = None,
    ):
        if num_joints != NUM_JOINTS:
            raise ValueError(f"protocol v3 carries exactly {NUM_JOINTS} joints")
        self.port = port
        self.baud = baud
        self.num_joints = num_joints
        # All callbacks are invoked from the background reader thread.
        self._on_status = on_status
        self._on_error = on_error
        self._on_routine_started = on_routine_started
        self._on_routine_log = on_routine_log
        self._on_routine_done = on_routine_done
        self._on_routine_error = on_routine_error
        self._on_faults = on_faults

        self._ser: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._pc_mode_confirmed = threading.Event()
        self._lock = threading.Lock()        # shared state
        self._write_lock = threading.Lock()  # serial writes

        self._seq = 0
        self._last_status: Optional[StatusMessage] = None
        self._last_rx_time = 0.0      # last S
        self._last_alive_time = 0.0   # last valid frame of any kind
        self._last_tx_time = 0.0      # last T/C written

        # Outstanding C command: {"seq", "name", "internal", "sent", "started"}
        self._cmd: Optional[dict] = None
        self._routine_active: Optional[str] = None   # user routine between K and D

        # Resync state machine, see module docstring.
        #   None | "slow" (C speed 1 queued/outstanding) | "probe" (next T is the
        #   probe) | "await_probe" | "await_ack" | "restore" (C speed 400 queued/out)
        self._resync: Optional[str] = None
        self._probe_seq = -1
        self._probe_sent = 0.0
        self._pending_resync: Optional[StatusMessage] = None
        self._last_stats_poll = 0.0

        # Protection (E-stop / faults). Requests are served in send_targets'
        # command slot, like the stats poll, so they never collide with an
        # outstanding command; only the bare `!` bypasses the slot.
        self._req_reset = False
        self._req_faults = False
        self._faults: Optional[dict] = None     # last `C faults` reply, key -> value text
        self._faults_time = 0.0
        self._estop_sent_at = 0.0               # last `!` written (0 = never)
        self._prev_flags = 0                    # flags of the previous S, for edges
        self._internal_backoff_until = 0.0
        self._stat_estops_sent = 0
        self._stat_resets = 0
        self._stat_refused_estop = 0            # E <seq> estop received

        self._t_sent_at = [0.0] * 256   # seq -> monotonic send time, for RTT
        self._t_outstanding = [False] * 256

        # Counters (monotonic totals since construction).
        self._stat_tx_frames = 0      # T frames written
        self._stat_tx_cmds = 0        # C frames written (incl. internal)
        self._stat_tx_bytes = 0
        self._stat_tx_failed = 0
        self._stat_tx_skipped = 0     # T not sent: backpressure
        self._stat_tx_dropped = 0     # T not sent: write timeout
        self._stat_tx_blocked = 0     # T not sent: command/routine in progress
        self._stat_rx_lines = 0
        self._stat_rx_bytes = 0
        self._stat_rx_status = 0      # valid S frames
        self._stat_rx_errors = 0      # E frames
        self._stat_rx_error_kinds: dict = {}
        self._stat_rx_malformed = 0   # checksum-valid S with the wrong shape
        self._stat_rx_badcs = 0       # lines that failed the checksum
        self._stat_lost = 0           # T frames never answered
        self._stat_gap = 0            # S with GAP
        self._stat_wdog = 0           # S with WDOG
        self._stat_rehandshakes = 0
        self._stat_resyncs = 0
        self._stat_rx_noise_bytes = 0
        self._stat_rx_backlog_max = 0
        self._stat_s_intervals: deque = deque(maxlen=120)
        self._stat_rtt: deque = deque(maxlen=120)
        self._v2_lines = 0
        self._v2_warned = False

        self._hand_stats: dict = {}
        self._hand_stats_time = 0.0

        self._last_error_report_t = 0.0
        self._suppressed_errors = 0

        self._handshake_started = 0.0
        self._handshake_last_retry = 0.0
        self._handshake_warned = False

        self._rx_buf = bytearray()
        self._log_file = None
        self._log_lock = threading.Lock()
        self.log_path: Optional[str] = None

    # -- lifecycle ---------------------------------------------------

    def open(self) -> None:
        """Opens the port and starts the background reader/handshake thread.
        Returns immediately; the handshake runs in the background."""
        self._ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=WRITE_TIMEOUT_S)
        self._stop_event.clear()
        self._pc_mode_confirmed.clear()
        self._rx_buf = bytearray()
        self._open_log()
        self._log("open", f"{self.port} @ {self.baud} (protocol v3)")
        self._reader_thread = threading.Thread(target=self._read_loop, name="hrh-link-reader", daemon=True)
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        if self._ser is not None:
            if self._pc_mode_confirmed.is_set():
                try:
                    self._write_raw(b"term\n", "term")
                except serial.SerialException:
                    pass
            try:
                self._ser.close()
            except serial.SerialException:
                pass
            self._ser = None
        self._log("close", "")
        if self._log_file is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
            self._log_file = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def is_pc_mode_confirmed(self) -> bool:
        return self._pc_mode_confirmed.is_set()

    def is_link_stale(self, timeout: float = LINK_STALE_TIMEOUT_S) -> bool:
        """True if no S has arrived recently. False while a command/routine
        is in progress or a resync is running (no S is expected then)."""
        if not self.is_open:
            return True
        with self._lock:
            if self._cmd is not None or self._routine_active is not None:
                return False
        return (time.monotonic() - self._last_rx_time) > timeout

    @property
    def last_status(self) -> Optional[StatusMessage]:
        with self._lock:
            return self._last_status

    @property
    def active_routine(self) -> Optional[str]:
        with self._lock:
            return self._routine_active

    @property
    def rehandshake_count(self) -> int:
        with self._lock:
            return self._stat_rehandshakes

    @property
    def pending_resync(self) -> Optional[StatusMessage]:
        """Set when the hand's actual pose is known after a connect, reset or
        routine and the hand is being held at 1 deg/s. The caller must adopt
        these positions as its targets (at least for joints whose feedback is
        trusted) and then call ack_resync()."""
        with self._lock:
            return self._pending_resync

    def ack_resync(self) -> None:
        with self._lock:
            if self._resync == "await_ack":
                self._pending_resync = None
                self._resync = "restore"
        self._log("resync", "caller adopted measured pose; restoring full speed")

    # -- protection: E-stop, reset, faults -----------------------------

    def estop(self) -> bool:
        """Digital E-stop: writes the bare `!` line *now*, from any thread.

        Bypasses the T/C slot logic on purpose -- it is the one frame that
        must never wait for a routine, a pending command or a resync. The
        hand does not reply; the next S carries FLAG_ESTOP. Keep the 30 Hz
        loop running: the hand still answers every T while stopped (it just
        ignores the targets). Returns True if the line was written."""
        if self._ser is None:
            return False
        ok = self._write_raw(ESTOP_LINE, "! (E-stop)")
        if ok:
            with self._lock:
                self._estop_sent_at = time.monotonic()
                self._stat_estops_sent += 1
        return ok

    def reset_protection(self) -> None:
        """`C reset`: clears the E-stop and every fault. Sent in the next free
        command slot. The hand then holds where it is; this module runs the
        usual resync afterwards, so the caller must adopt `pending_resync`
        (as it already does after a routine) before full speed returns."""
        with self._lock:
            self._req_reset = True
        self._log("reset", "requested")

    def request_faults(self) -> None:
        """`C faults`: the reply is published via on_faults / last_faults."""
        with self._lock:
            self._req_faults = True

    @property
    def estop_active(self) -> bool:
        """The hand reports its E-stop latched (from the latest S)."""
        with self._lock:
            return bool(self._last_status is not None and self._last_status.flags & FLAG_ESTOP)

    @property
    def fault_active(self) -> bool:
        with self._lock:
            return bool(self._last_status is not None and self._last_status.flags & FLAG_FAULT)

    @property
    def estop_pending(self) -> bool:
        """`!` was written within the last second but no S has confirmed the
        flag yet -- lets the UI show "stopping" instead of a stale "OK"."""
        with self._lock:
            sent = self._estop_sent_at
            confirmed = self._last_status is not None and self._last_status.flags & FLAG_ESTOP \
                and self._last_status.received_at >= sent
        return bool(sent and not confirmed and time.monotonic() - sent < 1.0)

    @property
    def last_faults(self) -> Optional[dict]:
        """Last `C faults` reply as {key: value text}, e.g. {"estop": "1",
        "P_P": "deviation"}; None if never asked or cleared by a reset."""
        with self._lock:
            return dict(self._faults) if self._faults is not None else None

    # -- outbound ------------------------------------------------------

    def send_targets(self, angles_deg: Sequence[float], speeds_deg_s: Optional[Sequence[float]] = None) -> bool:
        """One tick of the 30 Hz loop: sends a `T` with these targets.

        Call this on a fixed schedule whether or not the targets changed --
        it is the heartbeat. On some ticks the module uses the slot for its
        own command instead (resync steps, the stats poll) and returns False;
        it also returns False while a command/routine is in progress or the
        handshake is running. `speeds_deg_s` is accepted for compatibility
        and ignored: v3 has no per-frame speed (use set_speed()).
        Returns True only if a `T` was written."""
        if len(angles_deg) != self.num_joints:
            raise ValueError(f"expected {self.num_joints} angles, got {len(angles_deg)}")
        if not self._pc_mode_confirmed.is_set():
            return False
        now = time.monotonic()

        with self._lock:
            cmd = self._cmd
            if cmd is not None and not cmd["started"] and now - cmd["sent"] > CMD_START_TIMEOUT_S:
                # Never answered: presumed lost. Don't let it block T forever.
                self._cmd = None
                cmd_lost = cmd
                cmd = None
            else:
                cmd_lost = None
        if cmd_lost is not None:
            # An internal resync step is simply re-issued below (its stage
            # has not advanced); a user command is reported.
            self._log("cmd-lost", f"C {cmd_lost['seq']} {cmd_lost['name']}: no K/E within {CMD_START_TIMEOUT_S} s")
            if cmd_lost["user"]:
                self._report_routine_error(f"no response to {cmd_lost['name']!r} from the hand")
        if cmd is not None and cmd.get("user") and cmd["started"]:
            # `!` mid-routine ends the routine on the hand; normally its D or
            # E arrives at once. If it doesn't, stop waiting: T frames must
            # resume so the ESTOP flag can come back in an S.
            with self._lock:
                stopped = self._estop_sent_at > cmd["sent"] and now - self._estop_sent_at > ESTOP_ROUTINE_RELEASE_S
                if stopped:
                    self._cmd = None
                    self._routine_active = None
                    self._resync = "slow"
                    self._stat_resyncs += 1
            if stopped:
                self._log("estop", f"no D for {cmd['name']} after E-stop; resuming T frames")
                self._report_routine_error(f"{cmd['name']} stopped by E-stop")
                cmd = None
        if cmd is not None:
            with self._lock:
                self._stat_tx_blocked += 1
            return False

        # This tick's slot may be taken by a protection request (reset first:
        # it is what the operator is waiting for) or by one of our own
        # commands.
        with self._lock:
            req_reset, req_faults = self._req_reset, self._req_faults
            self._req_reset = False if req_reset else self._req_reset
            self._req_faults = False if (req_faults and not req_reset) else self._req_faults
        if req_reset:
            self._send_command("reset", [], internal=False, quiet=True)
            return False
        if req_faults:
            self._send_command("faults", [], internal=False, quiet=True)
            return False

        with self._lock:
            resync = self._resync
            if resync == "await_probe" and now - self._probe_sent > RESYNC_PROBE_TIMEOUT_S:
                self._resync = resync = "probe"   # probe reply lost: probe again
            backoff = now < self._internal_backoff_until
        if resync == "slow" and not backoff:
            self._send_command("speed", ["all", str(RESYNC_SLOW_DPS)], internal=True)
            return False
        if resync == "restore" and not backoff:
            self._send_command("speed", ["all", str(RESYNC_FULL_DPS)], internal=True)
            return False
        if resync is None and not backoff and now - self._last_stats_poll >= STATS_POLL_S:
            self._last_stats_poll = now
            self._send_command("stats", [], internal=True)
            return False

        if self._tx_backlog() > TX_BACKLOG_LIMIT_BYTES:
            with self._lock:
                self._stat_tx_skipped += 1
            return False

        # Range limits are left to the firmware on purpose: it clamps and
        # REPORTS which joints it clamped (S clamp mask), which is only
        # meaningful if we don't clamp first. Only the wire's int16 range is
        # enforced here, since outside it the whole frame would be rejected.
        vals = []
        for deg in angles_deg:
            t = int(round(float(deg) * 10.0))
            vals.append(-32768 if t < -32768 else 32767 if t > 32767 else t)
        seq = self._next_seq()
        # Book-keep BEFORE writing: the reply can be parsed by the reader
        # thread before write() even returns.
        with self._lock:
            old = (seq - STREAM_HZ) & 0xFF      # the T sent ~1 s ago
            if self._t_outstanding[old] and now - self._t_sent_at[old] > 0.5:
                self._stat_lost += 1            # ...never answered
                self._t_outstanding[old] = False
            self._t_outstanding[seq] = True
            self._t_sent_at[seq] = now
            if self._resync == "probe":
                self._probe_seq = seq
                self._probe_sent = now
                self._resync = "await_probe"
        if not self._write_raw(frame(f"T {seq} " + " ".join(map(str, vals))), None):
            with self._lock:
                self._t_outstanding[seq] = False
                if self._resync == "await_probe" and self._probe_seq == seq:
                    self._resync = "probe"
            return False
        with self._lock:
            self._stat_tx_frames += 1
            self._last_tx_time = now
        return True

    def set_speed(self, joint: str, dps: int) -> None:
        """`C speed <joint|all> <dps>` -- an instant command; T frames pause
        for one round trip. Note a resync resets speeds to full."""
        self._send_command("speed", [joint, str(int(dps))], internal=False, quiet=True)

    def run_routine(self, name: str, arg: Optional[str] = None) -> None:
        """Starts a built-in routine: `C <seq> <name> [arg]`. Progress comes
        back via the on_routine_* callbacks (K -> started, log lines, D ->
        done, E -> error). T frames are held from now until D."""
        if not self._pc_mode_confirmed.is_set():
            self._report_routine_error(f"cannot start {name!r}: PC mode not confirmed yet")
            return
        with self._lock:
            busy = self._cmd
        if busy is not None and not busy["internal"]:
            self._report_routine_error(f"cannot start {name!r}: {busy['name']!r} is already running")
            return
        if busy is not None:
            # An internal command (stats/speed) is a few ms from its D; wait it out.
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                with self._lock:
                    if self._cmd is None:
                        break
                time.sleep(0.005)
        self._send_command(name, [arg] if arg else [], internal=False)

    def send_routine_input(self, text: str) -> None:
        """Relays one plain line (no seq, no checksum) to an interactive
        routine -- `manualhome` jog values or `d`, or `abort` for demos.
        A no-op when no routine is running."""
        if self.active_routine is None:
            return
        self._write_raw((text.strip() + "\n").encode("ascii", "replace"), text.strip())

    def _next_seq(self) -> int:
        with self._lock:
            self._seq = (self._seq + 1) & 0xFF
            return self._seq

    def _send_command(self, name: str, args: list, internal: bool, quiet: bool = False) -> None:
        seq = self._next_seq()
        body = " ".join(["C", str(seq), name] + [a for a in args if a])
        with self._lock:
            self._cmd = {"seq": seq, "name": name, "internal": internal or quiet,
                         "user": not (internal or quiet), "sent": time.monotonic(), "started": False}
        if self._write_raw(frame(body), body if name != "stats" else None):
            with self._lock:
                self._stat_tx_cmds += 1
                self._last_tx_time = time.monotonic()
        else:
            with self._lock:
                self._cmd = None

    def _write_raw(self, payload: bytes, log_text: Optional[str]) -> bool:
        ser = self._ser
        if ser is None:
            return False
        if log_text is not None:
            self._log("tx", log_text)
        try:
            with self._write_lock:
                ser.write(payload)
        except serial.SerialTimeoutException:
            with self._lock:
                self._stat_tx_dropped += 1
            return False
        except serial.SerialException as exc:
            with self._lock:
                self._stat_tx_failed += 1
            self._report_error(f"write failed: {exc}")
            return False
        with self._lock:
            self._stat_tx_bytes += len(payload)
        return True

    def _tx_backlog(self) -> int:
        try:
            return int(self._ser.out_waiting) if self._ser is not None else 0
        except (OSError, ValueError, AttributeError, serial.SerialException):
            return 0

    # -- inbound -------------------------------------------------------

    def _read_loop(self) -> None:
        self._begin_handshake(reason=None)
        while not self._stop_event.is_set():
            if self._ser is None:
                break
            try:
                lines = self._read_lines()
            except serial.SerialException as exc:
                self._report_error(f"read failed: {exc}")
                break
            for raw in lines:
                try:
                    self._handle_line(raw)
                except Exception as exc:  # noqa: BLE001 - one bad line must not kill the reader
                    self._log("reader-error", f"{exc!r} on {raw[:80]!r}")
            try:
                self._housekeeping(time.monotonic())
            except serial.SerialException as exc:
                self._report_error(f"write failed during handshake: {exc}")
                break

    def _read_lines(self) -> list:
        waiting = self._ser.in_waiting
        if waiting > self._stat_rx_backlog_max:
            with self._lock:
                self._stat_rx_backlog_max = waiting
        data = self._ser.read(waiting if waiting > 0 else 1)
        if not data:
            return []
        buf = self._rx_buf
        buf += data
        if b"\n" not in data:
            if len(buf) > RX_PARTIAL_LINE_MAX:
                with self._lock:
                    self._stat_rx_noise_bytes += len(buf)
                buf.clear()
            return []
        lines = buf.split(b"\n")
        self._rx_buf = bytearray(lines.pop())
        return lines

    def _begin_handshake(self, reason: Optional[str]) -> None:
        self._pc_mode_confirmed.clear()
        with self._lock:
            interrupted = self._routine_active if self._cmd and self._cmd.get("user") else None
            self._routine_active = None
            self._cmd = None
            self._resync = None
            self._pending_resync = None
            self._t_outstanding = [False] * 256
        now = time.monotonic()
        self._handshake_started = now
        self._handshake_last_retry = now
        self._handshake_warned = False
        if reason is not None:
            with self._lock:
                self._stat_rehandshakes += 1
            self._log("rehandshake", reason)
            self._report_error(f"{reason} -- re-requesting PC mode")
            if interrupted is not None:
                self._report_routine_error(f"{interrupted} interrupted: {reason}")
        self._write_raw(b"pc\n", "pc")

    def _confirm_pc_mode(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._last_rx_time = now
            self._last_alive_time = now
            self._last_tx_time = now
            self._last_stats_poll = now - STATS_POLL_S + 0.5   # first stats soon, not instantly
            # Entering PC mode resets the hand's speeds and seq tracking; we
            # still don't know its pose, so resync before trusting our targets.
            self._resync = "slow"
            self._stat_resyncs += 1
        self._pc_mode_confirmed.set()
        self._log("pc-mode", f"confirmed after {now - self._handshake_started:.2f} s; resyncing")

    def _housekeeping(self, now: float) -> None:
        if not self._pc_mode_confirmed.is_set():
            if now - self._handshake_last_retry >= PC_MODE_RETRY_INTERVAL_S:
                self._write_raw(b"pc\n", "pc")
                self._handshake_last_retry = now
            if not self._handshake_warned and now - self._handshake_started >= PC_MODE_WARN_AFTER_S:
                self._report_error("hand hasn't answered MODE PC yet -- still retrying; "
                                   "check the port and that the firmware is running")
                self._handshake_warned = True
            return
        with self._lock:
            in_routine = self._routine_active is not None
            sent_since = self._last_tx_time - self._last_alive_time
        # We have been talking for a while and nothing valid came back.
        if not in_routine and sent_since > REHANDSHAKE_AFTER_S:
            self._begin_handshake(reason=f"no reply from the hand for {sent_since:.1f} s (reset or left PC mode?)")

    def _handle_line(self, raw: bytes) -> None:
        with self._lock:
            self._stat_rx_lines += 1
            self._stat_rx_bytes += len(raw) + 1
        text = raw.decode("ascii", errors="replace").strip()
        if not text:
            return
        now = time.monotonic()

        if text == PC_MODE_ACK:
            self._log("rx", text)
            if not self._pc_mode_confirmed.is_set():
                self._confirm_pc_mode()
            else:
                # Unexpected: something re-entered PC mode (session reset on
                # the hand). Resync, since its speeds and seq state reset.
                self._last_alive_time = now
                with self._lock:
                    if self._resync is None:
                        self._resync = "slow"
                        self._stat_resyncs += 1
            return

        if text.startswith(_LEFT_PC_MODE_MARKERS):
            self._log("rx", text)
            if self._pc_mode_confirmed.is_set():
                self._begin_handshake(reason=f"hand left PC mode ({text!r})")
            return

        if not self._pc_mode_confirmed.is_set():
            self._log("term", text)   # boot banner / menu while handshaking
            return

        parsed = parse_frame(text)
        with self._lock:
            cmd = self._cmd
            in_routine = self._routine_active is not None

        if parsed is None:
            if in_routine:
                # Free-form routine log text (not checksummed).
                self._last_alive_time = now
                if self._on_routine_log is not None:
                    self._on_routine_log(text)
                return
            with self._lock:
                self._stat_rx_badcs += 1
            self._log("rx-badcs", text[:200])
            if text.startswith("S ") and "*" not in text:
                self._v2_lines += 1
                if self._v2_lines >= _V2_DETECT_LINES and not self._v2_warned:
                    self._v2_warned = True
                    self._report_error("the hand is sending protocol v2 telemetry -- it is running "
                                       "old firmware; this app speaks protocol v3")
            return

        kind, f = parsed
        self._last_alive_time = now

        if kind == "S":
            self._handle_state(f, now)
            return

        if kind in ("K", "D", "E") and cmd is not None and f and f[0] == str(cmd["seq"]):
            self._handle_cmd_reply(kind, f, cmd, now)
            return

        if kind == "I":
            if cmd is not None and cmd["name"] == "faults":
                self._handle_faults(f, now, text)
            else:
                self._handle_info(f, now, text)
            return

        if kind == "E":
            code = f[1] if len(f) > 1 else "other"
            self._log("rx", text)
            with self._lock:
                self._stat_rx_errors += 1
                self._stat_rx_error_kinds[code] = self._stat_rx_error_kinds.get(code, 0) + 1
                if code == "estop":
                    self._stat_refused_estop += 1
                if len(f) and f[0].isdigit():
                    self._t_outstanding[int(f[0]) & 0xFF] = False
            if code == "estop":
                self._report_error_throttled("refused: the E-stop is active -- press Reset to clear it")
            else:
                self._report_error_throttled(f"hand rejected a frame: {code}" +
                                             ("" if f[0] == "-" else f" (seq {f[0]})"))
            return

        if in_routine:
            if self._on_routine_log is not None:
                self._on_routine_log(text)
            return
        self._log("rx-unexpected", text)

    def _handle_cmd_reply(self, kind: str, f: list, cmd: dict, now: float) -> None:
        name = cmd["name"]
        internal = cmd["internal"]
        if not (internal and name == "stats"):
            self._log("rx", f"{kind} {' '.join(f)}")
        if kind == "K":
            with self._lock:
                cmd["started"] = True
                if cmd.get("user"):
                    self._routine_active = name
            if cmd.get("user") and self._on_routine_started is not None:
                self._on_routine_started(name)
            return
        if kind == "E":
            code = f[1] if len(f) > 1 else "error"
            with self._lock:
                self._cmd = None
                self._routine_active = None
                self._stat_rx_errors += 1
                self._stat_rx_error_kinds[code] = self._stat_rx_error_kinds.get(code, 0) + 1
                if code == "estop":
                    self._stat_refused_estop += 1
                if not cmd.get("user"):
                    # Don't re-issue a refused internal step on the very next
                    # tick: every slot it takes is a T the hand doesn't get.
                    self._internal_backoff_until = now + INTERNAL_RETRY_BACKOFF_S
            if cmd.get("user"):
                if code == "estop":
                    self._report_routine_error(f"{name} refused: the E-stop is active -- press Reset first")
                else:
                    self._report_routine_error(f"{name}: {code}")
            elif name in ("reset", "faults"):
                self._log("cmd-error", f"C {name}: {code}")
                self._report_error(f"`{name}` was refused by the hand: {code}")
            else:
                # Internal step rejected; its resync stage has not advanced,
                # so it is re-issued on a later tick.
                self._log("cmd-error", f"internal C {name}: {code}")
            return
        # D
        with self._lock:
            self._cmd = None
            self._routine_active = None
            if name == "reset":
                # Faults and E-stop cleared; the hand holds where it is. Our
                # caller's targets may be anywhere (tracking, sliders moved
                # while stopped): resync before anything moves at full speed.
                self._faults = None
                self._stat_resets += 1
                self._resync = "slow"
                self._stat_resyncs += 1
            elif internal and name == "speed" and self._resync == "slow":
                self._resync = "probe"
            elif internal and name == "speed" and self._resync == "restore":
                self._resync = None
            elif cmd.get("user") and name not in _INSTANT_COMMANDS:
                # The routine moved joints: the hand re-seeded itself, but our
                # caller's targets are stale. Resync before full speed.
                self._resync = "slow"
                self._stat_resyncs += 1
            self._last_rx_time = now   # no false "stale" blip before S resumes
        if self._resync == "slow":
            self._log("resync", f"after {name}")
        if name == "reset" and self._on_faults is not None:
            self._on_faults({})
        if cmd.get("user") and self._on_routine_done is not None:
            self._on_routine_done(name)

    def _handle_state(self, f: list, now: float) -> None:
        if len(f) != 4 + 2 * self.num_joints:
            with self._lock:
                self._stat_rx_malformed += 1
            self._report_error_throttled(f"malformed S frame ({len(f)} fields, expected {4 + 2 * self.num_joints})")
            return
        try:
            seq = int(f[0])
            flags = int(f[1], 16)
            clamp = int(f[2], 16)
            fb = int(f[3], 16)
            v = [int(x) for x in f[4:]]
        except ValueError:
            with self._lock:
                self._stat_rx_malformed += 1
            self._report_error_throttled("malformed S frame (bad number)")
            return
        s = seq & 0xFF
        with self._lock:
            rtt = (now - self._t_sent_at[s]) if self._t_outstanding[s] else None
            self._t_outstanding[s] = False
        status = StatusMessage(
            positions_deg=[a / 10.0 for a in v[0::2]],
            currents_ma=[float(m) for m in v[1::2]],
            received_at=now, seq=seq, flags=flags, clamp_mask=clamp, fb_mask=fb, rtt_s=rtt,
        )
        notes = []
        with self._lock:
            if self._last_rx_time:
                self._stat_s_intervals.append(now - self._last_rx_time)
            if rtt is not None:
                self._stat_rtt.append(rtt)
            self._last_status = status
            self._last_rx_time = now
            self._stat_rx_status += 1
            if flags & FLAG_GAP:
                self._stat_gap += 1
            if flags & FLAG_WDOG:
                self._stat_wdog += 1
                notes.append("WDOG")
            if self._resync == "await_probe" and seq == self._probe_seq:
                self._pending_resync = status
                self._resync = "await_ack"
                notes.append("probe")
            rising = flags & ~self._prev_flags
            falling = self._prev_flags & ~flags
            self._prev_flags = flags
            if rising & FLAG_FAULT:
                self._req_faults = True    # find out which joint(s), once per trip
            if falling & FLAG_FAULT and not flags & FLAG_ESTOP:
                self._faults = None
        if rising & FLAG_ESTOP:
            self._log("rx-flag", f"S {seq} ESTOP: E-stop active, targets ignored")
        if falling & FLAG_ESTOP:
            self._log("rx-flag", f"S {seq} ESTOP cleared")
        if rising & FLAG_FAULT:
            self._log("rx-flag", f"S {seq} FAULT: protection stopped a joint; asking C faults")
        if "WDOG" in notes:
            self._log("rx-flag", f"S {seq} WDOG: hand saw no T for 250 ms before this one")
            self._report_error_throttled("hand watchdog fired: the send loop stalled for > 250 ms")
        if "probe" in notes:
            self._log("resync", f"probe S {seq}: pose known, waiting for caller to adopt it")
        if self._on_status is not None:
            self._on_status(status)

    def _handle_info(self, f: list, now: float, text: str) -> None:
        fields = {}
        for tok in f:
            k, sep, val = tok.partition("=")
            if sep:
                try:
                    fields[k] = int(val)
                except ValueError:
                    pass
        with self._lock:
            self._hand_stats = fields
            self._hand_stats_time = now
        if self._log_file is not None:
            self._log("rx", text)
            s = self.stats()
            self._log("pc", "T={tx_frames} S={rx_status} lost={lost} gap={gap} wdog={wdog} err={rx_errors} "
                            "badcs={rx_badcs} blocked={tx_blocked} skip={tx_skipped} drop={tx_dropped} "
                            "rehs={rehandshakes} resync={resyncs} backlogmax={rx_backlog_max}".format(**s))

    def _handle_faults(self, f: list, now: float, text: str) -> None:
        """`I estop=1 P_P=deviation` -> {"estop": "1", "P_P": "deviation"}.
        Values are kept as text: the joint entries are reasons, not numbers."""
        faults = {}
        for tok in f:
            k, sep, val = tok.partition("=")
            if sep and k:
                faults[k] = val
        with self._lock:
            self._faults = faults
            self._faults_time = now
        self._log("rx", text)
        if self._on_faults is not None:
            self._on_faults(dict(faults))

    # -- stats ---------------------------------------------------------

    def stats(self) -> dict:
        with self._lock:
            intervals = list(self._stat_s_intervals)
            rtts = list(self._stat_rtt)
            snap = {
                "tx_frames": self._stat_tx_frames,
                "tx_cmds": self._stat_tx_cmds,
                "tx_bytes": self._stat_tx_bytes,
                "tx_failed": self._stat_tx_failed,
                "tx_skipped": self._stat_tx_skipped,
                "tx_dropped": self._stat_tx_dropped,
                "tx_blocked": self._stat_tx_blocked,
                "rx_lines": self._stat_rx_lines,
                "rx_bytes": self._stat_rx_bytes,
                "rx_status": self._stat_rx_status,
                "rx_errors": self._stat_rx_errors,
                "rx_malformed": self._stat_rx_malformed,
                "rx_badcs": self._stat_rx_badcs,
                "rx_error_kinds": dict(self._stat_rx_error_kinds),
                "lost": self._stat_lost,
                "gap": self._stat_gap,
                "wdog": self._stat_wdog,
                "last_rx_time": self._last_rx_time,
                "rehandshakes": self._stat_rehandshakes,
                "resyncs": self._stat_resyncs,
                "resync_state": self._resync,
                "rx_noise_bytes": self._stat_rx_noise_bytes,
                "rx_backlog_max": self._stat_rx_backlog_max,
                "pc_mode": self._pc_mode_confirmed.is_set(),
                "hand": dict(self._hand_stats),
                "hand_age": (time.monotonic() - self._hand_stats_time) if self._hand_stats_time else None,
                "estops_sent": self._stat_estops_sent,
                "resets": self._stat_resets,
                "refused_estop": self._stat_refused_estop,
                "faults": dict(self._faults) if self._faults is not None else None,
                "last_flags": self._last_status.flags if self._last_status else 0,
                "last_clamp": self._last_status.clamp_mask if self._last_status else 0,
                "last_fb": self._last_status.fb_mask if self._last_status else 0,
            }
        try:
            snap["rx_backlog"] = int(self._ser.in_waiting) if self._ser is not None else 0
        except (OSError, ValueError, AttributeError, serial.SerialException):
            snap["rx_backlog"] = 0
        if intervals:
            mean = sum(intervals) / len(intervals)
            snap["s_interval_mean"] = mean
            snap["s_interval_jitter"] = (sum((v - mean) ** 2 for v in intervals) / len(intervals)) ** 0.5
        if rtts:
            snap["rtt_mean"] = sum(rtts) / len(rtts)
            snap["rtt_max"] = max(rtts)
        return snap

    # -- reporting -----------------------------------------------------

    def _report_error_throttled(self, message: str) -> None:
        """At most one UI report per ERROR_REPORT_INTERVAL_S; errors are still
        counted in full, and the suppressed count is folded into the next."""
        with self._lock:
            now = time.monotonic()
            if now - self._last_error_report_t < ERROR_REPORT_INTERVAL_S:
                self._suppressed_errors += 1
                return
            suppressed = self._suppressed_errors
            self._suppressed_errors = 0
            self._last_error_report_t = now
        if suppressed:
            message = f"{message}  (+{suppressed} more suppressed)"
        self._report_error(message)

    def _report_error(self, message: str) -> None:
        if self._on_error is not None:
            self._on_error(message)

    def _report_routine_error(self, message: str) -> None:
        if self._on_routine_error is not None:
            self._on_routine_error(message)
        else:
            self._report_error(message)

    # -- link log ------------------------------------------------------

    def _open_log(self) -> None:
        setting = os.environ.get(LINK_LOG_ENV, "").strip()
        if setting == "0":
            return
        try:
            if setting:
                path = Path(setting)
            else:
                LINK_LOG_DIR.mkdir(parents=True, exist_ok=True)
                path = LINK_LOG_DIR / time.strftime("link-%Y%m%d-%H%M%S.log")
            self._log_file = open(path, "a", encoding="ascii", errors="replace", buffering=1)
            self.log_path = str(path)
        except OSError:
            self._log_file = None

    def _log(self, tag: str, text: str) -> None:
        f = self._log_file
        if f is None:
            return
        t = time.time()
        stamp = time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 1000) % 1000:03d}"
        try:
            with self._log_lock:
                f.write(f"{stamp} {tag:<12} {text}\n")
        except (OSError, ValueError):
            pass


def list_available_ports() -> list[str]:
    """Return device names of currently connected serial ports."""
    from serial.tools import list_ports

    return [p.device for p in list_ports.comports()]
