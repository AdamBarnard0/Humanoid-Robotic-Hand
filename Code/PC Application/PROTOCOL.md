# HRH1 Serial Protocol v3

Exact wire specification between the PC application and the HRH1 hand
firmware (ESP32). Everything here matches the firmware as built:
`src/Protocol.cpp`, `src/main.cpp` and `include/Config.h` section 11.

> **v3 is not compatible with v2.** The exchange model, the frame layout and
> the number format have all changed. §14 lists every difference.

---

## 1. Summary

* The **PC is the only clock.** It sends one `T` frame every **33.3 ms (30 Hz)**.
* The hand answers **every line it receives with exactly one reply line**
  (`S` on success, `E` on error). It never sends anything unprompted.
* Angles travel as **integers in tenths of a degree**. No floats on the wire.
* Every frame ends in a **mandatory XOR checksum** (`*XX`).
* Every PC frame carries a **sequence number** that the hand echoes back.
* Speed is **not** in the fast frame. It is set occasionally with `C speed`.

---

## 2. Transport

| | |
|---|---|
| Link | USB serial (ESP32 DevKit's USB–UART bridge) |
| Baud | **921600** |
| Framing | 8N1, no flow control |
| Encoding | 7-bit ASCII, one frame per line |
| PC → hand line ending | `\n` (a trailing `\r` is tolerated and ignored) |
| Hand → PC line ending | `\r\n` — strip it before parsing |
| Longest line accepted | **255 characters**, excluding the line ending |

Write every frame with a **single** `write()` call.

---

## 3. Modes and handshake

The firmware boots into **terminal mode**: a human menu on the same port. It
does not speak the protocol until switched over.

```
PC → hand    pc\n              enter PC mode
hand → PC    MODE PC\r\n       acknowledged
```

```
PC → hand    term\n            leave PC mode
hand → PC    MODE TERM\r\n     followed by the human menu
```

`pc` and `term` are plain lowercase words, with **no** sequence number and
**no** checksum, so they can also be typed by hand.

`pc` is **idempotent**. Sent while already in PC mode it is answered
`MODE PC` again and resets the session (see §3.2). It is the recovery action
for any confused state.

### 3.1 Startup sequence

1. Open the port. Most ESP32 boards reset when the port opens; expect a boot
   banner, an I2C scan and a "Holding current position" summary.
2. Discard everything until the handshake completes.
3. Send `pc\n` every 500 ms until a line reading exactly `MODE PC` arrives.
4. Start the 30 Hz loop.

**The hand does not move on boot.** It reads each joint's feedback pot and
holds the position it is already in. The first `S` reply therefore describes
the pose the hand was physically left in. Seed your targets from it instead
of assuming a start pose.

### 3.2 What entering PC mode resets

* Sequence tracking. The next `T` is never flagged as a gap.
* Every joint's speed, back to its ceiling (§6).
* The statistics counters (§10.3).
* The watchdog, which stays **disarmed until the first valid `T`**.
* Motion targets, re-seeded from measured positions so nothing jumps.

---

## 4. Exchange model

```
 PC                                   hand
  |  T 41 ... *xx   ─────────────────▶ |   parse, apply targets,
  |                                    |   read feedback + current
  |  ◀───────────────── S 41 ... *xx  |   reply (typically < 3 ms)
  |      33.3 ms later
  |  T 42 ... *xx   ─────────────────▶ |
  |  ◀───────────────── S 42 ... *xx  |
```

**Rules:**

1. **Send `T` on a fixed 33.3 ms schedule, whether or not the targets have
   changed.** An unchanged frame is the heartbeat. Stop sending and the
   watchdog freezes the hand (§9).
2. **Never block waiting for a reply.** Send on your clock, then collect
   whatever has arrived. A reply that hasn't arrived by the time you send
   the next `T` counts as lost. Blocking on a lost reply stalls the loop.
3. **One line in, one line out.** Every line the hand receives in PC mode
   produces exactly one reply line: `S`, `E`, `K` or `MODE PC`. The only
   exception is the log text a routine prints between `K` and `D` (§10).
   Frames sent should equal replies received. If they don't, something was
   lost. (An empty line is ignored and gets no reply.)
4. **Match replies by sequence number,** never by arrival order.
5. **Use a deadline-based loop** (`next += period; sleep(next - now)`), not
   `sleep(period)`. Windows timers have ~15.6 ms granularity by default, so
   a naive sleep drifts badly. If you fall behind, reset the deadline rather
   than sending a burst to catch up.

The hand's own motion runs on an **independent 50 Hz slew limiter**. It
interpolates between your 30 Hz targets, so motion stays smooth even though
targets arrive less often.

---

## 5. Framing

### 5.1 Layout

```
<type> <field> <field> ... <field>*<cs>
```

* `<type>` is one uppercase letter.
* Fields are separated by **exactly one space.** No leading space, no
  trailing space, no double spaces.
* The last field is followed **directly** by `*` and exactly two hex digits.
* Integers are plain decimal: an optional `-`, then 1–5 digits. No `+`, no
  decimal point, no exponent, no leading/trailing whitespace.
* Names (routines, joints) are case-sensitive and must match this document.

Any violation gets an `E` reply. Nothing is guessed or repaired.

### 5.2 Checksum

The checksum is the **XOR of every byte from the first character of the line
up to, but not including, the `*`**. It is written as two hex digits. The hand
always emits uppercase and accepts either case.

Worked example on the body `T 3 0`:

```
'T' 0x54 ^ ' ' 0x20 = 0x74
     ^ '3' 0x33 = 0x47
     ^ ' ' 0x20 = 0x67
     ^ '0' 0x30 = 0x57      →  "T 3 0*57"
```

```python
def checksum(body: str) -> str:
    x = 0
    for b in body.encode("ascii"):
        x ^= b
    return f"{x:02X}"
```

### 5.3 Sequence number

Every `T` and `C` carries a `seq` from **0 to 255**, wrapping 255 → 0. Use
one counter for both frame types and increment it for every frame sent. The
hand echoes it in the reply, and flags a gap if a frame's `seq` is not the
previous `seq + 1` (§7.2).

### 5.4 Angles

All angles on the wire are **signed integers in tenths of a degree**:
`-350` is −35.0°, `1160` is +116.0°. Divide by 10 on receipt and multiply by
10 (then round) before sending.

0.1° is finer than the servos can resolve (roughly 0.5–0.8° per PWM step),
so this format loses nothing.

---

## 6. Joints

Every `T` and `S` frame lists all 19 joints in this fixed order. It is the
firmware's wiring-table order, **not** motor or PWM channel order.

| # | Name | Zero means | Min | Max | Min (wire) | Max (wire) | Speed ceiling |
|--:|---|---|--:|--:|--:|--:|--:|
| 0 | `P_MP` | finger straight | −35.0° | +3.0° | −350 | 30 | 400 °/s |
| 1 | `P_P` | fully extended | 0.0° | +90.0° | 0 | 900 | 400 °/s |
| 2 | `P_IP` | fully extended | 0.0° | +116.0° | 0 | 1160 | 400 °/s |
| 3 | `R_MP` | finger straight | −24.0° | +20.0° | −240 | 200 | 400 °/s |
| 4 | `R_P` | fully extended | 0.0° | +90.0° | 0 | 900 | 400 °/s |
| 5 | `R_IP` | fully extended | 0.0° | +116.0° | 0 | 1160 | 400 °/s |
| 6 | `M_MP` | finger straight | −24.0° | +24.0° | −240 | 240 | 400 °/s |
| 7 | `M_P` | fully extended | 0.0° | +90.0° | 0 | 900 | 400 °/s |
| 8 | `M_IP` | fully extended | 0.0° | +116.0° | 0 | 1160 | 400 °/s |
| 9 | `I_MP` | finger straight | −20.0° | +24.0° | −200 | 240 | 400 °/s |
| 10 | `I_P` | fully extended | 0.0° | +90.0° | 0 | 900 | 400 °/s |
| 11 | `I_IP` | fully extended | 0.0° | +116.0° | 0 | 1160 | 400 °/s |
| 12 | `T_CMC` | neutral | −37.0° | +82.0° | −370 | 820 | 400 °/s |
| 13 | `T_ABD` | neutral | −32.0° | +70.0° | −320 | 700 | 400 °/s |
| 14 | `T_MCP` | neutral | −65.0° | +95.0° | −650 | 950 | 400 °/s |
| 15 | `T_IP` | tip straight | −63.0° | +98.0° | −630 | 980 | 400 °/s |
| 16 | `W_1` | neutral (roll) | −90.0° | +90.0° | −900 | 900 | 240 °/s |
| 17 | `W_2` | neutral (pitch) | −55.0° | +55.0° | −550 | 550 | 240 °/s |
| 18 | `W_3` | neutral (yaw) | −16.5° | +16.5° | −165 | 165 | 240 °/s |

Prefixes: `P` pinky, `R` ring, `M` middle, `I` index, `T` thumb, `W` wrist.
`_MP` is abduction (splay), `_P` knuckle flexion, `_IP` the fused mid-finger
hinge.

**Limits** match the CAD/URDF export (`JOINT_ROM.md`) in the firmware's sign
convention.

**Sign convention vs the URDF.** `T_CMC`, `T_MCP` and `T_IP` are modelled
mirrored in the Fusion/URDF export. The wire uses the **firmware**
convention in the table above. If your kinematics come from the URDF, negate
those three joints (URDF `T_DP` for `T_IP`) when converting to and from the
wire. `T_ABD` is **not** mirrored.

**Positive abduction** is to the right when viewed from the palm, which is
also the direction those joints home in. None of the four abduction joints
are the same, and only the middle one is symmetric.

**Speed ceilings.** The hand never slews a joint faster than its ceiling.
The wrist is capped at 60% of full speed because it swings the whole hand.
Use `C speed` to set anything slower (§10.2).

---

## 7. `T` — targets (PC → hand)

```
T <seq> <a0> <a1> ... <a18>*<cs>
```

Exactly **20 integers**: `seq` then 19 angles in tenths of a degree, in §6
order. An angle must lie within −32768…32767. Anything outside that range
is a format error.

### 7.1 Clamping

A target outside a joint's limits is **clamped to the nearest limit, not
rejected**. That's deliberate: a tracked human hand routinely exceeds the
robot's range, and refusing those frames would make the hand stutter
instead of holding at its limit. Unlike v2, clamping is **reported**: the
reply's `clamp` field has a bit set for every joint that was clamped (§8).

### 7.2 Examples

All joints to zero:
```
T 7 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0*53
```

Index finger curled: `I_P` 90.0°, `I_IP` 105.0°, the rest at zero:
```
T 8 0 0 0 0 0 0 0 0 0 0 900 1050 0 0 0 0 0 0 0*61
```

`P_MP` asked for −40.0°, beyond its −35.0° limit. It's clamped, and bit 0 of
the reply's `clamp` field is set:
```
T 9 -400 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0*74
```

---

## 8. `S` — state (hand → PC)

Sent **only** as the reply to a valid `T`.

```
S <seq> <flags> <clamp> <fb> <a0> <m0> <a1> <m1> ... <a18> <m18>*<cs>
```

| Field | Format | Meaning |
|---|---|---|
| `seq` | decimal | echo of the `T` being answered |
| `flags` | 2 hex digits | status bits, see below |
| `clamp` | 5 hex digits | bit *n* set = joint *n*'s target in **this** `T` was clamped |
| `fb` | 5 hex digits | bit *n* set = `a`*n* is a live, trusted measurement |
| `a`*n* | integer, tenths of ° | measured angle of joint *n* |
| `m`*n* | integer, mA | current of joint *n* |

Bit *n* of `clamp` and `fb` refers to joint index *n* in §6. Bit 0 is the
least significant bit, and `0FFFF` means joints 0–15.

### 8.1 `flags`

| Bit | Value | Name | Meaning |
|--:|--:|---|---|
| 0 | `01` | `WDOG` | The watchdog fired since the previous `S`: the hand saw no valid `T` for 250 ms and froze its targets. Your loop stalled. |
| 1 | `02` | `GAP` | This `T`'s `seq` was not the previous `seq + 1`. A frame was lost or rejected in between. |
| 2 | `04` | `NODRV` | The PWM driver was not detected at boot. Joints 0–15 cannot move. |
| 3 | `08` | `ESTOP` | The E-stop is latched (§10.5). Targets are ignored until `C reset`. |
| 4 | `10` | `FAULT` | At least one joint has been stopped by the current or position protection (§10.5). |
| 5–7 | — | — | reserved, always 0 |

`WDOG` is reported once, on the first reply after it fires, and then clears.

### 8.2 Reading the values

* **Angles** come from the joints' feedback pots and are read when the reply
  is built. If a joint's `fb` bit is clear (never homed, feedback not
  calibrated, or the motor rail is off), its angle is the last known value.
  **Don't use it for control.**
* **Current, joints 0–15:** per motor, from a converter that measures one
  channel every 160 ms. A full sweep of the 16 channels takes **~2.6 s**, so
  most values in any one frame are old. Don't build fast reactions on
  current.
* **Current, joints 16–18:** the **total** for the whole wrist rail, from
  one shared sensor. All three report the same number. It's fresh every
  frame, but it cannot tell you which wrist axis is loaded.

### 8.3 Example

Reply to `T 8` above, with the index finger on its way to the target:
```
S 8 00 00000 0FFFF 0 12 0 8 0 10 0 11 0 7 0 9 0 10 0 8 0 9 0 12 412 211 388 185 0 9 0 8 0 10 0 9 0 340 0 340 0 340*68
```
No flags, no clamping. Joints 0–15 have trusted feedback; the three wrist
joints (bits 16–18) do not.

A full `S` frame is at most about 240 bytes.

---

## 9. Watchdog

The watchdog is **armed by the first valid `T`** after entering PC mode. It
is disarmed while a routine runs (§10) and re-armed by the next valid `T`.

If **250 ms** pass without a valid `T` (about 7 missed frames), the hand
freezes every joint's target at its current commanded position, so nothing
keeps driving toward a stale target. It sends **nothing** when this happens.
You learn about it from the `WDOG` flag on the next `S`, once you're sending
again. Sending a valid `T` resumes normal operation immediately.

---

## 10. `C` — commands and routines (PC → hand)

```
C <seq> <name> [<arg> ...]*<cs>
```

Every `C` is answered with **`K`** straight away, then zero or more lines of
log text, then **`D`**. If the command is rejected before it starts, it gets
a single **`E`** instead, and no `K` or `D`.

```
hand → PC    K <seq> <name>*<cs>        started
             ...                        free-form log text (not checksummed)
hand → PC    D <seq> <name>*<cs>        finished; send T frames again
```

**Rules between `K` and `D`:**

* **Do not send `T` frames.** Routines block the firmware, so frames sent
  meanwhile are not answered, and **anything received during a routine is
  discarded when it finishes.**
* Treat every line as log text except a **checksum-valid `D`** with your
  `seq`. The checksum is what makes that match unambiguous.
* After `D`, carry on with the next `seq`. Motion targets have already been
  re-seeded from measured positions, so the next `T` won't cause a jump.

### 10.1 Command list

| Command | Duration | Effect |
|---|---|---|
| `C <s> demo1` | ~30 s | movement demonstration |
| `C <s> demo2` | ~20 s | thumb touches each fingertip in turn |
| `C <s> test` | ~20 s | centre all, sweep each joint, PASS/FAIL report in the log |
| `C <s> sweep_r <joint>` | ~4 s | sweep one joint through its full homed range |
| `C <s> homeall` | minutes | automatic stall-detect homing of joints 0–15, saved to flash |
| `C <s> manualhome` | interactive | manual homing of all 19 joints, see §10.4 |
| `C <s> manualhome <joint>` | interactive | manual re-home of one joint |
| `C <s> fbfind` | ~1 min | discover and calibrate feedback channels |
| `C <s> zero` | instant | all joints to 0° |
| `C <s> center` | instant | all joints to servo neutral (1500 µs); not reachable via `T` |
| `C <s> speed <joint> <dps>` | instant | set one joint's slew rate |
| `C <s> speed all <dps>` | instant | set every joint's slew rate |
| `C <s> stats` | instant | link statistics, see §10.3 |
| `C <s> estop` | instant | latch the E-stop, like `!` but acknowledged, see §10.5 |
| `C <s> reset` | instant | clear the E-stop and all faults; the hand holds where it is |
| `C <s> faults` | instant | which protection tripped, as an `I` line, see §10.5 |

`<joint>` is a **name** from §6 (`I_P`, `W_2`, …). Channel numbers are not
accepted.

**Aborting.** `demo1`, `demo2`, `test` and `sweep_r` stop as soon as any byte
arrives. Send `abort\n` to cancel one; `D` still follows. `homeall` and
`fbfind` cannot be aborted.

### 10.2 `speed`

```
C 11 speed W_2 120*2D     →   K 11 speed*2C
                              D 11 speed*23
```

`<dps>` is an integer in degrees per second. It's clamped to between 1 and
that joint's ceiling (§6). The setting lasts until changed, and resets to
the ceiling every time PC mode is entered.

You rarely need this for teleoperation. At 30 Hz your target stream already
defines the speed. It's for deliberate slow moves, such as gliding into a
preset pose from a button.

### 10.3 `stats`

Between `K` and `D`, `stats` produces one extra line, an `I` frame, which is
checksummed:

```
C 13 stats*20
K 13 stats*28
I rx=1804 ok=1799 cs=2 fmt=1 long=0 unk=2 wd=0 loop=2140*60
D 13 stats*27
```

| Key | Meaning |
|---|---|
| `rx` | lines received in PC mode |
| `ok` | lines accepted (valid `T` or `C`) |
| `cs` | rejected: checksum missing or wrong |
| `fmt` | rejected: bad layout or numbers, or a `C` rejected with `badcmd` / `badjoint` / `badarg` |
| `long` | rejected: over 255 characters |
| `unk` | rejected: unknown frame type |
| `wd` | times the watchdog fired |
| `loop` | slowest main-loop pass since the last `stats`, µs |

Counters run from when PC mode was last entered. `loop` resets each time it
is read. It's the quickest way to check whether the link is healthy: steady
`ok`, zero `cs`, and `loop` well under 33 000.

### 10.4 `manualhome` is interactive

Between `K` and `D` the hand prompts for input and waits. Answer each prompt
with a plain line (**no** `seq`, **no** checksum):

| Send | Effect |
|---|---|
| `10\n`, `-5\n`, … | jog the joint by that many µs |
| `d\n` | this joint is at its limit, move to the next |

Show the log lines to the operator, relay what they type, and wait for `D`.

### 10.5 E-stop and joint protection

**E-stop.** Send the bare line `!\n` (no `seq`, no checksum). It works at any
time, including in the middle of a routine, which it ends. The hand sends no
reply; the next `S` carries the `ESTOP` flag (`08`). `C <s> estop` does the
same but is acknowledged with `K`/`D`.

While the E-stop is latched the hand **still answers every `T`** with an `S`
(positions and currents) but ignores the targets, so keep the 30 Hz loop
running. Any command that would move the hand is refused with
`E <seq> estop`.

**Reset.** `C <s> reset` clears the E-stop and every fault. The hand holds
where it is, so re-seed your targets from the next `S` before moving (the
PC app runs its usual 1 deg/s resync, §4, and adopts the measured pose).

**Faults.** The `FAULT` flag (`10`) means at least one joint has been stopped
by the current or position protection. In particular, a finger commanded
more than **20°** past where an object stops it trips the protection and
lets go. `C <s> faults` reports what tripped, as one `I` line between `K`
and `D`:

```
C 21 faults*59
K 21 faults*51
I estop=1 P_P=deviation*39
D 21 faults*5E
```

`estop=` is 0 or 1; every other key is a joint name (§6) with the reason it
was stopped.

---

## 11. `E` — errors (hand → PC)

```
E <seq|-> <code>*<cs>
```

`seq` is the rejected frame's sequence number if it could be read reliably,
otherwise `-`.

| Code | `seq` | Cause |
|---|---|---|
| `checksum` | `-` | no `*`, malformed checksum, or checksum mismatch |
| `toolong` | `-` | line over 255 characters; the whole line was discarded |
| `unknown` | `-` | valid checksum, but not a `T` or `C` frame |
| `format` | seq or `-` | valid checksum, but wrong field count, bad spacing, or a bad number |
| `badcmd` | seq | unknown routine name |
| `badjoint` | seq | unknown joint name |
| `badarg` | seq | missing, extra or invalid argument |
| `estop` | seq | the E-stop is latched and this command would move the hand (§10.5) |

A rejected `T` changes nothing: the previous targets stay in force.

```
E - checksum*65
E 10 format*47
```

---

## 12. Timing summary

| | |
|---|---|
| PC `T` period | **33.3 ms (30 Hz)**, PC-driven |
| Hand reply latency | typically < 3 ms after the `T` arrives |
| Hand slew limiter | 20 ms (50 Hz), independent of the link |
| Watchdog | 250 ms without a valid `T` |
| Current, joints 0–15 | one channel per 160 ms, ~2.6 s per full sweep |
| Current, joints 16–18 | fresh every reply |
| Link load | `T` ≤ ~125 B + `S` ≤ ~240 B at 30 Hz ≈ 11 kB/s, ~12% of 921600 baud |

---

## 13. Reference client (Python)

A complete, working client. It has been run against the firmware's protocol
code.

```python
import time
import serial   # pip install pyserial

JOINTS = ["P_MP", "P_P", "P_IP", "R_MP", "R_P", "R_IP",
          "M_MP", "M_P", "M_IP", "I_MP", "I_P", "I_IP",
          "T_CMC", "T_ABD", "T_MCP", "T_IP",
          "W_1", "W_2", "W_3"]

# (min, max) per joint in TENTHS of a degree, same order as JOINTS.
LIMITS = [(-350, 30), (0, 900), (0, 1160), (-240, 200), (0, 900), (0, 1160),
          (-240, 240), (0, 900), (0, 1160), (-200, 240), (0, 900), (0, 1160),
          (-370, 820), (-320, 700), (-650, 950), (-630, 980),
          (-900, 900), (-550, 550), (-165, 165)]

PERIOD_S = 1.0 / 30.0
FLAG_WDOG, FLAG_GAP, FLAG_NODRV = 0x01, 0x02, 0x04


def checksum(body: str) -> str:
    x = 0
    for b in body.encode("ascii"):
        x ^= b
    return f"{x:02X}"


def frame(body: str) -> bytes:
    return f"{body}*{checksum(body)}\n".encode("ascii")


def parse(line: str):
    """-> (type, [fields]) for a checksum-valid frame, else None."""
    star = line.rfind("*")
    if star < 1 or len(line) - star != 3:
        return None
    body, cs = line[:star], line[star + 1:]
    if checksum(body) != cs.upper():
        return None
    parts = body.split(" ")
    return parts[0], parts[1:]


class State:
    def __init__(self, f):
        self.seq = int(f[0])
        self.flags = int(f[1], 16)
        self.clamp = int(f[2], 16)
        self.fb = int(f[3], 16)
        v = [int(x) for x in f[4:]]
        self.deg = [a / 10.0 for a in v[0::2]]   # degrees
        self.ma = v[1::2]                        # milliamps

    def trusted(self, i):
        return bool(self.fb >> i & 1)

    def clamped(self, i):
        return bool(self.clamp >> i & 1)


class Hand:
    def __init__(self, port, baud=921600):
        self.ser = serial.Serial(port, baud, timeout=0)   # non-blocking
        self.buf = b""
        self.seq = 0

    # ---- low level -----------------------------------------------------
    def _lines(self):
        """Every complete line received so far, without its line ending."""
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        *done, self.buf = self.buf.split(b"\n")
        return [d.rstrip(b"\r").decode("ascii", "replace") for d in done]

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    # ---- handshake -----------------------------------------------------
    def enter_pc_mode(self, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.ser.write(b"pc\n")
            t = time.monotonic() + 0.5
            while time.monotonic() < t:
                if "MODE PC" in self._lines():
                    self.buf = b""
                    return
                time.sleep(0.01)
        raise TimeoutError("hand never answered MODE PC")

    def close(self):
        self.ser.write(b"term\n")
        self.ser.close()

    # ---- fast path -----------------------------------------------------
    def send_targets(self, deg):
        """deg: 19 floats in degrees, JOINTS order. Returns the seq used."""
        if len(deg) != 19:
            raise ValueError(f"need 19 angles, got {len(deg)}")
        vals = [max(lo, min(hi, round(d * 10))) for d, (lo, hi) in zip(deg, LIMITS)]
        seq = self._next_seq()
        self.ser.write(frame(f"T {seq} " + " ".join(map(str, vals))))
        return seq

    def poll(self):
        """Replies that have arrived, as ('S', State) or ('E', seq, code)."""
        out = []
        for line in self._lines():
            p = parse(line)
            if p is None:
                continue                 # not a valid frame: ignore
            kind, f = p
            if kind == "S" and len(f) == 42:
                out.append(("S", State(f)))
            elif kind == "E":
                out.append(("E", f[0], f[1]))
        return out

    # ---- routines ------------------------------------------------------
    def command(self, *words, on_log=print, timeout=600.0):
        """Run a C command to completion. Returns its log lines.
           Do not call send_targets() while this is running."""
        seq = self._next_seq()
        self.ser.write(frame(f"C {seq} " + " ".join(words)))
        log, started = [], False
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for line in self._lines():
                p = parse(line)
                if p and p[1] and p[1][0] == str(seq):
                    if p[0] == "E":
                        raise RuntimeError(f"hand rejected {words}: {p[1][1]}")
                    if p[0] == "K":
                        started = True
                        continue
                    if p[0] == "D":
                        return log
                if started:
                    log.append(line)
                    if on_log:
                        on_log(line)
            time.sleep(0.005)
        raise TimeoutError(f"{words} did not finish")


def control_loop(hand, get_targets, on_state, running):
    """Send get_targets() at 30 Hz; hand each reply to on_state(State).
       running() -> False to stop."""
    deadline = time.perf_counter()
    while running():
        hand.send_targets(get_targets())
        for r in hand.poll():
            if r[0] == "S":
                on_state(r[1])
            else:
                print("hand error:", r[1], r[2])
        deadline += PERIOD_S
        delay = deadline - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:
            deadline = time.perf_counter()   # fell behind: reset, don't burst


# Example:
#   hand = Hand("COM5")
#   hand.enter_pc_mode()
#   hand.command("speed", "W_2", "120")
#   control_loop(hand, lambda: [0.0] * 19, lambda s: None, lambda: True)
```

---

## 14. Migrating a v2 client

| | v2 | v3 |
|---|---|---|
| Who sets the rate | both sides, independently | **the PC only** |
| Rate | 50 Hz each way | **30 Hz**, one reply per frame |
| Unsolicited frames | `S` every 20 ms, `E timeout` | **none** |
| `T` layout | `T <deg> <spd>` × 19 | `T <seq> <a>` × 19, **no speed** |
| Angle format | float degrees | **integer tenths of a degree** |
| Integrity | none | **XOR checksum on every frame** |
| Sequence numbers | none | **`seq` echoed in every reply** |
| `S` layout | `S <deg> <mA>` × 19 | `S <seq> <flags> <clamp> <fb>` + `<a> <m>` × 19 |
| Clamping | silent | **reported** in `clamp` |
| Feedback validity | not reported | **reported** in `fb` |
| Speed | per joint in every `T` | `C speed`, occasional |
| Watchdog | 300 ms, sends `E timeout` | **250 ms, silent**, `WDOG` flag on next `S` |
| `C` frames | `C <name>` | `C <seq> <name>*cs` |
| `K`/`D` markers | `K <name>` | `K <seq> <name>*cs` |
| Joint arguments | name or PWM channel | **name only** |
| Repeated `pc` | `E unknown` | `MODE PC` (session reset) |
| New | | `C speed`, `C stats` + `I` line, `abort` |

The steps:

1. Replace the send loop with the fixed-rate, non-blocking loop in §13.
2. Remove speed from `T`; call `C speed` wherever speed actually varied.
3. Convert angles to integer tenths going out and back to degrees coming in.
4. Add the checksum to everything sent. Verify it on everything received,
   and drop any line that fails.
5. Match `S` to `T` by `seq`. Log the `GAP` and `WDOG` flags; if they are
   frequent, the PC loop is stalling.
6. Use `fb` to decide which measured angles to trust, and `clamp` to show
   the operator when the hand is at a limit.
7. Parse `K`/`D` with their `seq` and checksum, and stop sending `T` between
   them.
