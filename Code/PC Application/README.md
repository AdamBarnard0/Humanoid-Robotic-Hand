# HRH1 PC Application

PC control application for **HRH1**, a low-cost, tendon-driven humanoid robotic hand with
16 actuated degrees of freedom in the hand and a 3-DOF wrist (19 MG996R servos, ESP32
controller). Developed as a Mechatronic Project 478 (skripsie) at Stellenbosch University.

The application connects to the hand's firmware over USB serial and provides:

- manual control of all 19 joints with live measured positions
- preset poses, and access to the firmware's demonstration and calibration routines
- a live 3D model of the hand, built from the CAD (URDF) export
- webcam hand-tracking teleoperation (MediaPipe landmarks + optimisation-based IK retargeting)
- a digital E-stop, joint-fault display, current graph and link diagnostics

## Requirements

- Python 3.11+
- The HRH1 hand running its PC-mode firmware, connected over USB (921600 baud)
- A webcam for hand tracking (optional)

## Installation

```bash
pip install -r requirements.txt
```

## Running

```bash
python main.py
```

Select the serial port and press **Connect**. The first time hand tracking is started, the
MediaPipe hand landmark model (~10 MB) is downloaded into `models/`.

Press **E-STOP** (or **Esc**) at any time to stop the hand; **Reset** clears it.

## Project structure

```
main.py                     Entry point
PROTOCOL.md                 Serial protocol between the PC and the firmware
URDFs/                      Fusion 360 URDF export of HRH1 (meshes + kinematics)
hrh/
  gui.py                    Main window (PySide6)
  theme.py, widgets.py      Stylesheet and custom widgets
  joints.py                 19-joint table (names, order, ranges) matching the firmware
  gestures.py               Preset poses
  link.py                   Serial link: protocol framing, 30 Hz exchange, resync, E-stop
  urdf_model.py             URDF parser and forward kinematics
  hrh1_urdf_map.py          Firmware-to-URDF joint name and sign mapping
  hand_view3d_urdf.py       3D hand view (PyVista)
  camera.py                 Webcam capture and mode selection
  vision.py                 Capture + MediaPipe hand landmarker worker
  tracking.py               Hand selection, handedness, filtering, retarget worker
  ik_retarget.py            Finger/thumb IK retargeting (Levenberg-Marquardt)
  wrist_retarget.py         Wrist orientation retargeting
  one_euro.py               One Euro filter
```

## Serial protocol

The PC and firmware exchange line-based ASCII frames with checksums: the PC sends a target
frame (`T`) 30 times per second and the hand replies with a state frame (`S`) carrying the
measured angle and current of every joint. See [PROTOCOL.md](PROTOCOL.md) for the full
specification, so that other software can drive the hand.

## Author

Adam Barnard, Department of Mechanical and Mechatronic Engineering, Stellenbosch University (2026).
