# humanoi-ar4-arm

AR4 robot arms (a clone of the Annin Robotics AR4 — Teensy 4.1 + Arduino Nano) for the Humanoi robot — ROS 2 Jazzy

Humanoi has two AR4 arms mounted on the same column, each ending in an AmazingHand — this repo holds everything for the arms:
the two-arm + hand model, ros2_control driving both boards at once, MoveIt, the pose program player, and our own Teensy firmware
(joint-by-joint calibration · arm name and joint zero trims stored on the board, so the numbers in programs are true angles)
**Full project overview:** [humanoi README](https://github.com/Thenine2077/humanoi#readme)

Used together with the **[humanoi](https://github.com/Thenine2077/humanoi)** repo (base/Pi/Docker/manuals) — clone both side by side and name this repo's folder `ar4`:

```
Desktop/
├── Humanoi/   ← humanoi repo   (scripts/sync_to_pi.sh copies the packages from ../ar4 to the Pi)
└── ar4/       ← this repo
```

**Arm user manual:** `Humanoi/docs/AR4_ARM_MANUAL.pdf` (`arm` commands, MoveIt, remote control, calibration, troubleshooting)

## Repository layout

```
ar4/
├── ar4_ws/src/
│   ├── ar_description/         ┐
│   ├── ar_hardware_interface/  ├ from the vendor = ar4_ros_driver (open source, MIT) — do not edit these files;
│   ├── ar_moveit_config/       ┘   put fixes in our own packages instead
│   ├── ar_gz_bringup/          ours: Gazebo Harmonic sim + MoveIt for Jazzy (ar_moveit_jazzy.launch.py)
│   ├── ar_mirror/              ours: sim + real arm running together (dual_real_sim) + joint_mirror
│   ├── ar_dual/                ours: two arms on the column — upper-body model (column, head, camera, two arms, AmazingHand) shared by
│   │                               the real arms/MoveIt/Gazebo, ros2_control for 2 boards, MoveIt collision checking,
│   │                               config/zero_trims.yaml (joint zero trims + after-calibration pose),
│   │                               ar_ports.py (find ports/name boards/identify/jog), ar_calibrate.py, ar_zero.py, flash_teensy.sh
│   └── ar_program/             ours: plays pre-taught poses (arm_program) + teach / goto / delete pose
│       └── config/programs.yaml    taught poses and programs (the live copy is on the Pi)
├── firmware/
│   ├── AR4_ROS2_seq/           ours (v3): AR4_ROS2 + joint-by-joint calibration, arm name/motor directions/zero trims in EEPROM
│   │                               (currently on both arms)
│   ├── AR4_ROS2/               the vendor's original Teensy firmware for ROS (do not edit)
│   ├── AR4_teensy41_v43_HMI/   Teensy firmware for the HMI program (AR4.py) — from the vendor
│   ├── AR4_nano_sketch/        the vendor's original Nano firmware
│   └── AR4_nano_inputs/        ours: Nano v2 — active-LOW relays stay off at boot + inputs D3–D7
├── tools/                      serial↔TCP bridge (Windows ↔ WSL), COM / HMI board tests
├── run_ar4_sim.bat             double-click to open the Gazebo + MoveIt sim on a PC (WSL) (single arm)
└── run_dual_sim.bat            opens two simulated (mock) arms + MoveIt + program player on a PC — teach poses/try programs without the Pi
```

## Usage

| Where | What | See |
|---|---|---|
| Raspberry Pi (real arms) | `arm start` / `arm player` / `arm run <program>` / `arm moveit` / `arm remote 1` | PDF manual in the humanoi repo |
| PC (WSL) sim | double-click `run_ar4_sim.bat` or `ros2 launch ar_gz_bringup ar_gz.launch.py` | the .bat file |
| PC (WSL) two-arm sim | double-click `run_dual_sim.bat`, then in RViz: File > Open Config > `dual_moveit.rviz` | the .bat file |
| PC (WSL) real arm | `tools/serial_bridge_win.py` + `tools/serial_bridge_wsl.sh`, then launch with `serial_port:=/dev/ttyV0` | file headers in tools/ |

Build on a PC (WSL Ubuntu-24.04):

```bash
cd /mnt/c/Users/User/Desktop/ar4/ar4_ws
colcon build --symlink-install && source install/setup.bash
```

## Firmware

- The Teensy can be flashed from the Pi without moving cables: compile the hex on a PC (Arduino IDE + Teensyduino), then run
  `teensy_loader_cli --mcu=TEENSY41 -w -v file.hex` in the container — details in `Humanoi/docs/PI_DOCKER.md`
- Both arms' Teensys now run **AR4_ROS2_seq v3** (`JV` replies `JVseq3`) — flash one board at a time from the Pi with
  `ar_dual/scripts/flash_teensy.sh <by-path port> <hex>` (the HMI program AR4.py won't work until AR4_teensy41_v43_HMI is flashed back)
- v3 adds `JZ` / `JZ=a,b,c,d,e,f` = joint zero trims (degrees) — reported angle = counted angle + trim ·
  the values in use live in `ar_dual/config/zero_trims.yaml`; write them to the board with `arm zero <arm>` (`arm start` checks that they match)
- **Flashing erases the EEPROM** (arm name, motor directions, zero trims) → `ar_ports.py --name <port> left|right`, then `arm zero <arm>`
- The two Teensys report the same USB serial number — the arms are told apart by the name in EEPROM: `arm identify` (turns joint 6 on one board at a time)
- Never unplug/plug an arm's USB while arm power is on — a Pi 5 once failed after a Teensy cable was unplugged with arm power on (suspected back-feed/ground surge)
- Arduino IDE can't open folders with Chinese characters in their names — use this `firmware/` folder (same copy as `C:\ar4_fw`)

## Good to know

- Program player speed is 0.19 rad/s (cap 0.22) — any faster and the firmware can't keep up, the controller aborts,
  and the J1 belt has slipped with the arm stretched far out
- Calibration takes ~2 minutes with a quiet log — don't press Ctrl+C until the arm is back in its rest pose
- The vendor download files (`sendgb-*`) are not in the repo (large, ~8 GB, and not our code)
