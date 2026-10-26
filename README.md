# humanoi-ar4-arm

แขนกล AR4 (เครื่อง clone ของ Annin Robotics AR4 — Teensy 4.1 + Arduino Nano) สำหรับหุ่น Humanoi — ROS 2 Jazzy

หุ่น Humanoi มีแขน AR4 สองตัวติดเสาเดียวกัน ปลายแขนเป็นมือ AmazingHand — repo นี้คือทุกอย่างของแขน:
โมเดลสองแขน + มือ, ros2_control ที่คุมสองบอร์ดพร้อมกัน, MoveIt, ตัวเล่นโปรแกรมท่าทาง และเฟิร์มแวร์ Teensy ฉบับของเรา
(คาลิเบรตทีละข้อ · ชื่อแขนและค่าชดเชยศูนย์ข้อต่อเก็บในบอร์ด ตัวเลขในโปรแกรม = มุมจริง)
**ภาพรวมโปรเจกต์ทั้งหมด:** [README ของ humanoi](https://github.com/Thenine2077/humanoi#readme)

ใช้คู่กับ repo **[humanoi](https://github.com/Thenine2077/humanoi)** (ฐาน/Pi/Docker/คู่มือ) — clone ไว้ข้างกัน โดยตั้งชื่อโฟลเดอร์ repo นี้ว่า `ar4`:

```
Desktop/
├── Humanoi/   ← repo humanoi   (scripts/sync_to_pi.sh ส่งแพ็กเกจจาก ../ar4 ไป Pi)
└── ar4/       ← repo นี้
```

**คู่มือใช้งานแขน:** `Humanoi/docs/AR4_ARM_MANUAL.pdf` (คำสั่ง `arm`, MoveIt, รีโมท, calibrate, แก้ปัญหา)

## โครงสร้าง

```
ar4/
├── ar4_ws/src/
│   ├── ar_description/         ┐
│   ├── ar_hardware_interface/  ├ จากผู้ขาย = ar4_ros_driver (open source, MIT) — ห้ามแก้ไฟล์เหล่านี้
│   ├── ar_moveit_config/       ┘   แก้ปัญหาใส่ในแพ็กเกจของเราแทน
│   ├── ar_gz_bringup/          ของเรา: ซิม Gazebo Harmonic + MoveIt สำหรับ Jazzy (ar_moveit_jazzy.launch.py)
│   ├── ar_mirror/              ของเรา: ชุดซิม + แขนจริงพร้อมกัน (dual_real_sim) + joint_mirror
│   ├── ar_dual/                ของเรา: สองแขนบนเสา — โมเดลส่วนบน (เสา หัว กล้อง สองแขน มือ AmazingHand) ใช้ร่วม
│   │                               แขนจริง/MoveIt/Gazebo, ros2_control 2 บอร์ด, MoveIt เช็กการชน,
│   │                               config/zero_trims.yaml (ค่าชดเชยศูนย์ข้อต่อ + ท่าหลังคาลิเบรต),
│   │                               ar_ports.py (หาพอร์ต/ตั้งชื่อบอร์ด/identify/jog), ar_calibrate.py, ar_zero.py, flash_teensy.sh
│   └── ar_program/             ของเรา: เล่นท่าที่ตั้งไว้ล่วงหน้า (arm_program) + teach / goto / ลบท่า
│       └── config/programs.yaml    ท่าและโปรแกรมที่ teach ไว้ (ตัวที่ใช้จริงอยู่บน Pi)
├── firmware/
│   ├── AR4_ROS2_seq/           ของเรา (v3): AR4_ROS2 + calibrate ทีละข้อ, ชื่อแขน/ทิศมอเตอร์/ค่าชดเชยศูนย์ใน EEPROM
│   │                               (ใช้อยู่ตอนนี้ทั้งสองแขน)
│   ├── AR4_ROS2/               เฟิร์มแวร์ Teensy สำหรับ ROS ต้นฉบับผู้ขาย (ห้ามแก้)
│   ├── AR4_teensy41_v43_HMI/   เฟิร์มแวร์ Teensy สำหรับโปรแกรม HMI (AR4.py) — จากผู้ขาย
│   ├── AR4_nano_sketch/        เฟิร์มแวร์ Nano ต้นฉบับผู้ขาย
│   └── AR4_nano_inputs/        ของเรา: Nano v2 — relay active-LOW ไม่ติดตอนบูต + อินพุต D3–D7
├── tools/                      สะพาน serial↔TCP (Windows ↔ WSL), ทดสอบ COM / บอร์ด HMI
├── run_ar4_sim.bat             เปิดซิม Gazebo + MoveIt บน PC (WSL) ด้วยดับเบิลคลิก (แขนเดียว)
└── run_dual_sim.bat            เปิดสองแขนจำลอง (mock) + MoveIt + ตัวเล่นโปรแกรม บน PC — สอนท่า/ลองโปรแกรมได้โดยไม่มี Pi
```

## ใช้งาน

| ที่ไหน | ทำอะไร | ดู |
|---|---|---|
| Raspberry Pi (แขนจริง) | `arm start` / `arm player` / `arm run <โปรแกรม>` / `arm moveit` / `arm remote 1` | คู่มือ PDF ใน repo humanoi |
| PC (WSL) ซิม | ดับเบิลคลิก `run_ar4_sim.bat` หรือ `ros2 launch ar_gz_bringup ar_gz.launch.py` | ไฟล์ .bat |
| PC (WSL) ซิมสองแขน | ดับเบิลคลิก `run_dual_sim.bat` แล้วใน RViz: File > Open Config > `dual_moveit.rviz` | ไฟล์ .bat |
| PC (WSL) แขนจริง | `tools/serial_bridge_win.py` + `tools/serial_bridge_wsl.sh` แล้ว launch ด้วย `serial_port:=/dev/ttyV0` | หัวไฟล์ใน tools/ |

Build บน PC (WSL Ubuntu-24.04):

```bash
cd /mnt/c/Users/User/Desktop/ar4/ar4_ws
colcon build --symlink-install && source install/setup.bash
```

## เฟิร์มแวร์

- แฟลช Teensy จาก Pi ได้โดยไม่ต้องย้ายสาย: คอมไพล์ hex บน PC (Arduino IDE + Teensyduino) แล้ว
  `teensy_loader_cli --mcu=TEENSY41 -w -v ไฟล์.hex` ใน container — รายละเอียดใน `Humanoi/docs/PI_DOCKER.md`
- ตอนนี้ Teensy ทั้งสองแขนเป็น **AR4_ROS2_seq v3** (`JV` ตอบ `JVseq3`) — แฟลชทีละบอร์ดจาก Pi ด้วย
  `ar_dual/scripts/flash_teensy.sh <พอร์ต by-path> <hex>` (โปรแกรม HMI AR4.py ใช้ไม่ได้จนกว่าจะแฟลช AR4_teensy41_v43_HMI กลับ)
- v3 เพิ่ม `JZ` / `JZ=a,b,c,d,e,f` = ค่าชดเชยศูนย์ข้อต่อ (องศา) — มุมที่รายงาน = มุมที่นับได้ + ค่าชดเชย ·
  ค่าที่ใช้อยู่ในไฟล์ `ar_dual/config/zero_trims.yaml` เขียนลงบอร์ดด้วย `arm zero <แขน>` (`arm start` เช็กว่าตรง)
- **แฟลชแล้ว EEPROM ถูกล้าง** (ชื่อแขน ทิศมอเตอร์ ค่าชดเชย) → `ar_ports.py --name <พอร์ต> left|right` แล้ว `arm zero <แขน>`
- Teensy สองตัวรายงานหมายเลข USB ซ้ำกัน — แยกแขนด้วยชื่อใน EEPROM: `arm identify` (ข้อ 6 หมุนให้ดูทีละบอร์ด)
- ห้ามถอด/เสียบ USB แขนขณะไฟแขนเปิด — Pi 5 เคยเสียหลังถอดสาย Teensy ตอนไฟแขนเปิด (สงสัยไฟย้อน/กราวด์กระชาก)
- Arduino IDE เปิดโฟลเดอร์ที่มีภาษาจีนในชื่อไม่ได้ — ใช้โฟลเดอร์ `firmware/` นี้ (สำเนาเดียวกับ `C:\ar4_fw`)

## ข้อควรรู้

- ความเร็วตัวเล่นโปรแกรม 0.19 rad/s (เพดาน 0.22) — เร็วกว่านี้เฟิร์มแวร์ตามไม่ทัน controller จะ abort
  และสายพาน J1 เคยรูดตอนแขนยื่นไกล
- calibrate ใช้เวลา ~2 นาที log เงียบ — ห้าม Ctrl+C จนแขนกลับท่าพัก
- ไฟล์ดาวน์โหลดจากผู้ขาย (`sendgb-*`) ไม่ได้อยู่ใน repo (ใหญ่ ~8 GB และไม่ใช่โค้ดของเรา)
