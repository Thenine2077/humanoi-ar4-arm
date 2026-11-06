#!/usr/bin/env python3
"""Test whether the HMI v4.3.2 boards answer commands — with a timeout (doesn't hang like AR4.py)

  python C:\\Users\\User\\Desktop\\ar4\\tools\\test_hmi_boards.py --teensy COM9 --nano COM6

Why it exists: AR4.py opens the port with serial.Serial(port, baud) without setting a timeout, then calls
ser.read() / ser.readline() to wait for a reply; if the board doesn't answer, the window hangs (Not Responding) forever
This script sends the same commands as the HMI but waits only 1.5 seconds, then tells you which board is silent

Commands used — none of them move the arm:
  Teensy (9600)   : RP          request the current position (read-only)
  Nano   (115200) : TMhello     echo test
                    ONX<pin> / OFX<pin>  turn a digital out pin on/off (default pin 14 = A0)

Close AR4.py before running (a port can only be open in one program at a time)
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not found — install it with:  pip install pyserial")

TIMEOUT = 1.5


def ask(ser, cmd, wait_line=True):
    """Send a command and wait up to TIMEOUT seconds for the reply. Returns (text, ms) or (None, ms)"""
    ser.reset_input_buffer()
    t0 = time.monotonic()
    ser.write(cmd.encode())
    buf = b""
    deadline = t0 + TIMEOUT
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if not wait_line or b"\n" in buf:
                # read a little more of what follows to get the whole message
                time.sleep(0.05)
                buf += ser.read(ser.in_waiting)
                break
    ms = (time.monotonic() - t0) * 1000
    return (buf.decode("ascii", "replace").strip() or None) if buf else None, ms


def report(name, cmd, reply, ms, expect=None):
    """expect = exact text required (None = any reply is enough)"""
    shown = cmd.strip()
    if reply is None:
        print(f"  ✗ {shown:<10} -> no reply within {TIMEOUT:.1f} s   <-- AR4.py would hang here")
        return False
    if expect is not None and reply != expect:
        print(f"  ✗ {shown:<10} -> {reply[:70]!r}  (expected {expect!r} — the command didn't actually run)")
        return False
    print(f"  ✓ {shown:<10} -> {reply[:70]!r}  ({ms:.0f} ms)")
    return True


def test_teensy(port):
    print(f"\n[Teensy] {port} @ 9600")
    try:
        ser = serial.Serial(port, 9600, timeout=0.05)
    except Exception as exc:
        print(f"  ✗ cannot open the port: {exc}")
        return False
    with ser:
        time.sleep(0.3)
        reply, ms = ask(ser, "RP\n")
        ok = report("Teensy", "RP", reply, ms)
        if ok and not reply.startswith("A"):
            print("    ⚠ replied, but not in the HMI position format (A..B..C..) — the firmware may not be v4.3")
        return ok


def test_nano(port, pin):
    print(f"\n[Nano] {port} @ 115200")
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except Exception as exc:
        print(f"  ✗ cannot open the port: {exc}")
        return False
    with ser:
        # The Nano (CH340) resets itself every time the port is opened (DTR line) — the bootloader takes ~1-2 s
        # commands sent earlier than that are silently lost
        print("  (waiting 2.5 s for the Nano to boot after opening the port)")
        time.sleep(2.5)
        # ON/OF reply "Done" (no newline) — must get exactly this to count the pin as really changed
        r, ms = ask(ser, f"ONX{pin}\n", wait_line=False)
        ok = report("Nano", f"ONX{pin}", r, ms, expect="Done")
        if ok:
            # v2 firmware (C:\ar4_fw\AR4_nano_inputs) inverts the logic for active-LOW relays:
            # ON = pin LOW = relay on, whereas with the vendor firmware ON = pin HIGH = relay off
            # confirmed relay channel: pin 14 -> relay channel 8 (order reversed from the pin numbers)
            print(f"    -> ON pin {pin} held for 2 seconds: v2 firmware = this pin's relay on"
                  f" / vendor firmware = pin at 5V (relay off)")
            time.sleep(2.0)
        r, ms = ask(ser, f"OFX{pin}\n", wait_line=False)
        ok &= report("Nano", f"OFX{pin}", r, ms, expect="Done")
        # TM must always be the last command: the vendor's Nano firmware doesn't clear its buffer after TM
        # (else { inData = ""; } belongs to the if TM), so the command after TM gets appended and parsed as TM
        # again — it doesn't actually run, but a reply still comes back, so it looks like a pass
        ok &= report("Nano", "TMhello", *ask(ser, "TMhello\n"), expect="hello")
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--teensy", default="COM9")
    ap.add_argument("--nano", default="COM6")
    ap.add_argument("--pin", type=int, default=14, help="digital out pin to try (14-19)")
    args = ap.parse_args()
    if not 14 <= args.pin <= 19:
        sys.exit("The Nano firmware only has output pins 14-19 (A0-A5)")

    t_ok = test_teensy(args.teensy)
    n_ok = test_nano(args.nano, args.pin)

    print("\n===== Summary =====")
    print(f"  Teensy : {'responding' if t_ok else 'no reply / cannot open'}")
    print(f"  Nano   : {'responding' if n_ok else 'no reply / cannot open'}")


if __name__ == "__main__":
    main()
