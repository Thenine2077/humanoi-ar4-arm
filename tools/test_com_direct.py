#!/usr/bin/env python3
"""Test the Teensy COM port directly on Windows — no WSL, no bridge, no usbip

Cuts the variables down to a minimum: if this still drops out = the problem is the board/cable/driver
if this survives a long run = the problem is in a layer above (bridge / usbip / ROS)

  # read-only mode (safest — the arm definitely won't move)
  python C:\\Users\\User\\Desktop\\ar4\\tools\\test_com_direct.py --port COM9 --seconds 60

  # full ROS imitation mode — includes "MT" (move command)
  python C:\\Users\\User\\Desktop\\ar4\\tools\\test_com_direct.py --port COM9 --mt --hz 30 --seconds 120

Normal mode only sends "JP\\n" (3 bytes, read position) — it does cut variables, but it does "not cover"
what ROS really does, because ARHardwareInterface sends two commands per cycle:
    read()  -> "JP\\n"        3 bytes
    write() -> "MTA...F...\\n" 67 bytes  <-- this is the heavy one, and it actually drives the steppers
so --mt simulates both; use it to find out whether the "dropouts" come from the board/firmware
or from the transport layer (usbip / TCP bridge)

⚠️ --mt sends move commands "to the arm's own current position" (read with JP first, then the header
   changed JP->MT), so in principle the arm shouldn't move, but the motors really are driven — always clear the area around the arm
   before running
"""
import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not found — install it with:  pip install pyserial")


def read_line(ser, deadline):
    """Read byte by byte until \\n, the same way as the vendor's TeensyDriver::receive()"""
    buf = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return bytes(buf).decode("ascii", "replace").strip()
        if b != b"\r":
            buf += b
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=60.0, help="test duration in seconds")
    ap.add_argument("--hz", type=float, default=10.0, help="commands per second")
    ap.add_argument("--mt", action="store_true",
                    help="also send MT (move command), full ROS imitation — the motors will really be driven")
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud} ...")
    ser = serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2.0)
    print("Opened — starting test (Ctrl-C to stop)\n")

    mt_cmd = None
    if args.mt:
        # read the current position first, then change the "JP" header to "MT" — this gives a command in exactly
        # the format TeensyDriver::update() builds, targeting the position the arm is already at
        ser.write(b"JP\n")
        pos = read_line(ser, time.monotonic() + 2.0)
        if pos is None or not pos.startswith("JP"):
            sys.exit(f"Couldn't read the starting position (got: {pos!r})")
        mt_cmd = ("MT" + pos[2:] + "\n").encode("ascii")
        print(f"--mt mode: will send move commands to the current position ({len(mt_cmd)} bytes)")
        print(f"  {mt_cmd[:70].decode()}...\n")

    period = 1.0 / args.hz
    end = time.monotonic() + args.seconds
    latencies = []
    ok = bad = 0
    started = time.monotonic()

    try:
        while time.monotonic() < end:
            cycle_start = time.monotonic()

            # same order as controller_manager: read() first, then write()
            ser.write(b"JP\n")
            reply = read_line(ser, cycle_start + 1.0)
            if reply is not None and mt_cmd is not None:
                ser.write(mt_cmd)
                reply = read_line(ser, cycle_start + 2.0)

            elapsed = (time.monotonic() - cycle_start) * 1000.0

            if reply is None:
                bad += 1
                print(f"[{elapsed:7.1f} ms] no reply (timeout)")
            else:
                ok += 1
                latencies.append(elapsed)
                if ok <= 3 or ok % 50 == 0:
                    print(f"[{elapsed:7.1f} ms] {reply[:70]}")

            sleep = period - (time.monotonic() - cycle_start)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as exc:
        print(f"\n!!! crashed at second {time.monotonic() - started:.1f}")
        print(f"!!! {type(exc).__name__}: {exc}")
    finally:
        try:
            ser.close()
        except OSError:
            pass

    print("\n===== Summary =====")
    print(f"Ran for          {time.monotonic() - started:.1f} seconds")
    print(f"Replies OK       {ok} times")
    print(f"No reply/timeout {bad} times")
    if latencies:
        latencies.sort()
        print(f"Time per cycle   avg {statistics.mean(latencies):.1f} ms | "
              f"median {latencies[len(latencies) // 2]:.1f} ms | "
              f"worst {latencies[-1]:.1f} ms")


if __name__ == "__main__":
    main()
