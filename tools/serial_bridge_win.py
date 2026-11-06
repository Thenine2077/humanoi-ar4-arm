#!/usr/bin/env python3
"""Windows-side serial <-> TCP bridge to get around usbip

Background
----------
usbip works but is slow and fragile: ar_hardware_interface does a blocking serial round-trip
twice per control loop cycle (read -> "JP\\n" and wait for the reply, write -> "MT...\\n" and wait for the reply)
over usbip that measured 25 ms + 43 ms per cycle with the arm not even moving, and it dropped within ~1 minute
compared with 1.9 ms per cycle when opening the COM port directly on Windows (see test_com_direct.py --mt)

This bridge avoids usbip entirely: Windows opens the COM port directly (no USB forwarding)
and streams the raw bytes over TCP to WSL; on the WSL side socat creates a fake pty
so ROS sees a plain /dev/ttyV0 — not a single line of the vendor code needs changing

⚠️ Single-threaded on purpose — do not split it into a reader thread and a writer thread
   pyserial is not thread-safe: the first version of this file used 2 threads (serial->tcp and
   tcp->serial); once ROS started sending real traffic, one thread was blocked in ser.read() while the other
   called ser.write() at the same time, and it crashed with
       ClearCommError failed (PermissionError(13, 'The device does not recognize
       the command.', winerror 22 = ERROR_BAD_COMMAND))
   followed by the COM port vanishing from Windows entirely (FileNotFoundError on reopen),
   which "looks exactly like the device was unplugged", so for a long time we wrongly suspected the board
   Proven otherwise: test_com_direct.py --mt --hz 30 (single thread, direct COM)
   ran 3572 cycles × 2 without a single miss, using exactly the same traffic

Usage
-------
  Windows side (plain PowerShell, no admin needed):
      pip install pyserial
      python C:\\Users\\User\\Desktop\\ar4\\tools\\serial_bridge_win.py --port COM9

  WSL side (another terminal window):
      bash /mnt/c/Users/User/Desktop/ar4/tools/serial_bridge_wsl.sh
      # -> creates /dev/ttyV0; then launch ROS with serial_port:=/dev/ttyV0

Note: if WSL can't connect, it's usually the Windows Firewall — open the port with (PowerShell as admin):
  New-NetFirewallRule -DisplayName "AR4 serial bridge" -Direction Inbound `
      -LocalPort 5555 -Protocol TCP -Action Allow
"""
import argparse
import select
import socket
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not found — install it with:  pip install pyserial")


def serve_client(conn, addr, ser, idle_s):
    """Pass bytes both ways on a single thread. Returns True if the serial port is still usable

    Polls the socket with select timeout=0 (pure poll, no waiting), then checks ser.in_waiting,
    so serial reads and writes always happen on this one thread and can never overlap

    Why timeout=0 + time.sleep instead of letting select do the waiting:
    Windows timer granularity is ~15.6 ms, so asking select to wait 1 ms gets rounded up to
    over ten ms every time; measured 23.6 ms per cycle (vs 1.9 ms with the COM port opened directly)
    whereas time.sleep() in Python 3.11+ on Windows uses a high-resolution waitable timer
    so it really can sleep with sub-millisecond resolution
    and a cycle that "had work to do" doesn't sleep at all, so the rest of a message's bytes are picked up immediately
    """
    print(f"[bridge] WSL connected from {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # drop data left in the pipe from the previous session, or the driver will parse fragments of old messages
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    serial_ok = True
    try:
        closed = False
        while not closed:
            busy = False

            # ---- TCP -> serial ----
            readable, _, _ = select.select([conn], [], [], 0)
            if readable:
                data = conn.recv(4096)
                if not data:
                    print(f"[bridge] {addr} closed the connection")
                    closed = True
                else:
                    ser.write(data)
                    busy = True

            # ---- serial -> TCP ----
            waiting = ser.in_waiting
            if waiting:
                conn.sendall(ser.read(waiting))
                busy = True

            if not busy and not closed:
                time.sleep(idle_s)
    except OSError as exc:
        # tell which side failed: if the COM port is still alive, it was the TCP side (the client just dropped)
        serial_ok = ser.is_open and _serial_alive(ser)
        who = "serial" if not serial_ok else "TCP"
        print(f"[bridge] {addr} session ended ({who}): {exc}")
    finally:
        try:
            conn.close()
        except OSError:
            pass

    print("[bridge] waiting for a new client")
    return serial_ok


def _serial_alive(ser):
    """Check whether the COM port still answers IOCTLs (in_waiting calls ClearCommError internally)"""
    try:
        ser.in_waiting
        return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM9", help="Teensy COM port (see Device Manager)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--listen", default="0.0.0.0")
    ap.add_argument("--tcp-port", type=int, default=5555)
    ap.add_argument("--idle-us", type=float, default=200.0,
                    help="microseconds to sleep when there is no data (lower = less latency, more CPU)")
    args = ap.parse_args()

    def open_serial():
        """Open the COM port, retrying until it works (in case the Teensy is unplugged/rebooted mid-way)"""
        while True:
            try:
                # timeout=0 -> read() never blocks; we only read the byte count in_waiting already reported
                s = serial.Serial(args.port, args.baud, timeout=0, write_timeout=2.0)
                print(f"[bridge] opened {args.port} @ {args.baud}")
                return s
            except OSError as exc:
                print(f"[bridge] can't open {args.port} ({exc}) — retrying in 2 s")
                time.sleep(2.0)

    ser = open_serial()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.listen, args.tcp_port))
    srv.listen(1)
    print(f"[bridge] listening on {args.listen}:{args.tcp_port} — waiting for WSL to connect (Ctrl-C to quit)")

    idle_s = args.idle_us / 1_000_000.0
    try:
        while True:
            conn, addr = srv.accept()
            if not serve_client(conn, addr, ser, idle_s):
                print("[bridge] serial problem — closing and reopening the COM port")
                try:
                    ser.close()
                except OSError:
                    pass
                ser = open_serial()
    except KeyboardInterrupt:
        print("\n[bridge] stopped by user")
    finally:
        srv.close()
        try:
            ser.close()
        except OSError:
            pass


if __name__ == "__main__":
    main()
