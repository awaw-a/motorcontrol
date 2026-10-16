"""200/0 rpm board test; preview by default, real serial I/O requires --run.

Close WTR_UpperMachine before opening the same COM port. This is a test aid,
not an independent emergency stop. The MCU's existing eight-byte protocol is
unchanged. Only --run needs pyserial; preview and unit tests are offline.
"""
import argparse
import csv
from datetime import datetime
from pathlib import Path
import struct
import sys
import time


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xFF
    return value


def speed_frame(current, rpm):
    body = b"\xAA\xBB\x00" + struct.pack("<hh", current, rpm)
    return body + bytes([crc8(body)])


class FeedbackParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b"\xBB\xAA":
                del self.buffer[0]
                continue
            if len(self.buffer) < 8:
                break
            frame = bytes(self.buffer[:8])
            if crc8(frame[:7]) != frame[7]:
                del self.buffer[0]
                continue
            del self.buffer[:8]
            status = frame[2]
            rpm, position = struct.unpack("<hh", frame[3:7])
            frames.append((status, rpm, position, frame))
        return frames


def target_at(elapsed, cycles):
    if not 0 <= elapsed < 2 * cycles:
        return 0
    return 200 if int(elapsed) % 2 == 0 else 0


def run_session(ser, current, cycles, log, clock=time.monotonic, sleep=time.sleep):
    """Injectable clock/serial keep offline fault tests independent of hardware."""
    parser = FeedbackParser()
    origin = clock()
    latest = None
    latest_time = None

    def send(limit, rpm, phase, lateness=0):
        frame = speed_frame(limit, rpm)
        if ser.write(frame) != len(frame):
            raise RuntimeError("Incomplete UART write")
        log.writerow(["TX", f"{clock()-origin:.6f}", phase, rpm, limit,
                      "", "", "", frame.hex(" "), f"{lateness:.6f}"])

    def receive(phase, target):
        nonlocal latest, latest_time
        data = ser.read(min(ser.in_waiting, 4096))
        for status, rpm, position, frame in parser.feed(data):
            latest, latest_time = (status, rpm, position), clock()
            log.writerow(["RX", f"{latest_time-origin:.6f}", phase, target,
                          current if phase == "wave" else 0,
                          f"{status:02X}", rpm, position, frame.hex(" "), ""])
            if phase == "wave" and status not in (0xAA, 0xAF):
                raise RuntimeError(f"Unexpected/fault status 0x{status:02X}; stop")

    try:
        # Require a fresh stop acknowledgement before sending any motion frame.
        start = clock()
        due = start
        while clock() - start < 1.0:
            now = clock()
            if now >= due:
                send(0, 0, "prepare")
                due = now + 0.1
            receive("prepare", 0)
            sleep(0.002)
        if (latest is None or clock() - latest_time > 0.3 or
                latest[0] != 0xAF or abs(latest[1]) > 15):
            raise RuntimeError("No fresh AF stop acknowledgement at rest; check wiring/faults")

        start = clock()
        due = start
        last_target = None
        while clock() - start < 2 * cycles:
            now = clock()
            target = target_at(now - start, cycles)
            receive("wave", target)
            if latest_time is None or clock() - latest_time > 0.3:
                raise RuntimeError("No valid feedback for 300 ms; stop")
            if now >= due:
                lateness = now - due
                if lateness > 0.15:
                    raise RuntimeError("PC sender delayed over 150 ms; timing test invalid")
                send(current, target, "wave", lateness)
                if target != last_target:
                    print(f"{now-start:.3f} s: target {target} rpm", flush=True)
                    last_target = target
                due += 0.1
            sleep(0.002)
    finally:
        # Best effort only: a broken wire/driver can prevent every write below.
        for _ in range(4):
            try:
                send(0, 0, "stop")
                sleep(0.1)
            except Exception as exc:
                print(f"Could not deliver stop frame: {exc}; use physical cutoff", file=sys.stderr)
                break


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", choices=("C620", "C610"), default="C620")
    parser.add_argument("--current", type=int, default=500, help="raw current limit, not amperes")
    parser.add_argument("--cycles", type=int, default=3, help="1..10 cycles, each cycle lasts 2 s")
    parser.add_argument("--port", help="actual Windows COM port, e.g. COM3")
    parser.add_argument("--log", type=Path, help="new CSV path; existing files are never overwritten")
    parser.add_argument("--run", action="store_true", help="open serial port and drive the motor")
    args = parser.parse_args(argv)
    maximum = 8000 if args.motor == "C620" else 4000
    if not 0 < args.current <= maximum:
        parser.error(f"--current must be 1..{maximum} for {args.motor}")
    if not 1 <= args.cycles <= 10:
        parser.error("--cycles must be 1..10")
    if not args.run:
        print("PREVIEW ONLY: no serial port is opened; no motor commands are sent.")
        print(f"Motor selection {args.motor} checks the limit; it cannot identify MCU firmware.")
        print("Prepare/final stop:", speed_frame(0, 0).hex(" ").upper())
        for rpm in (200, 0):
            print(f"{rpm:3d} rpm, A={args.current}:", speed_frame(args.current, rpm).hex(" ").upper())
        print(f"1 s preparation; {args.cycles} x (200 rpm 1 s, 0 rpm 1 s); zero-current stop.")
        return 0
    if not args.port:
        parser.error("--run requires --port; close WTR_UpperMachine first")
    try:
        import serial
    except ImportError:
        parser.error("Install pyserial with: python -m pip install pyserial")
    log_path = args.log or Path(f"board-wave-{datetime.now():%Y%m%d-%H%M%S-%f}.csv")
    ser = None
    try:
        # Open the log first, so a path error cannot leave an unrecorded motion run.
        with log_path.open("x", newline="", encoding="utf-8-sig") as stream:
            writer = csv.writer(stream)
            writer.writerow(["event", "pc_elapsed_s", "phase", "target_rpm", "current_limit_raw",
                             "status_hex", "actual_rpm", "position_deg", "frame_hex", "tx_late_s"])
            ser = serial.Serial(port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                                timeout=0, write_timeout=0.05,
                                xonxoff=False, rtscts=False, dsrdtr=False)
            ser.dtr = False
            ser.rts = False
            ser.port = args.port
            ser.open()
            ser.reset_input_buffer()
            print(f"RUN: {args.port}, {args.motor}, limit={args.current}, cycles={args.cycles}")
            run_session(ser, args.current, args.cycles, writer)
        print(f"Sequence finished; inspect response and timing in {log_path.resolve()}")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"Test aborted: {exc}; inspect the bench before retrying", file=sys.stderr)
        return 1
    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
