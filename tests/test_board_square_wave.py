"""Offline protocol, waveform timing and abort-path checks; never opens a COM port."""
import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location(
    "board_square_wave", Path(__file__).resolve().parents[1] / "tools/board_square_wave.py")
wave = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wave)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def clock(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakeSerial:
    def __init__(self, clock, status=0xAF, fail_after=None, corrupt=False,
                 fault_after=None, silence_after=None):
        self.clock = clock
        self.status = status
        self.fail_after = fail_after
        self.corrupt = corrupt
        self.fault_after = fault_after
        self.silence_after = silence_after
        self.pending = bytearray()
        self.sent = []

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, data):
        self.sent.append((self.clock.now, bytes(data)))
        if self.fail_after is not None and self.clock.now >= self.fail_after:
            raise OSError("injected disconnect")
        status = 0xFF if self.fault_after is not None and self.clock.now >= self.fault_after else self.status
        body = b"\xBB\xAA" + bytes([status]) + struct.pack("<hh", 0, -720)
        if self.silence_after is None or self.clock.now < self.silence_after:
            self.pending.extend(body + bytes([wave.crc8(body) ^ int(self.corrupt)]))
        return len(data)

    def read(self, count):
        data = bytes(self.pending[:count])
        del self.pending[:count]
        return data


class BoardWaveTests(unittest.TestCase):
    def test_crc_and_emergency_frame(self):
        self.assertEqual(wave.crc8(b"123456789"), 0xF4)
        frame = wave.speed_frame(0, 0)
        self.assertEqual(frame[:7], bytes.fromhex("AA BB 00 00 00 00 00"))
        self.assertEqual(len(frame), 8)
        self.assertEqual(wave.speed_frame(500, -200)[3:7], struct.pack("<hh", 500, -200))

    def test_fragmented_corrupt_and_joined_feedback(self):
        body = bytes.fromhex("BB AA BA") + struct.pack("<hh", -200, -720)
        frame = body + bytes([wave.crc8(body)])
        parser = wave.FeedbackParser()
        self.assertEqual(parser.feed(b"\x01\x02\xBB"), [])
        self.assertEqual(parser.feed(frame[1:4]), [])
        self.assertEqual(parser.feed(frame[4:])[0][:3], (0xBA, -200, -720))
        bad = frame[:-1] + bytes([frame[-1] ^ 1])
        self.assertEqual(len(parser.feed(bad + frame + frame)), 2)

    def test_wave_boundaries(self):
        for elapsed, expected in [(-1, 0), (0, 200), (.999, 200), (1, 0),
                                  (1.999, 0), (2, 200), (5.999, 0), (6, 0)]:
            with self.subTest(elapsed=elapsed):
                self.assertEqual(wave.target_at(elapsed, 3), expected)

    def run_fake(self, ser, clock):
        log = io.StringIO()
        import csv
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            wave.run_session(ser, 500, 1, csv.writer(log), clock.clock, clock.sleep)
        return log.getvalue()

    def test_zero_ack_wave_and_final_stop(self):
        clock = FakeClock()
        ser = FakeSerial(clock)
        self.run_fake(ser, clock)
        moving = [(t, frame) for t, frame in ser.sent if struct.unpack("<h", frame[3:5])[0]]
        self.assertTrue(moving)
        self.assertTrue(all(t >= 1.0 for t, _ in moving))
        origin = moving[0][0]
        for timestamp, frame in moving:
            self.assertEqual(struct.unpack("<h", frame[5:7])[0], wave.target_at(timestamp-origin, 1))
            self.assertEqual(wave.crc8(frame[:7]), frame[7])
        self.assertTrue(all(frame == wave.speed_frame(0, 0) for _, frame in ser.sent[-4:]))

    def test_no_movement_without_valid_stop_ack(self):
        for status, corrupt in [(0xFF, False), (0xAF, True)]:
            clock = FakeClock()
            ser = FakeSerial(clock, status=status, corrupt=corrupt)
            with self.assertRaisesRegex(RuntimeError, "acknowledgement"):
                self.run_fake(ser, clock)
            self.assertTrue(all(frame == wave.speed_frame(0, 0) for _, frame in ser.sent))

    def test_disconnect_still_attempts_stop(self):
        clock = FakeClock()
        ser = FakeSerial(clock, fail_after=1.3)
        with self.assertRaises(OSError):
            self.run_fake(ser, clock)
        self.assertEqual(ser.sent[-1][1], wave.speed_frame(0, 0))

    def test_fault_during_motion_aborts(self):
        clock = FakeClock()
        ser = FakeSerial(clock, fault_after=1.3)
        with self.assertRaisesRegex(RuntimeError, "status 0xFF"):
            self.run_fake(ser, clock)
        self.assertTrue(all(frame == wave.speed_frame(0, 0) for _, frame in ser.sent[-4:]))

    def test_missing_feedback_during_motion_aborts(self):
        clock = FakeClock()
        ser = FakeSerial(clock, silence_after=1.2)
        with self.assertRaisesRegex(RuntimeError, "300 ms"):
            self.run_fake(ser, clock)
        self.assertTrue(all(frame == wave.speed_frame(0, 0) for _, frame in ser.sent[-4:]))

    def test_delayed_pc_sender_aborts(self):
        clock = FakeClock()
        normal_sleep = clock.sleep
        stalled = False

        def sleep(duration):
            nonlocal stalled
            normal_sleep(duration)
            if not stalled and clock.now >= 1.25:
                stalled = True
                clock.now += 0.23

        clock.sleep = sleep
        ser = FakeSerial(clock)
        with self.assertRaisesRegex(RuntimeError, "150 ms"):
            self.run_fake(ser, clock)
        self.assertTrue(all(frame == wave.speed_frame(0, 0) for _, frame in ser.sent[-4:]))

    def test_preview_opens_no_serial(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(wave.main(["--port", "THIS_PORT_MUST_NOT_OPEN"]), 0)
        self.assertIn("PREVIEW ONLY", output.getvalue())

    def test_reject_invalid_limits_cycles_and_run_without_port(self):
        for args in (["--motor", "C610", "--current", "4001"],
                     ["--current", "0"], ["--cycles", "0"], ["--run"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                wave.main(args)
            self.assertEqual(caught.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
