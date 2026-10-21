import unittest

from cubesat_sim import protocol as wire
from cubesat_sim.protocol_log import ProtocolRxLogger


class ProtocolLogTests(unittest.TestCase):
    def test_units_for_every_message_type(self):
        cases = [
            (wire.M, [1000, 1, 10000, -20000, 1],
             'M time=1.000 s sensor=1 B=[10.000000, -20.000000, 0.001000] uT'),
            (wire.C, [1000, 0, 1000, -500, 1],
             'C time=1.000 s sensor=0 a=[0.001000, -0.000500, 0.000001] m/s^2'),
            (wire.G, [1000, 10000, -20000, 1],
             'G time=1.000 s omega=[0.010000, -0.020000, 0.000001] rad/s'),
            (wire.S, [1000, 1, 32767, 0, -32767],
             'S time=1.000 s valid=1 sun=[1.000000, 0.000000, -1.000000] (unitless)'),
            (wire.B, [1000, 800, -750], 'B time=1.000 s soc=80.0% power=-0.750 W'),
            (wire.T, [100, 1000, 10], 'T tick=100 time=1.000 s period=0.010 s'),
            (wire.A, [100, 100000, 0, -100000, 500, 1000, 1, 10000, -20000, 15000],
             'A tick=100 m=[0.100000, 0.000000, -0.100000] A*m^2 boom=0.500 m '
             'E1 time=1.000 s valid=1 omega=[0.010000, -0.020000, 0.015000] rad/s'),
        ]
        for kind, values, expected in cases:
            with self.subTest(kind=kind):
                lines = []
                ProtocolRxLogger(lines.append)(wire.encode(kind, values))
                self.assertEqual(lines, ['USB RX: ' + expected])

    def test_autonomous_sync_metadata(self):
        lines = []
        ProtocolRxLogger(lines.append)(wire.encode(wire.U,
            [5, 0, 0, 0, 0, 1010, 0, 0, 0, 0, 1010, 7, 20, 1000, 1]))
        self.assertIn('U sequence=5', lines[0])
        self.assertIn('local_time=1.010 s sync_token=7 sync_host=0.020 s '
                      'sync_local=1.000 s sync_valid=1', lines[0])

    def test_fragmented_and_batched_frames(self):
        lines = []
        log = ProtocolRxLogger(lines.append)
        frame = wire.encode(wire.A, [0xffffffff, 0, 0, 0, 0, 0xffffffff, 0, 0, 0, 0])
        for byte in frame[:-1]:
            log(bytes([byte]))
        self.assertEqual(lines, [])
        log(b'\0\0' + frame + frame[:3])
        self.assertEqual(len(lines), 2)
        self.assertIn('tick=4294967295', lines[0])
        self.assertIn('time=4294967.295 s valid=0', lines[0])
        log(frame[3:])
        self.assertEqual(len(lines), 3)

    def test_corrupt_and_oversized_frames_recover(self):
        lines = []
        log = ProtocolRxLogger(lines.append)
        valid = wire.encode(wire.T, [0, 0, 10])
        corrupt = bytearray(valid)
        corrupt[-2] ^= 1
        log(corrupt)
        self.assertIn('INVALID', lines[0])
        self.assertIn('hex=', lines[0])
        log(b'x' * 10000)
        self.assertEqual(len(lines), 2)
        self.assertIn('exceeds 64 bytes', lines[1])
        self.assertEqual(len(log.buffer), 0)
        log(b'\0' + valid)
        self.assertEqual(lines[-1], 'USB RX: T tick=0 time=0.000 s period=0.010 s')


if __name__ == '__main__':
    unittest.main()
