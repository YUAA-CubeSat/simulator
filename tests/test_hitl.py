"""Portable serial transport tests; no UART hardware or Unix PTYs required."""
import unittest
from unittest.mock import Mock, patch

import numpy as np
import serial

from cubesat_sim.io import HitlController


class HitlTests(unittest.TestCase):
    def setUp(self):
        self.port = Mock(in_waiting=0)
        self.port.write.side_effect = lambda data: len(data)
        factory = patch('cubesat_sim.io.serial.Serial', return_value=self.port)
        self.factory = factory.start()
        self.addCleanup(factory.stop)
        # HITL must never reach the Unix-only subprocess pipe implementation.
        selector = patch('cubesat_sim.io.select.select', side_effect=AssertionError('Unix I/O'))
        selector.start()
        self.addCleanup(selector.stop)
        self.config = {'port': 'COM12', 'baudrate': 115200, 'timeout_s': 0.15}

    def controller(self, **kwargs):
        controller = HitlController(self.config, **kwargs)
        self.addCleanup(controller.close)
        return controller

    def test_partial_reply_estimate_and_legacy(self):
        received = []
        c = self.controller(on_receive=received.append)
        self.port.read.side_effect = [b'A 7 100000 ', b'0 -100000 400 E1 1000 1 10000 0 -10000\r\n',
                                      b'A 8 0 0 0 0\n']
        dipole, length = c.step(7, 1, .01)
        np.testing.assert_allclose(dipole, [.1, 0, -.1])
        self.assertEqual(length, .4)
        np.testing.assert_allclose(c.estimate, [1, 1, .01, 0, -.01])
        self.assertEqual(len(received), 2)
        self.port.write.assert_called_with(b'T 7 1000 10\n')
        c.step(8, 1.01, .01)
        self.assertIsNone(c.estimate)
        self.factory.assert_called_once_with(port='COM12', baudrate=115200,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=.15, write_timeout=.15, xonxoff=False, rtscts=False, dsrdtr=False)
        self.port.reset_input_buffer.assert_called_once()

    def test_partial_writes(self):
        c = self.controller()
        self.port.write.side_effect = [2, 8]
        c._send('123456789')
        self.assertEqual(self.port.write.call_args_list[1].args[0], b'3456789\n')

    def test_read_timeout_and_disconnect(self):
        c = self.controller()
        self.port.read.return_value = b''
        with self.assertRaisesRegex(TimeoutError, 'actuator command'):
            c.step(0, 0, 1)
        self.port.read.side_effect = serial.SerialException('disconnected')
        with self.assertRaisesRegex(ConnectionError, 'disconnected'):
            c.step(0, 0, 1)

    def test_write_timeout_disconnect_and_zero_progress(self):
        c = self.controller()
        for failure, expected in [(serial.SerialTimeoutException('slow'), TimeoutError),
                                  (serial.SerialException('gone'), ConnectionError)]:
            self.port.write.side_effect = failure
            with self.assertRaises(expected):
                c._send('test')
        self.port.write.side_effect = None
        self.port.write.return_value = 0
        with self.assertRaises(ConnectionError):
            c._send('test')

    def test_partial_replies_share_one_deadline(self):
        c = self.controller()
        self.port.read.return_value = b'A '
        with patch('cubesat_sim.io.time.monotonic', side_effect=[0, .01, 1, 1.05, 1.16]):
            with self.assertRaises(TimeoutError):
                c.step(0, 0, 1)
        self.port.read.assert_called_once()

    def test_invalid_and_oversized_replies(self):
        for reply in [b'A 8 0 0 0 0\n', b'garbage\n', b'x' * 256]:
            c = self.controller()
            self.port.read.return_value = reply
            with self.assertRaises(ValueError):
                c.step(7, 0, 1)

    def test_cleanup_on_initialization_failure(self):
        self.port.reset_input_buffer.side_effect = serial.SerialException('failed')
        with self.assertRaises(serial.SerialException):
            HitlController(self.config)
        self.port.close.assert_called_once()

    def test_invalid_settings_do_not_open_port(self):
        for timeout in [0, -1, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                HitlController(dict(self.config, timeout_s=timeout))
        with self.assertRaises(ValueError):
            HitlController(dict(self.config, baudrate=0))
        self.factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
