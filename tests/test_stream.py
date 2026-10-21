"""Autonomous transport: no per-step wait, bounded framing, freshness and clock anchors."""
import unittest
from unittest.mock import Mock, patch

import numpy as np
from cubesat_sim.io import HitlController
from cubesat_sim import protocol as w


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.now=100.0
        self.clock=patch('cubesat_sim.io.time.monotonic',side_effect=lambda:self.now)
        self.clock.start(); self.addCleanup(self.clock.stop)
        self.port=Mock(in_waiting=0)
        self.port.write.side_effect=lambda data:len(data)
        factory=patch('cubesat_sim.io.serial.Serial',return_value=self.port)
        factory.start(); self.addCleanup(factory.stop)
        self.c=HitlController({'port':'COM12','baudrate':921600,'timeout_s':.15})
        self.addCleanup(self.c.close)
        self.rx=Mock(return_value=b'')
        self.c._read_available=self.rx
        self.c._read=Mock(side_effect=AssertionError('blocking read in autonomous mode'))
        self.c.step(0,0,.01)
        self.token=self.c.sync_token

    def frame(self,seq=1,local=10010,sync_local=10000,token=None,host=0,synced=1):
        return w.encode(w.U,[seq,100000,0,-100000,3000,local,1,10000,0,0,
                             local,self.token if token is None else token,host,sync_local,synced])

    def test_nonblocking_hold_and_occasional_sync(self):
        self.now+=.01; self.rx.return_value=self.frame()
        dipole,boom=self.c.step(1,.01,.01)
        np.testing.assert_allclose(dipole,[.1,0,-.1]); self.assertEqual(boom,3)
        self.assertAlmostEqual(self.c.estimate[0],.01)
        self.rx.return_value=b''
        self.now+=.01
        np.testing.assert_allclose(self.c.step(2,.02,.01)[0],dipole)
        self.assertEqual(self.port.write.call_count,1)  # No T for the second/third step.
        self.c._read.assert_not_called()

    def test_lost_sync_does_not_interrupt_stream(self):
        self.now+=.01; self.rx.return_value=self.frame()
        self.c.step(1,.01,.01)
        self.now+=1; self.rx.return_value=self.frame(seq=2,local=11010)
        self.c.step(101,1.01,.01)  # A new T is sent, firmware still echoes the old one.
        self.assertNotEqual(self.c.sync_token,self.token)
        self.assertEqual(self.c.last_stream_sequence,2)

    def test_lost_initial_sync_is_retried_before_timeout(self):
        self.now+=.06
        self.c.step(6,.06,.01)
        self.assertNotEqual(self.c.sync_token,self.token)
        self.assertEqual(self.port.write.call_count,2)
        self.rx.return_value=self.frame(token=self.c.sync_token,host=60,local=10000)
        self.c.step(6,.06,.01)
        self.assertEqual(self.c.last_stream_sequence,1)

    def test_partial_corrupt_and_oversized_frames_recover(self):
        packet=self.frame()
        self.now+=.01; self.rx.return_value=packet[:12]
        self.c.step(1,.01,.01); self.assertIsNone(self.c.last_stream_sequence)
        self.rx.return_value=packet[12:]
        self.c.step(1,.01,.01); self.assertEqual(self.c.last_stream_sequence,1)
        bad=bytearray(self.frame(seq=2)); bad[-2]^=1
        self.rx.return_value=b'x'*100+b'\0'+bytes(bad)+self.frame(seq=3)
        self.c.step(1,.01,.01)
        self.assertEqual(self.c.last_stream_sequence,3)
        self.assertEqual(self.c.rx_frame_errors,2)

    def test_stale_duplicate_unsynchronized_and_old_session_do_not_refresh_watchdog(self):
        self.now+=.01; self.rx.return_value=self.frame()
        self.c.step(1,.01,.01)
        self.now+=.2
        self.rx.return_value=(self.frame(seq=2)+self.frame(seq=3,local=10210,synced=0)
                              +self.frame(seq=4,local=10210,token=(self.token+10)&0xffffffff))
        with self.assertRaisesRegex(TimeoutError,'stream timed out'):
            self.c.step(21,.21,.01)
        self.assertEqual(self.c.last_stream_sequence,1)

    def test_recently_received_old_command_expires_by_source_age(self):
        self.now+=.14; self.rx.return_value=self.frame(local=10000)
        self.c.step(14,.14,.01)
        self.now+=.02; self.rx.return_value=b''
        with self.assertRaises(TimeoutError): self.c.step(16,.16,.01)

    def test_sequence_and_clock_wrap(self):
        self.now+=.01; self.rx.return_value=self.frame(seq=0xffffffff,local=4,sync_local=0xfffffffa)
        self.c.step(1,.01,.01)
        self.now+=.01; self.rx.return_value=self.frame(seq=0,local=14,sync_local=0xfffffffa)
        self.c.step(2,.02,.01)
        self.assertEqual(self.c.last_stream_sequence,0)
        self.assertAlmostEqual(self.c.estimate[0],.02)

    def test_latest_of_multiple_frames_wins_and_legacy_mode_mismatch_is_clear(self):
        self.now+=.01; self.rx.return_value=self.frame(seq=1)+self.frame(seq=2)+self.frame(seq=1)
        self.c.step(1,.01,.01); self.assertEqual(self.c.last_stream_sequence,2)
        self.rx.return_value=w.encode(w.A,[1,0,0,0,0,0,0,0,0,0])
        with self.assertRaisesRegex(ValueError,'clock mode'): self.c.step(2,.02,.01)

    def test_serial_reader_is_nonblocking(self):
        self.port.in_waiting=51
        self.port.read.return_value=b'payload'
        self.assertEqual(HitlController._read_available(self.c),b'payload')
        self.assertEqual(self.port.timeout,0)
        self.port.read.assert_called_once_with(51)

    def test_invalid_exchange_configuration(self):
        for config in ({'exchange':'bad'}, {'sync_period_s':0}, {'sync_period_s':float('nan')}):
            with self.assertRaises(ValueError):
                HitlController(dict(port='COM12',baudrate=921600,timeout_s=.15,**config))
