from __future__ import annotations

import math
import subprocess
import os
import select
import time

import numpy as np
import serial

from . import protocol as wire


def validate_command(dipole, length):
    dipole = np.asarray(dipole, dtype=float)
    if dipole.shape != (3,) or not np.all(np.isfinite(dipole)):
        raise ValueError("dipole must be a finite 3-vector")
    length = float(length)
    if not np.isfinite(length):
        raise ValueError("boom target must be finite")
    return dipole, length


class _BinaryController:
    def _send(self, data):
        deadline = time.monotonic() + self.timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"{self.transport} write timed out")
            written = self._write(data[offset:], remaining)
            if not written:
                raise ConnectionError(f"{self.transport} closed during write")
            offset += written

    def _sensor(self, kind, values):
        # Batch the current cycle into one host write, reducing USB transactions.
        values[0] &= 0xffffffff  # Millisecond timestamps wrap consistently with firmware.
        frame = wire.encode(kind, values)
        if len(self.pending) + len(frame) > 4096:
            raise ValueError("sensor batch exceeds 4096 bytes; issue a control step")
        self.pending.extend(frame)

    def magnetometers(self, time_s, *readings):
        for sensor_id, reading in enumerate(readings):
            self._sensor(wire.M, [wire.quantize(time_s, 1000), sensor_id,
                                 *(wire.quantize(x, 1e9) for x in reading)])

    def accelerometers(self, time_s, *readings):
        for sensor_id, reading in enumerate(readings):
            self._sensor(wire.C, [wire.quantize(time_s, 1000), sensor_id,
                                 *(wire.quantize(x, 1e6) for x in reading)])

    def gyroscope(self, time_s, reading):
        self._sensor(wire.G, [wire.quantize(time_s, 1000), *(wire.quantize(x, 1e6) for x in reading)])

    def sun(self, time_s, vector):
        values = [wire.quantize(x, 32767) for x in vector] if vector is not None else [0, 0, 0]
        self._sensor(wire.S, [wire.quantize(time_s, 1000), int(vector is not None), *values])

    def battery(self, time_s, soc, power_W):
        if not 0 <= soc <= 1:
            raise ValueError("battery SOC must be in [0, 1]")
        self._sensor(wire.B, [wire.quantize(time_s, 1000), wire.quantize(soc, 1000), wire.quantize(power_W, 1000)])

    def step(self, tick, time_s, dt_s):
        self.estimate = None
        period = wire.quantize(dt_s, 1000)
        if period <= 0:
            raise ValueError("control period must round to at least 1 ms")
        request = wire.encode(wire.T, [tick & 0xffffffff, wire.quantize(time_s, 1000) & 0xffffffff, period])
        self._send(bytes(self.pending) + request)
        self.pending.clear()
        deadline = time.monotonic() + self.timeout
        while True:
            if 0 in self.buffer:
                packet, _, rest = self.buffer.partition(b"\0")
                self.buffer = bytearray(rest)
                if not packet:
                    continue
                kind, values = wire.decode(packet)
                if kind != wire.A or values[0] != (tick & 0xffffffff):
                    raise ValueError("invalid actuator message or tick mismatch")
                _, mx, my, mz, boom, epoch, valid, wx, wy, wz = values
                if valid not in (0, 1) or (not valid and any((wx, wy, wz))):
                    raise ValueError("invalid estimate telemetry")
                self.estimate = (epoch / 1000, valid, wx / 1e6, wy / 1e6, wz / 1e6)
                return validate_command([mx / 1e6, my / 1e6, mz / 1e6], boom / 1000)
            if len(self.buffer) > wire.MAX_FRAME:
                raise ValueError("oversized binary actuator frame")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise self._actuator_timeout(tick)
            try:
                data = self._read(remaining)
            except TimeoutError as exc:
                raise self._actuator_timeout(tick) from exc
            if not data:
                raise ConnectionError(f"{self.transport} closed")
            if self.on_receive is not None:
                self.on_receive(data)
            self.buffer.extend(data)

    def _actuator_timeout(self, tick):
        port = getattr(self, "port_name", None)
        endpoint = f", port={port}" if port is not None else ""
        return TimeoutError(
            f"{self.transport} actuator command timed out "
            f"(tick={tick}{endpoint}, partial_reply_bytes={len(self.buffer)}, "
            f"timeout_s={self.timeout:g})")


class SitlController(_BinaryController):
    transport = "SITL"

    def __init__(self, config, on_receive=None):
        self.timeout = float(config["timeout_s"])
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("SITL timeout_s must be positive and finite")
        startup_delay = float(config.get("startup_delay_s", 0))
        if not math.isfinite(startup_delay) or startup_delay < 0:
            raise ValueError("SITL startup_delay_s must be nonnegative and finite")
        command = config.get("command")
        if not isinstance(command, list) or not command or any(not isinstance(arg, str) or not arg for arg in command):
            raise ValueError("sitl.command must be a nonempty list of strings")
        self.on_receive = on_receive
        self.buffer = bytearray()
        self.pending = bytearray()
        self.process = subprocess.Popen(command, cwd=config.get("cwd"), stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, bufsize=0)
        try:
            self.write_fd = self.process.stdin.fileno()
            self.read_fd = self.process.stdout.fileno()
            os.set_blocking(self.write_fd, False)
            os.set_blocking(self.read_fd, False)
            if startup_delay:
                try:
                    status = self.process.wait(timeout=startup_delay)
                except subprocess.TimeoutExpired:
                    pass  # Boot grace period elapsed; begin protocol traffic.
                else:
                    raise ConnectionError(f"SITL process exited during startup with status {status}")
        except BaseException:
            self.close()
            raise

    def _write(self, data, remaining):
        if not select.select([], [self.write_fd], [], remaining)[1]:
            raise TimeoutError("SITL write timed out")
        return os.write(self.write_fd, data)

    def _read(self, remaining):
        if not select.select([self.read_fd], [], [], remaining)[0]:
            raise TimeoutError("SITL actuator command timed out")
        return os.read(self.read_fd, 256)

    def close(self):
        # EOF lets cooperative controllers exit; terminate/kill bound cleanup time.
        self.process.stdin.close()
        try:
            self.process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        finally:
            self.process.stdout.close()


class HitlController(_BinaryController):
    transport = "UART"

    def __init__(self, config, on_receive=None):
        self.on_receive = on_receive
        baud = int(config["baudrate"])
        if baud <= 0:
            raise ValueError("HITL baudrate must be positive")
        self.timeout = float(config["timeout_s"])
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("HITL timeout_s must be positive and finite")
        startup_delay = float(config.get("startup_delay_s", 0))
        if not math.isfinite(startup_delay) or startup_delay < 0:
            raise ValueError("HITL startup_delay_s must be nonnegative and finite")
        self.port_name = config["port"]
        self.serial = serial.Serial(
            port=self.port_name, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=self.timeout,
            write_timeout=self.timeout, xonxoff=False, rtscts=False, dsrdtr=False)
        try:
            # Opening a serial adapter may reset the board; UART initialization
            # runs later in the OBC services task. Do not send T during boot.
            if startup_delay:
                time.sleep(startup_delay)
            self.serial.reset_input_buffer()
        except BaseException:
            self.serial.close()
            raise
        self.buffer = bytearray()
        self.pending = bytearray()

    def _write(self, data, remaining):
        try:
            self.serial.write_timeout = remaining
            return self.serial.write(data)
        except serial.SerialTimeoutException as exc:
            raise TimeoutError("UART write timed out") from exc
        except serial.SerialException as exc:
            raise ConnectionError(f"UART write failed: {exc}") from exc

    def _read(self, remaining):
        try:
            self.serial.timeout = remaining
            # Wait for one byte when idle; drain available bytes without waiting
            # to fill a 256-byte buffer on every short protocol response.
            data = self.serial.read(max(1, min(256, self.serial.in_waiting)))
        except serial.SerialException as exc:
            raise ConnectionError(f"UART read failed: {exc}") from exc
        if not data:
            raise TimeoutError("UART actuator command timed out")
        return data

    def close(self):
        self.serial.close()
