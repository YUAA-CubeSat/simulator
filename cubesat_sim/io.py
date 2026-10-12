from __future__ import annotations

import math
import subprocess
import os
import select
import termios
import time

import numpy as np


def validate_command(dipole, length):
    dipole = np.asarray(dipole, dtype=float)
    if dipole.shape != (3,) or not np.all(np.isfinite(dipole)):
        raise ValueError("dipole must be a finite 3-vector")
    length = float(length)
    if not np.isfinite(length):
        raise ValueError("boom target must be finite")
    return dipole, length


class _LineController:
    def _send(self, line):
        data = (line + "\n").encode("ascii")
        deadline = time.monotonic() + self.timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.write_fd], [], remaining)[1]:
                raise TimeoutError(f"{self.transport} write timed out")
            offset += os.write(self.write_fd, data[offset:])

    def magnetometers(self, time_s, *readings):
        time_ms = round(time_s * 1000)
        for sensor_id, reading in enumerate(readings):
            values = [round(float(x) * 1e9) for x in reading]
            self._send(f"M {time_ms} {sensor_id} " + " ".join(map(str, values)))

    def accelerometers(self, time_s, *readings):
        time_ms = round(time_s * 1000)
        for sensor_id, reading in enumerate(readings):
            values = [round(float(x) * 1e6) for x in reading]
            self._send(f"C {time_ms} {sensor_id} " + " ".join(map(str, values)))

    def sun(self, time_s, vector):
        values = [round(float(x) * 32767) for x in vector] if vector is not None else [0, 0, 0]
        self._send("S " + str(round(time_s * 1000)) + " " + str(int(vector is not None)) + " " + " ".join(map(str, values)))

    def battery(self, time_s, soc, power_W):
        """Send state of charge in permille and signed battery power in mW."""
        self._send(f"B {round(time_s * 1000)} {round(soc * 1000)} {round(power_W * 1000)}")

    def step(self, tick, time_s, dt_s):
        self._send(f"T {tick} {round(time_s * 1000)} {round(dt_s * 1000)}")
        deadline = time.monotonic() + self.timeout
        while True:
            if b"\n" in self.buffer:
                line, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                fields = line.decode("ascii").split()
                if len(fields) != 6 or fields[0] != "A" or int(fields[1]) != tick:
                    raise ValueError("invalid actuator line or tick mismatch")
                return validate_command([int(x) / 1e6 for x in fields[2:5]], int(fields[5]) / 1000)
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.read_fd], [], [], remaining)[0]:
                raise TimeoutError(f"{self.transport} actuator command timed out")
            data = os.read(self.read_fd, 256)
            if not data:
                raise ConnectionError(f"{self.transport} closed")
            if self.on_receive is not None:
                self.on_receive(data)
            self.buffer.extend(data)
            if len(self.buffer) > 256:
                raise ValueError(f"{self.transport} actuator line exceeds 256 bytes")


class SitlController(_LineController):
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


class HitlController(_LineController):
    transport = "UART"

    def __init__(self, config, on_receive=None):
        self.on_receive = on_receive
        baud = int(config["baudrate"])
        attr = getattr(termios, f"B{baud}", None)
        if attr is None:
            raise ValueError(f"unsupported baudrate: {baud}")
        self.timeout = float(config["timeout_s"])
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("HITL timeout_s must be positive")
        self.fd = os.open(config["port"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        settings = termios.tcgetattr(self.fd)
        settings[0] = 0
        settings[1] = 0
        settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[3] = 0
        settings[4] = attr
        settings[5] = attr
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.read_fd = self.write_fd = self.fd
        self.buffer = bytearray()

    def close(self):
        os.close(self.fd)
