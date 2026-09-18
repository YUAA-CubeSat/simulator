from __future__ import annotations

import ctypes
import os
import select
import termios
import time
from pathlib import Path

import numpy as np


def validate_command(dipole, length):
    dipole = np.asarray(dipole, dtype=float)
    if dipole.shape != (3,) or not np.all(np.isfinite(dipole)):
        raise ValueError("dipole must be a finite 3-vector")
    length = float(length)
    if not np.isfinite(length):
        raise ValueError("boom target must be finite")
    return dipole, length


class SitlController:
    def __init__(self, config):
        library = Path(config["library"])
        if not library.is_file():
            raise FileNotFoundError(f"SITL library missing: {library}")
        self.lib = ctypes.CDLL(str(library))
        self.mag_fn = getattr(self.lib, config["magnetometer_symbol"])
        self.mag_fn.argtypes = [ctypes.c_double, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        self.mag_fn.restype = None
        self.sun_fn = getattr(self.lib, config["sun_symbol"])
        self.sun_fn.argtypes = [ctypes.c_double, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        self.sun_fn.restype = None
        self.step_fn = getattr(self.lib, config["step_symbol"])
        self.step_fn.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        self.step_fn.restype = ctypes.c_int

    def magnetometers(self, time_s, mag1, mag2):
        a = (ctypes.c_double * 3)(*mag1)
        b = (ctypes.c_double * 3)(*mag2)
        self.mag_fn(time_s, a, b)

    def sun(self, time_s, vector):
        values = vector if vector is not None else [0.0, 0.0, 0.0]
        a = (ctypes.c_double * 3)(*values)
        self.sun_fn(time_s, a, int(vector is not None))

    def step(self, tick, time_s, dt_s):
        out = (ctypes.c_double * 3)()
        boom = ctypes.c_double()
        status = self.step_fn(time_s, dt_s, out, ctypes.byref(boom))
        if status:
            raise RuntimeError(f"SITL controller returned status {status}")
        return validate_command(list(out), boom.value)

    def close(self):
        pass


class HitlController:
    def __init__(self, config):
        baud = int(config["baudrate"])
        attr = getattr(termios, f"B{baud}", None)
        if attr is None:
            raise ValueError(f"unsupported baudrate: {baud}")
        self.timeout = float(config["timeout_s"])
        if self.timeout <= 0:
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
        self.buffer = bytearray()

    def _send(self, line):
        data = (line + "\n").encode("ascii")
        deadline = time.monotonic() + self.timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError("UART write timed out")
            offset += os.write(self.fd, data[offset:])

    def magnetometers(self, time_s, mag1, mag2):
        values = [round(float(x) * 1e9) for x in (*mag1, *mag2)]
        self._send("M " + str(round(time_s * 1000)) + " " + " ".join(map(str, values)))

    def sun(self, time_s, vector):
        values = [round(float(x) * 32767) for x in vector] if vector is not None else [0, 0, 0]
        self._send("S " + str(round(time_s * 1000)) + " " + str(int(vector is not None)) + " " + " ".join(map(str, values)))

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
            if remaining <= 0 or not select.select([self.fd], [], [], remaining)[0]:
                raise TimeoutError("UART actuator command timed out")
            data = os.read(self.fd, 256)
            if not data:
                raise ConnectionError("UART closed")
            self.buffer.extend(data)
            if len(self.buffer) > 256:
                raise ValueError("UART actuator line exceeds 256 bytes")

    def close(self):
        os.close(self.fd)
