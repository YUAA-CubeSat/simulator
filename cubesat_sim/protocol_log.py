"""Human-readable diagnostics for received binary protocol frames."""
from . import protocol as wire


def format_message(kind, values):
    """Format decoded integer fields in physical units; IDs remain integers."""
    def vector(name, components, scale, unit):
        return f"{name}=[" + ', '.join(f'{x / scale:.6f}' for x in components) + f"] {unit}"

    if kind == wire.U:
        command = format_message(wire.A, values[:10]).replace('A tick=', 'U sequence=', 1)
        local, token, host, anchor, valid = values[10:]
        return (command + f" local_time={local / 1000:.3f} s sync_token={token} "
                f"sync_host={host / 1000:.3f} s sync_local={anchor / 1000:.3f} s sync_valid={valid}")
    if kind == wire.A:
        tick, mx, my, mz, boom, epoch, valid, wx, wy, wz = values
        return (f"A tick={tick} " + vector('m', (mx, my, mz), 1e6, 'A*m^2')
                + f" boom={boom / 1000:.3f} m E1 time={epoch / 1000:.3f} s valid={valid} "
                + vector('omega', (wx, wy, wz), 1e6, 'rad/s'))
    if kind == wire.T:
        tick, epoch, period = values
        return f"T tick={tick} time={epoch / 1000:.3f} s period={period / 1000:.3f} s"
    prefix = f"{'MCGSB'[kind]} time={values[0] / 1000:.3f} s"
    if kind == wire.M:
        return prefix + f" sensor={values[1]} " + vector('B', values[2:], 1000, 'uT')
    if kind == wire.C:
        return prefix + f" sensor={values[1]} " + vector('a', values[2:], 1e6, 'm/s^2')
    if kind == wire.G:
        return prefix + ' ' + vector('omega', values[1:], 1e6, 'rad/s')
    if kind == wire.S:
        return prefix + f" valid={values[1]} " + vector('sun', values[2:], 32767, '(unitless)')
    return prefix + f" soc={values[1] / 10:.1f}% power={values[2] / 1000:.3f} W"


class ProtocolRxLogger:
    """Reassemble raw RX chunks for logging without changing transport validation.

    Invalid frames get a bounded hex preview. Oversized input is discarded until
    its delimiter so a broken stream cannot grow the diagnostic buffer forever.
    """
    def __init__(self, emit):
        self.emit = emit
        self.buffer = bytearray()
        self.discard = False

    def __call__(self, data):
        for byte in data:
            if byte == 0:
                if self.buffer and not self.discard:
                    try:
                        kind, values = wire.decode(self.buffer)
                    except ValueError as exc:
                        self.emit(f"USB RX: INVALID ({exc}) hex={self.buffer.hex(' ')}")
                    else:
                        self.emit('USB RX: ' + format_message(kind, values))
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                self.buffer.append(byte)
                if len(self.buffer) > wire.MAX_FRAME:
                    self.emit(f"USB RX: INVALID (frame exceeds {wire.MAX_FRAME} bytes) "
                              f"hex={self.buffer[:wire.MAX_FRAME].hex(' ')} ...")
                    self.buffer.clear()
                    self.discard = True
