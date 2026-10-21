"""Text-shaped fixtures converted to binary wire bytes; never a live protocol."""
from cubesat_sim import protocol as wire


def frame(line):
    parts = line.split()
    kind = "MCGSBTAU".index(parts[0])
    if kind == wire.A:
        values = list(map(int, parts[1:6]))
        values += list(map(int, parts[7:])) if len(parts) == 12 else [0, 0, 0, 0, 0]
    else:
        values = list(map(int, parts[1:]))
    return wire.encode(kind, values)


def text_frame(packet):
    kind, values = wire.decode(packet)
    if kind == wire.A:
        return "A " + " ".join(map(str, values[:5])) + " E1 " + " ".join(map(str, values[5:]))
    if kind == wire.U:
        return "U " + " ".join(map(str, values))
    return "MCGSBT"[kind] + " " + " ".join(map(str, values))


def encode_lines(text):
    if isinstance(text, bytes):
        text = text.decode()
    return b"".join(frame(line) for line in text.splitlines() if line)


def decode_lines(data):
    return "".join(text_frame(packet) + "\n" for packet in data.split(b"\0") if packet).encode()
