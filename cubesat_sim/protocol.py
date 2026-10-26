"""ADCS binary v1: fixed integer fields, little endian, COBS + CRC16.

See PROTOCOL.md. No implicit saturation: out-of-range values are errors.
"""
import binascii
import math

M, C, G, S, B, T, A, U = range(8)
# (width in bytes, signed). Header has five version bits and three type bits.
U32 = (4, False)
I24 = (3, True)
I32 = (4, True)
U16 = (2, False)
U8 = (1, False)
I16 = (2, True)
FIELDS = (
    (U32, U8, I24, I24, I24),
    (U32, U8, I32, I32, I32),
    (U32, I24, I24, I24),
    (U32, U8, I16, I16, I16),
    (U32, U16, I32),
    (U32, U32, U16),
    (U32, I24, I24, I24, U16, U32, U8, I24, I24, I24),
    (U32, I24, I24, I24, U16, U32, U8, I24, I24, I24, U32, U32, U32, U32, U8),
)
INDEXED_G_FIELDS = (U32, U8, I24, I24, I24)
MAX_FRAME = 64


def quantize(value, scale):
    scaled = float(value) * scale
    if not math.isfinite(scaled):
        raise ValueError("non-finite protocol value")
    # Round half away from zero, matching the C response encoder.
    return math.floor(scaled + .5) if scaled >= 0 else math.ceil(scaled - .5)


def encode(kind, values):
    if not 0 <= kind < len(FIELDS):
        raise ValueError("invalid message type")
    fields = INDEXED_G_FIELDS if kind == G and len(values) == 5 else FIELDS[kind]
    if len(values) != len(fields):
        raise ValueError("invalid message type or field count")
    raw = bytearray([8 | kind])
    try:
        for value, (width, signed) in zip(values, fields):
            raw.extend(int(value).to_bytes(width, 'little', signed=signed))
    except OverflowError as exc:
        raise ValueError("protocol value outside representable range") from exc
    raw.extend(binascii.crc_hqx(raw, 0xffff).to_bytes(2, 'little'))
    out = bytearray([0])
    code_at = 0
    code = 1
    for byte in raw:
        if byte:
            out.append(byte)
            code += 1
        else:
            out[code_at] = code
            code_at = len(out)
            out.append(0)
            code = 1
    out[code_at] = code
    out.append(0)
    return bytes(out)


def decode(packet):
    """Decode one COBS packet excluding its zero delimiter."""
    if not packet or len(packet) > MAX_FRAME:
        raise ValueError("invalid frame length")
    raw = bytearray()
    offset = 0
    while offset < len(packet):
        code = packet[offset]
        offset += 1
        if not code or offset + code - 1 > len(packet):
            raise ValueError("invalid COBS frame")
        raw.extend(packet[offset:offset + code - 1])
        offset += code - 1
        if code != 255 and offset < len(packet):
            raw.append(0)
    if len(raw) < 3 or raw[0] >> 3 != 1 or raw[0] & 7 >= len(FIELDS):
        raise ValueError("unsupported protocol version/type")
    kind = raw[0] & 7
    fields = INDEXED_G_FIELDS if kind == G and len(raw) == 17 else FIELDS[kind]
    if len(raw) != 3 + sum(width for width, _ in fields):
        raise ValueError("invalid payload length")
    if int.from_bytes(raw[-2:], 'little') != binascii.crc_hqx(raw[:-2], 0xffff):
        raise ValueError("frame CRC mismatch")
    values = []
    offset = 1
    for width, signed in fields:
        values.append(int.from_bytes(raw[offset:offset + width], 'little', signed=signed))
        offset += width
    return kind, values
