# ADCS binary protocol v1

This is the shared simulator ↔ YUAA development board ↔ OBC wire contract.
It replaces the ASCII line protocol completely. Upgrade both endpoints together;
there is no automatic fallback or negotiation. `M/C/G/S/B/T/A` below are names
for numeric identifiers, **not transmitted ASCII characters**.

## Framing

A frame is `COBS(header || payload || crc16_le) || 0x00`. COBS removes all zero
bytes from the encoded body; zero is the frame delimiter. The one-byte header
contains `version=1` in bits 7–3 and the **3-bit type** in bits 2–0. Type 7 is
reserved. Payload sizes are fixed by type; no length field is needed.

CRC is CRC-16/CCITT-FALSE: polynomial `0x1021`, initial `0xffff`, no reflection,
no final XOR. It covers the unencoded header and payload; its two bytes are
little endian. COBS encodes those CRC bytes too. The maximum accepted encoded
body is 64 bytes, excluding the delimiter. Empty delimiters are ignored.

Firmware buffers a complete frame and verifies framing, version, length, CRC,
and field constraints before publishing data. Corrupt/oversized frames are
ignored through the next delimiter. A lost delimiter can discard two frames;
the following valid frame resynchronizes. No retransmission occurs: missing
control requests eventually time out, and cached readings retain their original
freshness limits. The simulator stops on a corrupt, unexpected, or wrong-tick
reply instead of applying an uncertain command. UART/USB diagnostics must use a
separate channel. Serial configuration is 8N1, no flow control.

## Payloads and units

All integers are little endian, signed integers use two's complement, and fields
are packed without padding. `uN/iN` indicate N-bit unsigned/signed integers.
Vector components are ordered x, y, z. Sensor values include configured bias/noise
and remain in the sensor frame; calibration/fusion still belongs to firmware.

| Type | ID | Payload in transmission order | Payload bytes | Complete frame bytes |
| --- | --- | --- | ---: | ---: |
| M | 0 | time_ms:u32, sensor_id:u8, magnetic_xyz:3×i24 | 14 | 19 |
| C | 1 | time_ms:u32, sensor_id:u8, acceleration_xyz:3×i32 | 17 | 22 |
| G | 2 | time_ms:u32, angular_rate_xyz:3×i24 | 13 | 18 |
| S | 3 | time_ms:u32, valid:u8, sun_xyz:3×i16 | 11 | 16 |
| B | 4 | time_ms:u32, soc_permille:u16, power_mW:i32 | 10 | 15 |
| T | 5 | tick:u32, time_ms:u32, period_ms:u16 | 10 | 15 |
| A | 6 | tick:u32, dipole_xyz:3×i24, boom_mm:u16, rate_time_ms:u32, rate_valid:u8, rate_xyz:3×i24 | 29 | 34 |

Quantization rounds to nearest integer, ties **away from zero**. Values outside
the representable range cause a host error rather than wrapping or saturating.

- Magnetic field: 1 nT/LSB, range −8,388,608…8,388,607 nT.
- Acceleration: 1 µm/s²/LSB, signed 32-bit range (approximately ±2147 m/s²).
- Gyroscope and estimated rates: 1 µrad/s/LSB, approximately ±8.3886 rad/s.
- Dipole: 1 µA·m²/LSB, approximately ±8.3886 A·m². Actuator limits remain lower.
- Boom: absolute extension in millimetres, 0…65535; not an increment.
- Sun: signed integer scaled by 32767; `valid` is 0 or 1. Invalid vectors are zero.
- Battery: SOC 0…1000 permille. Signed mW is net battery power: positive charging,
  negative discharging. Power describes the last completed dynamics step.
- M/C IDs are zero-based, separate namespaces; firmware rejects IDs beyond its
  configured capacity. The wire supports 256 IDs per sensor family.
- `period_ms` must be 1…65535. Millisecond timestamps and ticks wrap modulo 2³²;
  firmware uses unsigned elapsed-time arithmetic. Avoid gaps approaching 2³¹ ms.
- A rate validity is 0 or 1. Invalid or unrepresentable estimates have validity
  zero and all three rate components zero. Estimated rates are inertial angular
  velocity expressed in body axes. The preserved controller only estimates rates
  in pointing mode, so invalid telemetry during detumbling/deployment is expected.

## Exchange and clocks

The simulator queues due M/C/G/S readings, then B and T, into **one host write**.
Each valid T produces exactly one A with the identical tick; other packet types
produce no reply. T commits the preceding sensor snapshot. The simulator waits
for A before advancing the dynamics and holds the command until the next exchange.
A response deadline covers the entire frame; fragments never restart it.
The batch is bounded to 4096 bytes. There is no acknowledgement/retry of individual
sensor packets, and no claim that all packets in a batch were received atomically.

In simulated time, firmware uses host time_ms. In real-time simulation, both OBC
and YUAA replace sensor timestamps with RTOS receipt time and execute T at RTOS
time, while echoing the host tick. A.rate_time_ms is then in the RTOS epoch,
not necessarily the simulator epoch. Host period/time cannot advance the RTOS
clock. YUAA real-time simulation now uses this request/reply contract instead of
its former unsolicited output stream. Physical sensor mode is unchanged.

Binary encoding reduces traffic but cannot eliminate host/USB scheduling jitter
or oscillator differences. The simulator uses absolute monotonic deadlines
(`start + simulated_time`), never a cumulative per-step sleep. It stops when lag
exceeds `simulation.max_realtime_lag_s` (default **0.25 s**, positive finite),
rather than silently running the plant slower than the firmware clock. Choose a
smaller bound for stricter experiments; this is a detection limit, not automatic
clock synchronization. No physics steps or sensor samples are skipped to catch up.

A cycle with two M, two C, one G, B, T and A uses **164 bytes**, including framing
and CRC (180 with a sun update). Sequential wire time is about 1.78 ms at 921600
baud, or 14.24 ms at 115200. **Binary alone still cannot fit that full exchange
into 10 ms at 115200.** Retain 921600 for the OBC 100 Hz experiment, with matching
`YUAA_ADCS_UART_BAUDRATE` and `hitl.baudrate`; leave `hitl.log_usb_rx: false`.
YUAA UART speed remains its configured board speed; native USB CDC baud settings
do not determine USB throughput. RTOS scheduling and host work consume the rest
of the cycle budget. Magnetic gaps over the development estimator's 20 ms limit
can still reset the derivative and keep commands zero.

## Implementations and verification

`adcs_wire.h` is an identical, bounded, allocation-free C codec in all projects.
`sim_protocol.c` applies decoded payloads to each firmware's sensor state and
encodes command snapshots. Python's `cubesat_sim/protocol.py` implements the same
contract. Simulator tests compile the C codec and compare its frames with Python,
including signed limits, CRC errors, fragmentation and resynchronization. Native
algorithm fixtures may still use text adapters; no live transport accepts text.

## Golden frame

A T request with tick=1, time_ms=1000, period_ms=10 is exactly these bytes:

```text
03 0d 01 01 01 03 e8 03 01 02 0a 03 a9 17 00
```

The unencoded header is `0x0d` (version 1, type 5). The final zero is the
frame delimiter, not part of the CRC input.
