"""Absolute monotonic pacing; never silently run an RTOS HITL clock in slow motion."""
import time


def pace(start, simulation_time, max_lag):
    deadline = start + simulation_time
    remaining = deadline - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)
    lag = time.monotonic() - deadline
    if lag > max_lag:
        raise RuntimeError(
            f"Real-time deadline missed: lag={lag:.3f}s exceeds {max_lag:.3f}s. "
            "The host/serial exchange cannot sustain the configured rate; "
            "simulation stopped to avoid diverging from the OBC RTOS clock.")
