"""
stall_detection_test.py — Interactive test for motor stall protection.

Workflow:
  1. Connect to motor over Modbus RTU.
  2. Read & display current stall parameters.
  3. Write tuned stall detection parameters (100 RPM / 400 mA / 50 ms).
  4. Wait for operator to block the motor shaft.
  5. Send a motion command (300 RPM motor-side, acc=100).
  6. Poll status flags until stall protection is triggered.
  7. Clear stall protection and re-enable the motor.
  8. Reset stall current to 2000 mA (effectively disabling detection).

Usage:
  cd <project_root>
  python -m test.stall_detection_test
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Allow importing from the project root when run as `python -m test.stall_detection_test`
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pymodbus.client import ModbusSerialClient
from motor_comm import ZDTEmmMotor, MotorKinematics


# ── Configuration ────────────────────────────────────────────
PORT = "COM21"
BAUD = 115200
SLAVE_ID = 2

# Tuned stall detection parameters (from testing with blocked joint)
STALL_SPEED_RPM = 100    # detect stall when RPM drops below this
STALL_CURRENT_MA = 400   # detect stall when phase current exceeds this
STALL_TIME_MS = 50        # how long the condition must persist

# Motion parameters for the stall test
MOTION_SPEED_RPM = 300    # motor-side RPM
MOTION_ACC = 100          # acceleration (0–255 scale)
MOTION_PULSES = 200 * 16 * 10  # large relative move (~10 motor revolutions)

# "Disabled" stall current — high enough to never trigger
STALL_DISABLED_CURRENT_MA = 2000


def fmt_flags(flags: int) -> str:
    parts = []
    if flags & 0x01: parts.append("ENABLED")
    if flags & 0x02: parts.append("REACHED")
    if flags & 0x04: parts.append("STALL_DETECTED")
    if flags & 0x08: parts.append("STALL_PROTECTED")
    if flags & 0x10: parts.append("LIMIT_L")
    if flags & 0x20: parts.append("LIMIT_R")
    if flags & 0x80: parts.append("POWER_OFF")
    return f"0x{flags:02X} [{', '.join(parts) or 'none'}]"


def main() -> None:
    print("=" * 60)
    print("  STALL DETECTION TEST")
    print("=" * 60)

    # ── 1. Connect ────────────────────────────────────────────
    client = ModbusSerialClient(
        port=PORT, baudrate=BAUD, parity="N", stopbits=1, bytesize=8, timeout=0.4,
    )
    if not client.connect():
        print(f"[ERROR] Cannot connect to {PORT} @ {BAUD}")
        return

    motor = ZDTEmmMotor(
        client,
        slave_id=SLAVE_ID,
        kinematics=MotorKinematics(motor_steps_per_rev=200, microsteps=16, gearbox_ratio=30.0),
    )

    try:
        # Quick probe
        flags = motor.read_status_flags()
        print(f"\n[1] Connected. Status flags: {fmt_flags(flags)}")

        # ── 2. Read current stall parameters ──────────────────
        params = motor.read_stall_params()
        print(f"\n[2] Current stall parameters:")
        print(f"    mode        = {params['mode']}  (0=off, 1=enable+release, 2=reset-to-zero)")
        print(f"    speed_rpm   = {params['speed_rpm']}")
        print(f"    current_ma  = {params['current_ma']}")
        print(f"    time_ms     = {params['time_ms']}")

        # ── 3. Write tuned stall detection parameters ─────────
        print(f"\n[3] Writing stall params: mode=1, speed={STALL_SPEED_RPM} RPM, "
              f"current={STALL_CURRENT_MA} mA, time={STALL_TIME_MS} ms")
        motor.write_stall_params(
            mode=0x01,
            speed_rpm=STALL_SPEED_RPM,
            current_ma=STALL_CURRENT_MA,
            time_ms=STALL_TIME_MS,
            store=False,  # volatile — don't persist to EEPROM
        )

        # Verify by reading back
        params = motor.read_stall_params()
        print(f"    Readback: mode={params['mode']}, speed={params['speed_rpm']} RPM, "
              f"current={params['current_ma']} mA, time={params['time_ms']} ms")

        # ── 4. Ensure motor is enabled ────────────────────────
        if not motor.is_enabled():
            print("\n[4] Motor not enabled — enabling now...")
            motor.set_enabled(True)
            time.sleep(0.2)
        print(f"[4] Motor enabled: {motor.is_enabled()}")

        # ── 5. Wait for operator ──────────────────────────────
        print("\n" + "-" * 60)
        print("  STALL DETECTION IS NOW ACTIVE.")
        print("  Block the motor shaft to simulate a stall, then press ENTER")
        print("  to send a motion command.")
        print("-" * 60)
        input("Press ENTER when ready to send motion command... ")

        # ── 6. Send motion command ────────────────────────────
        print(f"\n[6] Sending relative move: {MOTION_PULSES} pulses, "
              f"speed={MOTION_SPEED_RPM} RPM, acc={MOTION_ACC}")
        motor.move_relative_pulses_motor(
            MOTION_PULSES,
            speed_rpm=MOTION_SPEED_RPM,
            acc=MOTION_ACC,
        )

        # ── 7. Poll stall flags ──────────────────────────────
        print("\n[7] Polling status flags (Ctrl+C to abort)...\n")
        t0 = time.time()
        stall_protection_seen = False

        while True:
            flags = motor.read_status_flags()
            elapsed = time.time() - t0
            line = f"\r  t={elapsed:6.2f}s  flags={fmt_flags(flags)}   "
            print(line, end="", flush=True)

            if flags & motor.FLAG_STALL_PROTECTED:
                stall_protection_seen = True
                print(f"\n\n  >>> STALL PROTECTION TRIGGERED at t={elapsed:.2f}s <<<")
                break

            if flags & motor.FLAG_STALL_DETECTED:
                # Stall condition detected but protection not yet triggered
                pass  # keep polling

            # If motion completed normally (no stall), also stop
            if (flags & motor.FLAG_REACHED) and elapsed > 0.5:
                print(f"\n\n  Motor reached target without stalling at t={elapsed:.2f}s.")
                break

            time.sleep(0.1)

        # ── 8. Recovery ───────────────────────────────────────
        if stall_protection_seen:
            print("\n[8] Clearing stall protection...")
            motor.clear_stall_protection()
            time.sleep(0.2)

            print("    Re-enabling motor...")
            motor.set_enabled(True)
            time.sleep(0.2)

            flags = motor.read_status_flags()
            print(f"    Status after recovery: {fmt_flags(flags)}")
        else:
            print("\n[8] No stall detected — skipping recovery.")

        # ── 9. Reset stall current to disable detection ───────
        print(f"\n[9] Resetting stall current to {STALL_DISABLED_CURRENT_MA} mA "
              f"(effectively disabling stall detection)...")
        motor.write_stall_params(
            mode=0x01,
            speed_rpm=STALL_SPEED_RPM,
            current_ma=STALL_DISABLED_CURRENT_MA,
            time_ms=STALL_TIME_MS,
            store=False,
        )

        params = motor.read_stall_params()
        print(f"    Final stall params: mode={params['mode']}, speed={params['speed_rpm']} RPM, "
              f"current={params['current_ma']} mA, time={params['time_ms']} ms")

        print("\n" + "=" * 60)
        print("  TEST COMPLETE")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n[ABORT] Interrupted by user.")
    finally:
        client.close()
        print("[exit] Serial port closed.")


if __name__ == "__main__":
    main()
