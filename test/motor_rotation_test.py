"""
motor_rotation_test.py — Minimal motor rotation test.

Moves the output shaft: 0° → 180° → 0° using settings from settings.json.

Usage:
  python test/motor_rotation_test.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jig_controller import JigSettings, connect_hardware, load_json


def main() -> None:
    settings_path = PROJECT_ROOT / "settings.json"
    raw_settings = load_json(settings_path)
    cfg = JigSettings.from_dict(raw_settings)

    hw = connect_hardware(cfg)
    m = cfg.motion

    print(f"Connected on {hw.port}  speed={m.rot_speed_rpm} RPM  acc={m.rot_acc}")
    print(f"Enabled: {hw.motor.is_enabled()}")

    try:
        # 1) Go to absolute 0°
        print("\n[1] Moving to 0° (output) ...")
        hw.motor.move_absolute_deg_output(0.0, speed_rpm=m.rot_speed_rpm, acc=m.rot_acc)
        ok = hw.motor.wait_until_reached(timeout_s=30.0)
        deg = hw.motor.read_realtime_position_deg_output()
        print(f"    reached={ok}  position={deg:.3f}°")

        time.sleep(0.5)

        # 2) Go to 180°
        print("[2] Moving to 180° (output) ...")
        hw.motor.move_absolute_deg_output(180.0, speed_rpm=m.rot_speed_rpm, acc=m.rot_acc)
        ok = hw.motor.wait_until_reached(timeout_s=30.0)
        deg = hw.motor.read_realtime_position_deg_output()
        print(f"    reached={ok}  position={deg:.3f}°")

        time.sleep(0.5)

        # 3) Return to 0°
        print("[3] Moving back to 0° (output) ...")
        hw.motor.move_absolute_deg_output(0.0, speed_rpm=m.rot_speed_rpm, acc=m.rot_acc)
        ok = hw.motor.wait_until_reached(timeout_s=30.0)
        deg = hw.motor.read_realtime_position_deg_output()
        print(f"    reached={ok}  position={deg:.3f}°")

        print("\nTest complete.")

    except KeyboardInterrupt:
        print("\n[ABORT] Interrupted.")
    finally:
        hw.close()
        print("[exit] Serial port closed.")


if __name__ == "__main__":
    main()
