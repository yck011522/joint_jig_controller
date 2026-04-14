"""
controller_cli.py — Command-line interface for the joint placement jig.

This is a thin CLI wrapper around the shared ``jig_controller`` module.
All hardware logic, configuration parsing, stall detection, and design
validation live in jig_controller.py so that a future GUI can reuse them.

The CLI adds only:
  - Console text output / ``print()``
  - Non-blocking Enter detection (msvcrt on Windows)
  - Interactive ``prompt_choice()`` for tube selection
  - The main while-loop that drives the tube→joint→install workflow
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from jig_controller import (
    JigSettings,
    HardwareContext,
    RotationResult,
    append_event,
    build_offset_map,
    connect_hardware,
    design_log_path,
    ensure_schema_version,
    load_json,
    now_iso_local,
    rotate_with_stall_detection,
    safe_get,
    validate_offsets_complete,
)


# ═══════════════════════════════════════════════════════════════
#  CLI-only helpers (console I/O)
# ═══════════════════════════════════════════════════════════════

def enter_pressed_nonblocking() -> bool:
    """Return True if ENTER was pressed since the last check.

    Uses ``msvcrt`` on Windows; returns False on other platforms (caller
    should fall back to blocking ``input()``).
    """
    try:
        import msvcrt  # type: ignore
    except Exception:
        return False

    if msvcrt.kbhit():
        ch = msvcrt.getch()
        if ch in (b"\r", b"\n"):
            return True
    return False


def prompt_choice(
    prompt_text: str,
    valid_range: range,
    *,
    allow_quit: bool = True,
) -> Optional[int]:
    """Prompt the user for an integer within *valid_range*.

    Returns the chosen integer, or ``None`` if the user types 'q'/'quit'.
    """
    while True:
        raw = input(prompt_text).strip().lower()
        if allow_quit and raw in ("q", "quit", "exit"):
            return None
        try:
            value = int(raw)
        except ValueError:
            print(
                f"  [!] Please enter a number "
                f"({valid_range.start}–{valid_range.stop - 1})"
                + (" or 'q' to quit." if allow_quit else ".")
            )
            continue
        if value not in valid_range:
            print(
                f"  [!] Out of range. Choose "
                f"{valid_range.start}–{valid_range.stop - 1}"
                + (" or 'q' to quit." if allow_quit else ".")
            )
            continue
        return value


# ═══════════════════════════════════════════════════════════════
#  Main CLI workflow
# ═══════════════════════════════════════════════════════════════

def main() -> None:
    base_dir = Path(__file__).resolve().parent

    # ── 1) Load settings ─────────────────────────────────────────
    settings_path = base_dir / "settings.json"
    if not settings_path.exists():
        raise RuntimeError(f"settings.json not found: {settings_path}")

    raw_settings = load_json(settings_path)
    cfg = JigSettings.from_dict(raw_settings)

    # ── 2) Connect hardware (auto-detect or fixed port) + reset motor ──
    hw: HardwareContext = connect_hardware(cfg)

    # ── 3) Load design file ──────────────────────────────────────
    design_path = base_dir / "design" / "dummy.json"
    if not design_path.exists():
        hw.close()
        raise RuntimeError(f"Design file not found: {design_path}")

    design = load_json(design_path)
    ensure_schema_version(design, expected=1)

    tubes = design.get("tubes", [])
    if not isinstance(tubes, list) or len(tubes) == 0:
        hw.close()
        raise RuntimeError("Design file contains no tubes[].")

    # ── 4) Pre-compute shared state ──────────────────────────────
    offset_map = build_offset_map(raw_settings)
    log_path = design_log_path(design_path, raw_settings)

    # ── 5) Multi-tube workflow loop ──────────────────────────────
    tubes_completed = 0

    try:
        while True:
            # ── Tube selection menu ──────────────────────────────
            print("\n" + "=" * 60)
            print("  TUBE SELECTION")
            print("=" * 60)
            for idx, tube_entry in enumerate(tubes):
                display_id = tube_entry.get("id", f"tube_{idx}")
                length = tube_entry.get("length", "?")
                num_joints = len(tube_entry.get("joints", []) or [])
                print(f"  [{idx}] id={display_id}  length={length}mm  joints={num_joints}")

            selected_index = prompt_choice(
                "\nSelect tube index (or 'q' to quit): ",
                range(len(tubes)),
            )
            if selected_index is None:
                print("\n[quit] Operator chose to exit.")
                break

            tube = tubes[selected_index]

            # Validate that all joint types/orientations have offsets
            try:
                validate_offsets_complete(tube, offset_map)
            except RuntimeError as e:
                print(f"\n[error] {e}")
                print("Skipping this tube. Fix offsets in settings.json and try again.")
                continue

            tube_id = str(tube.get("id", f"tube_{selected_index}"))
            tube_length = float(tube.get("length", 0.0))
            joints = list(tube.get("joints", []) or [])
            joints.sort(key=lambda j: float(j.get("position_mm", 0.0)))

            print(f"\n--- Initiating Tube: {tube_id}  length={tube_length}mm  joints={len(joints)} ---")
            print(f"Log will be saved to:  {log_path}")
            # print(f"Tolerance: linear ±{cfg.linear.linear_tol_mm} mm | rotation ±{cfg.motion.rot_tol_deg} deg")
            # print(f"Motor:     speed={cfg.motion.rot_speed_rpm} RPM  acc={cfg.motion.rot_acc}")
            input("\nMount tube, then press Enter to START ...")

            # ── Begin tube assembly ──────────────────────────────
            tube_start_time = time.perf_counter()
            hw.motor.zero_here()

            append_event(log_path, {
                "ts": now_iso_local(),
                "event": "tube_start",
                "design_file": design_path.name,
                "tube_id": tube_id,
                "joint_id": None,
                "joint_type": None,
                "ori": None,
                "joint_seq": None,
                "tube_length_mm": tube_length,
                "num_joints": len(joints),
            })

            for joint_index, joint_entry in enumerate(joints):
                joint_id = str(joint_entry.get("id", f"joint_{joint_index}"))
                joint_type = str(joint_entry.get("type", "")).strip()
                joint_ori = str(joint_entry.get("ori", "")).strip()
                position_mm = float(joint_entry.get("position_mm", 0.0))
                rotation_deg = float(joint_entry.get("rotation_deg", 0.0))

                joint_offset = offset_map[(joint_type, joint_ori)]
                target_mm = position_mm + cfg.linear.sensor_global_offset_mm + joint_offset

                print("\n" + "-" * 60)
                print(
                    f"Prepare Joint {joint_id} ({joint_index+1} of {len(joints)})"
                    f"| type={joint_type} ori={joint_ori}"
                )
                print(
                    f"-  Distance Target: position_mm={position_mm:.1f}mm  -> "
                    f"target_mm={target_mm:.1f}mm (global+joint offsets applied)"
                )
                print(f"-  Rotation Target: rotation_deg={rotation_deg:.1f}°")

                # ── LINEAR POSITIONING ───────────────────────────
                linear_start_time = time.perf_counter()
                print("\nMove carriage to target. Press ENTER to confirm (Rotation will then begin) ")
                # print("Live: current_mm_raw | measured_mm | target_mm | error_mm")

                while True:
                    raw_sensor_mm = int(hw.sensor.read_mm())
                    measured_mm = float(raw_sensor_mm) + cfg.linear.sensor_global_offset_mm
                    error_mm = measured_mm - target_mm
                    
                    if error_mm > 0.7:
                        indicator = "←"
                    elif error_mm < -0.7:
                        indicator = "→"
                    else:
                        indicator = "✓"

                    line = (
                        f"\rPosition raw={raw_sensor_mm:6d} mm | "
                        f"meas={measured_mm:8.2f} | "
                        f"target={target_mm:8.2f} | err={error_mm:7.2f}   "
                        f"{indicator} "
                        f"(tol ±{cfg.linear.linear_tol_mm})"
                    )
                    print(line, end="", flush=True)

                    if enter_pressed_nonblocking():
                        if abs(error_mm) <= cfg.linear.linear_tol_mm:
                            linear_time_ms = int(
                                (time.perf_counter() - linear_start_time) * 1000
                            )
                            print("\n[ok] Linear position confirmed.")
                            append_event(log_path, {
                                "ts": now_iso_local(),
                                "event": "linear_confirm",
                                "design_file": design_path.name,
                                "tube_id": tube_id,
                                "joint_id": joint_id,
                                "joint_type": joint_type,
                                "ori": joint_ori,
                                "joint_seq": joint_index,
                                "target_mm": target_mm,
                                "measured_mm_raw": raw_sensor_mm,
                                "measured_mm": measured_mm,
                                "error_mm": error_mm,
                                "tol_mm": cfg.linear.linear_tol_mm,
                                "linear_time_ms": linear_time_ms,
                            })
                            break
                        else:
                            print(
                                "\n[warn] Outside tolerance. Keep adjusting "
                                "and press ENTER again."
                            )

                    time.sleep(0.1)

                # ── ROTATION (with stall detection) ──────────────
                def _on_stall(actual_deg: float, target: float, elapsed_ms: int) -> None:
                    """CLI stall callback — prints a warning, logs the event,
                    and blocks until the operator presses Enter."""
                    print(
                        f"\n[STALL] Motor stall detected! The rotation axis "
                        f"may be blocked."
                        f"\n        Position at stall: {actual_deg:.2f}° "
                        f"(target: {target:.2f}°)  "
                        f"time since motion start: {elapsed_ms} ms"
                    )
                    append_event(log_path, {
                        "ts": now_iso_local(),
                        "event": "stall_detected",
                        "design_file": design_path.name,
                        "tube_id": tube_id,
                        "joint_id": joint_id,
                        "joint_type": joint_type,
                        "ori": joint_ori,
                        "joint_seq": joint_index,
                        "target_deg": target,
                        "actual_deg": actual_deg,
                        "time_since_motion_start_ms": elapsed_ms,
                    })
                    input(
                        "[STALL] Clear the obstruction, then press ENTER "
                        "to retry the rotation... "
                    )

                rot_result: RotationResult = rotate_with_stall_detection(
                    hw.motor,
                    rotation_deg,
                    cfg.motion,
                    cfg.stall,
                    on_stall=_on_stall,
                )

                print(
                    f"[rot] reached={rot_result.reached} "
                    f"target={rotation_deg:.2f}° "
                    f"actual={rot_result.actual_deg:.2f}° "
                    f"time={rot_result.rotation_time_ms}ms"
                )

                append_event(log_path, {
                    "ts": now_iso_local(),
                    "event": "rotation_reached",
                    "design_file": design_path.name,
                    "tube_id": tube_id,
                    "joint_id": joint_id,
                    "joint_type": joint_type,
                    "ori": joint_ori,
                    "joint_seq": joint_index,
                    "target_deg": rotation_deg,
                    "actual_deg": rot_result.actual_deg,
                    "tol_deg": cfg.motion.rot_tol_deg,
                    "rot_speed_rpm": cfg.motion.rot_speed_rpm,
                    "rot_acc": cfg.motion.rot_acc,
                    "rotation_time_ms": rot_result.rotation_time_ms,
                    "motor_reached_flag": rot_result.reached,
                })

                # ── INSTALL JOINT ────────────────────────────────
                print(
                    f"\nInstall joint now: type={joint_type} ori={joint_ori}. "
                    f"Press ENTER when installed."
                )
                install_start_time = time.perf_counter()
                input()
                install_time_ms = int(
                    (time.perf_counter() - install_start_time) * 1000
                )

                append_event(log_path, {
                    "ts": now_iso_local(),
                    "event": "install_confirm",
                    "design_file": design_path.name,
                    "tube_id": tube_id,
                    "joint_id": joint_id,
                    "joint_type": joint_type,
                    "ori": joint_ori,
                    "joint_seq": joint_index,
                    "install_time_ms": install_time_ms,
                })

            # ── Tube complete ────────────────────────────────────
            total_tube_time_ms = int(
                (time.perf_counter() - tube_start_time) * 1000
            )
            append_event(log_path, {
                "ts": now_iso_local(),
                "event": "tube_complete",
                "design_file": design_path.name,
                "tube_id": tube_id,
                "joint_id": None,
                "joint_type": None,
                "ori": None,
                "joint_seq": None,
                "tube_time_ms": total_tube_time_ms,
                "completed": True,
            })

            tubes_completed += 1
            print("\n" + "=" * 60)
            print(
                f"Tube complete: {tube_id} | "
                f"time = {total_tube_time_ms/1000:.1f}s | "
                f"session total = {tubes_completed} tube(s)"
            )
            print(f"Log written to: {log_path}")
            # Loop back to tube selection

    except KeyboardInterrupt:
        print("\n\n[abort] Interrupted by operator (Ctrl+C).")

    finally:
        print(f"\nSession summary: {tubes_completed} tube(s) completed.")
        hw.close()
        print("[exit] Serial port closed.")


if __name__ == "__main__":
    main()
