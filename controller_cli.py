"""
controller_cli.py — Command-line interface for the joint placement jig.

This is a thin CLI wrapper around the shared ``jig_controller`` module.
All hardware logic, configuration parsing, stall detection, and design
validation live in jig_controller.py so that a future GUI can reuse them.

The CLI adds only:
  - Console text output / ``print()``
  - Non-blocking Enter detection (msvcrt on Windows)
  - Interactive prompts for design-file and tube selection
  - The main while-loop that drives the tube → joint → install workflow
  - Tube abandonment support (type 'a' at any step)
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from jig_controller import (
    EVENT_INSTALL_CONFIRM,
    EVENT_LINEAR_CONFIRM,
    EVENT_ROTATION_REACHED,
    EVENT_STALL_DETECTED,
    EVENT_TUBE_ABANDONED,
    EVENT_TUBE_INSTALL_COMPLETE,
    EVENT_TUBE_START,
    JigSettings,
    HardwareContext,
    RotationResult,
    TubeCompletionInfo,
    append_event,
    build_offset_map,
    connect_hardware,
    design_log_path,
    ensure_schema_version,
    format_duration_ms,
    format_time_ago,
    get_last_design_dir,
    get_tube_completion_map,
    load_json,
    now_iso_local,
    parse_log_events,
    rotate_with_stall_detection,
    safe_get,
    save_settings,
    set_last_design_dir,
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

# ═══════════════════════════════════════════════════════════════
#  Design file selection (CLI)
# ═══════════════════════════════════════════════════════════════

def select_design_file(base_dir: Path) -> Optional[Path]:
    """Prompt the user to enter a path to a design JSON file.

    Lists .json files in the ``design/`` subfolder as suggestions, but
    accepts any valid path.  Returns None if the user quits.
    """
    design_dir = base_dir / "design"

    # Show available files in the default design/ folder
    if design_dir.is_dir():
        json_files = sorted(design_dir.glob("*.json"))
        if json_files:
            print("\nDesign files found in design/ folder:")
            for i, f in enumerate(json_files):
                print(f"  [{i}] {f.name}")
            print()
            raw = input(
                "Enter file number, full path, or 'q' to quit: "
            ).strip()
            if raw.lower() in ("q", "quit", "exit"):
                return None
            # Try as index
            try:
                idx = int(raw)
                if 0 <= idx < len(json_files):
                    return json_files[idx]
            except ValueError:
                pass
            # Try as path
            p = Path(raw)
            if p.exists():
                return p.resolve()
            # Try relative to design/
            p2 = design_dir / raw
            if p2.exists():
                return p2.resolve()
            print(f"  [!] File not found: {raw}")
            return select_design_file(base_dir)
    else:
        raw = input("Enter path to design JSON file (or 'q' to quit): ").strip()
        if raw.lower() in ("q", "quit", "exit"):
            return None
        p = Path(raw)
        if p.exists():
            return p.resolve()
        print(f"  [!] File not found: {raw}")
        return select_design_file(base_dir)


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

    # ── Outer loop: design-file selection ────────────────────────
    tubes_completed = 0

    try:
        while True:
            # ── 3) Select design file ────────────────────────────
            design_path = select_design_file(base_dir)
            if design_path is None:
                print("\n[quit] Operator chose to exit.")
                break

            design = load_json(design_path)
            ensure_schema_version(design, expected=1)

            tubes = design.get("tubes", [])
            if not isinstance(tubes, list) or len(tubes) == 0:
                print("[error] Design file contains no tubes[]. Try another file.")
                continue

            # Remember this directory for next time
            set_last_design_dir(raw_settings, str(design_path.parent))
            save_settings(settings_path, raw_settings)

            # ── 4) Pre-compute shared state ──────────────────────
            offset_map = build_offset_map(raw_settings)
            log_path = design_log_path(design_path, raw_settings)

            print(f"\nDesign loaded: {design_path.name}  ({len(tubes)} tube(s))")
            print(f"Log file: {log_path}")

            # ── 5) Tube selection loop ───────────────────────────
            while True:
                # Parse log to get completion status for each tube
                log_events = parse_log_events(log_path)
                completion_map = get_tube_completion_map(log_events)

                print("\n" + "=" * 60)
                print("  TUBE SELECTION")
                print("=" * 60)
                print(f"  {'#':>3}  {'ID':<10} {'Length':>8} {'Joints':>7}  {'Last Assembled'}")
                print(f"  {'─'*3}  {'─'*10} {'─'*8} {'─'*7}  {'─'*30}")

                for idx, tube_entry in enumerate(tubes):
                    display_id = tube_entry.get("id", f"tube_{idx}")
                    length = tube_entry.get("length", "?")
                    num_joints = len(tube_entry.get("joints", []) or [])

                    # Check completion status from log
                    info = completion_map.get(str(display_id))
                    if info:
                        ago = format_time_ago(info.completed_at)
                        dur = format_duration_ms(info.tube_time_ms)
                        status_str = f"{ago} ({dur})"
                    else:
                        status_str = "—"

                    print(f"  [{idx:>2}] {display_id:<10} {length:>7}mm {num_joints:>6}   {status_str}")

                print(f"\n  Type 'q' to choose a different design file.")
                selected_index = prompt_choice(
                    "Select tube index: ",
                    range(len(tubes)),
                )
                if selected_index is None:
                    # Go back to design file selection
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
                print("Type 'a' at any prompt to abandon this tube.\n")
                raw = input("Mount tube, then press Enter to START ... ").strip().lower()
                if raw in ("a", "abandon"):
                    print("[abandon] Tube not started.")
                    continue

                # ── Begin tube assembly ──────────────────────────
                tube_start_time = time.perf_counter()
                hw.motor.zero_here()
                abandoned = False

                append_event(log_path, {
                    "ts": now_iso_local(),
                    "event": EVENT_TUBE_START,
                    "design_file": design_path.name,
                    "tube_id": tube_id,
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
                        f" | type={joint_type} ori={joint_ori}"
                    )
                    print(
                        f"-  Distance Target: position_mm={position_mm:.1f}mm  -> "
                        f"target_mm={target_mm:.1f}mm (global+joint offsets applied)"
                    )
                    print(f"-  Rotation Target: rotation_deg={rotation_deg:.1f}°")

                    # ── LINEAR POSITIONING ───────────────────────
                    linear_start_time = time.perf_counter()
                    print("\nMove carriage to target. Press ENTER to confirm (Rotation will then begin)")
                    print("Type 'a' to abandon this tube.\n")

                    linear_confirmed = False
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
                                    "event": EVENT_LINEAR_CONFIRM,
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
                                linear_confirmed = True
                                break
                            else:
                                print(
                                    "\n[warn] Outside tolerance. Keep adjusting "
                                    "and press ENTER again."
                                )

                        # Check for 'a' key (abandon) via msvcrt
                        try:
                            import msvcrt
                            if msvcrt.kbhit():
                                ch = msvcrt.getch()
                                if ch in (b"a", b"A"):
                                    print("\n")
                                    abandoned = True
                                    break
                        except ImportError:
                            pass

                        time.sleep(0.1)

                    if abandoned:
                        break
                    if not linear_confirmed:
                        break

                    # ── ROTATION (with stall detection) ──────────
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
                            "event": EVENT_STALL_DETECTED,
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
                        "event": EVENT_ROTATION_REACHED,
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

                    # ── INSTALL JOINT ────────────────────────────
                    print(
                        f"\nInstall joint now: type={joint_type} ori={joint_ori}. "
                        f"Press ENTER when installed (or 'a' to abandon)."
                    )
                    install_start_time = time.perf_counter()
                    raw_input = input().strip().lower()
                    if raw_input in ("a", "abandon"):
                        abandoned = True
                        break

                    install_time_ms = int(
                        (time.perf_counter() - install_start_time) * 1000
                    )

                    append_event(log_path, {
                        "ts": now_iso_local(),
                        "event": EVENT_INSTALL_CONFIRM,
                        "design_file": design_path.name,
                        "tube_id": tube_id,
                        "joint_id": joint_id,
                        "joint_type": joint_type,
                        "ori": joint_ori,
                        "joint_seq": joint_index,
                        "install_time_ms": install_time_ms,
                    })

                # ── After joint loop: complete or abandoned ──────
                total_tube_time_ms = int(
                    (time.perf_counter() - tube_start_time) * 1000
                )

                if abandoned:
                    print(f"\n[abandon] Tube {tube_id} abandoned.")
                    append_event(log_path, {
                        "ts": now_iso_local(),
                        "event": EVENT_TUBE_ABANDONED,
                        "design_file": design_path.name,
                        "tube_id": tube_id,
                        "tube_time_ms": total_tube_time_ms,
                        "joints_completed": joint_index,
                        "joints_total": len(joints),
                    })
                else:
                    # All joints installed successfully
                    append_event(log_path, {
                        "ts": now_iso_local(),
                        "event": EVENT_TUBE_INSTALL_COMPLETE,
                        "design_file": design_path.name,
                        "tube_id": tube_id,
                        "tube_time_ms": total_tube_time_ms,
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
