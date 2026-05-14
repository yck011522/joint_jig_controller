"""
controller_gui.py — Tkinter GUI for the joint placement jig.

Layout:
  ┌──────────────────────────────────────┬───────────────────────┐
  │  LEFT PANEL  (swaps between          │  ENGINEER PANEL       │
  │  Selection screen and Assembly       │  (always visible)     │
  │  Wizard screen)                      │                       │
  └──────────────────────────────────────┴───────────────────────┘

The engineer panel continuously displays sensor / motor / stall
readings from the HardwareHub's cached values (updated every poll cycle).

The left panel shows either:
  A) Selection screen: design-file picker, bar table, bar details,
     "Start Assembly" button.
  B) Assembly wizard: joint image at top (visible across all steps),
     step-by-step workflow with linear colour bar, rotation progress bar,
     and "Abandon Bar" button.

Threading model:
  - A ``HardwareHub`` (from jig_controller.py) owns the Modbus lock
    and runs a background poller thread that caches sensor + motor
    readings.  All hardware I/O goes through the hub.
  - An *assembly worker thread* drives the joint-by-joint workflow
    using hub command methods.  It posts GUI-update messages into a
    ``queue.Queue``.
  - The Tk main loop drains the queue and refreshes the engineer panel
    every 100 ms via ``root.after()``.
"""

from __future__ import annotations

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import label_printer
from jig_controller import (
    EVENT_BAR_ABANDONED,
    EVENT_BAR_INSTALL_COMPLETE,
    EVENT_BAR_START,
    EVENT_INSTALL_CONFIRM,
    EVENT_LINEAR_CONFIRM,
    EVENT_ROTATION_REACHED,
    EVENT_STALL_DETECTED,
    BarCompletionInfo,
    HardwareContext,
    HardwareHub,
    JigSettings,
    MotionSettings,
    RotationResult,
    StallSettings,
    append_event,
    build_offset_map,
    connect_hardware,
    design_log_path,
    ensure_schema_version,
    format_duration_ms,
    format_time_ago,
    get_bar_completion_map,
    get_last_design_dir,
    joint_type_key,
    load_json,
    now_iso_local,
    parse_log_events,
    safe_get,
    save_settings,
    set_last_design_dir,
    validate_offsets_complete,
)


# ═══════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════

POLL_INTERVAL_S = 1 / 30    # HardwareHub background poll interval (~30 Hz)
GUI_DRAIN_MS = 100          # how often the GUI drains the message queue
LABEL_PRINTER_POLL_MS = 2000  # how often to re-check label-printer USB presence
JOG_STEP_DEG = 1.0
JOG_INTERVAL_S = 0.10
WINDOW_TITLE = "Joint Jig Controller"
WINDOW_MIN_SIZE = (1100, 700)


# ═══════════════════════════════════════════════════════════════
#  Message types posted from the assembly worker → GUI
# ═══════════════════════════════════════════════════════════════

@dataclass
class AssemblyMsg:
    """Message from the assembly worker → GUI."""
    kind: str                         # e.g. "step", "linear_update", "done"
    data: Dict[str, Any] = None

    def __post_init__(self):
        if self.data is None:
            self.data = {}


# ═══════════════════════════════════════════════════════════════
#  Assembly worker thread
# ═══════════════════════════════════════════════════════════════

class AssemblyWorker(threading.Thread):
    """Drives the bar assembly workflow in a background thread.

    All hardware I/O goes through the ``HardwareHub``.  Posts
    ``AssemblyMsg`` objects so the GUI can update.  The GUI can set
    ``abandon_event`` to signal the worker to stop early.
    """

    def __init__(
        self,
        hub: Optional[HardwareHub],
        msg_queue: queue.Queue,
        bar: dict,
        bar_id: str,
        joints: List[dict],
        offset_map: Dict,
        cfg: JigSettings,
        design_path: Path,
        log_path: Path,
        raw_settings: dict,
        abandon_event: threading.Event,
        # Events set by the GUI when the operator confirms a step
        confirm_linear_event: threading.Event,
        confirm_install_event: threading.Event,
        stall_retry_event: threading.Event,
        simulate_mode: bool = False,
    ):
        super().__init__(daemon=True, name="assembly-worker")
        self.hub = hub
        self.q = msg_queue
        self.bar = bar
        self.bar_id = bar_id
        self.joints = joints
        self.offset_map = offset_map
        self.cfg = cfg
        self.design_path = design_path
        self.log_path = log_path
        self.raw_settings = raw_settings
        self.abandon = abandon_event
        self.confirm_linear = confirm_linear_event
        self.confirm_install = confirm_install_event
        self.stall_retry = stall_retry_event
        self.simulate_mode = simulate_mode
        self._sim_motor_deg = 0.0

    # ── helpers ──────────────────────────────────────────────────

    def _post(self, kind: str, **kw) -> None:
        self.q.put(AssemblyMsg(kind=kind, data=kw))

    def _log_event(self, event_obj: dict) -> None:
        append_event(self.log_path, event_obj)

    # ── main entry ───────────────────────────────────────────────

    def run(self) -> None:
        try:
            self._run_assembly()
        except Exception as e:
            self._post("error", message=str(e))

    def _run_assembly(self) -> None:
        if not self.simulate_mode and self.hub is None:
            raise RuntimeError("Hardware is not connected.")

        # Zero motor at the start of each bar (real hardware only).
        if not self.simulate_mode and self.hub is not None:
            self.hub.zero_here()

        bar_start = time.perf_counter()

        self._log_event({
            "ts": now_iso_local(),
            "event": EVENT_BAR_START,
            "design_file": self.design_path.name,
            "bar_id": self.bar_id,
            "bar_length_mm": float(self.bar.get("length_mm", 0)),
            "num_joints": len(self.joints),
        })

        joints_completed = 0

        for ji, joint in enumerate(self.joints):
            if self.abandon.is_set():
                break

            joint_id = str(joint.get("joint_id", f"joint_{ji}"))
            jtype_key = joint_type_key(joint)  # e.g. "T20_Female"
            joint_type_raw = str(joint.get("type", "")).strip()
            joint_subtype = str(joint.get("subtype", "")).strip()
            joint_ori = str(joint.get("ori", "")).strip()
            position_mm = float(joint.get("position_mm", 0.0))
            rotation_deg = float(joint.get("rotation_deg", 0.0))

            joint_offset = self.offset_map.get((jtype_key, joint_ori), 0.0)
            target_mm = position_mm - joint_offset

            # Tell the GUI we're starting a new joint (image + title)
            self._post("joint_start", joint_index=ji, joint_total=len(self.joints),
                        joint_id=joint_id, joint_type=jtype_key,
                        joint_type_display=f"{joint_type_raw} {joint_subtype}".strip(),
                        joint_ori=joint_ori,
                        position_mm=position_mm, target_mm=target_mm,
                        rotation_deg=rotation_deg)

            # ── LINEAR POSITIONING ───────────────────────────────
            self._post("step", phase="linear", joint_index=ji)
            self.confirm_linear.clear()
            linear_start = time.perf_counter()

            # Poll sensor (real) or emit synthetic values (simulation)
            # until the GUI signals confirmation.
            if self.simulate_mode:
                while (not self.confirm_linear.is_set()
                       and not self.abandon.is_set()):
                    raw_mm = target_mm - self.cfg.linear.sensor_global_offset_mm
                    measured = target_mm
                    error = 0.0
                    self._post(
                        "linear_update",
                        raw_mm=raw_mm,
                        measured_mm=measured,
                        target_mm=target_mm,
                        error_mm=error,
                    )
                    time.sleep(0.1)
            else:
                while (not self.confirm_linear.is_set()
                       and not self.abandon.is_set()):
                    try:
                        raw_mm = self.hub.read_sensor_mm_sync()
                    except Exception:
                        time.sleep(0.2)
                        continue
                    measured = float(raw_mm) + self.cfg.linear.sensor_global_offset_mm
                    error = measured - target_mm
                    self._post("linear_update",
                               raw_mm=raw_mm, measured_mm=measured,
                               target_mm=target_mm, error_mm=error)
                    time.sleep(0.1)

            if self.abandon.is_set():
                break

            # Log the linear confirmation
            linear_ms = int((time.perf_counter() - linear_start) * 1000)
            if self.simulate_mode:
                raw_mm = target_mm - self.cfg.linear.sensor_global_offset_mm
                measured = target_mm
                error = 0.0
            else:
                raw_mm = self.hub.read_sensor_mm_sync()
                measured = float(raw_mm) + self.cfg.linear.sensor_global_offset_mm
                error = measured - target_mm
            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_LINEAR_CONFIRM,
                "design_file": self.design_path.name,
                "bar_id": self.bar_id,
                "joint_id": joint_id, "joint_type": joint_type_raw,
                "joint_subtype": joint_subtype,
                "ori": joint_ori, "joint_seq": ji,
                "target_mm": target_mm, "measured_mm_raw": raw_mm,
                "measured_mm": measured, "error_mm": error,
                "tol_mm": self.cfg.linear.linear_tol_mm,
                "linear_time_ms": linear_ms,
            })

            # ── ROTATION (with stall detection + progress) ──────
            self._post("step", phase="rotating", joint_index=ji,
                        target_deg=rotation_deg)

            def on_progress(actual_deg: float, target: float) -> None:
                """Called every poll cycle during rotation."""
                self._post("rotation_progress",
                           actual_deg=actual_deg, target_deg=target)

            def on_stall(actual_deg: float, target: float, elapsed_ms: int) -> None:
                """Called from hub on stall — blocks until retry or abandon."""
                self._log_event({
                    "ts": now_iso_local(),
                    "event": EVENT_STALL_DETECTED,
                    "design_file": self.design_path.name,
                    "bar_id": self.bar_id,
                    "joint_id": joint_id, "joint_type": joint_type_raw,
                    "joint_subtype": joint_subtype,
                    "ori": joint_ori, "joint_seq": ji,
                    "target_deg": target, "actual_deg": actual_deg,
                    "time_since_motion_start_ms": elapsed_ms,
                })
                self._post("stall", actual_deg=actual_deg, target_deg=target,
                           elapsed_ms=elapsed_ms)
                self.stall_retry.clear()
                while not self.stall_retry.is_set() and not self.abandon.is_set():
                    time.sleep(0.1)

            if self.simulate_mode:
                rot_start = time.perf_counter()
                start_deg = self._sim_motor_deg
                steps = max(10, int(abs(rotation_deg - start_deg) // 5) + 1)
                for step in range(1, steps + 1):
                    if self.abandon.is_set():
                        break
                    actual_deg = (
                        start_deg
                        + (rotation_deg - start_deg) * (step / steps)
                    )
                    self._sim_motor_deg = actual_deg
                    on_progress(actual_deg, rotation_deg)
                    time.sleep(0.05)
                rotation_ms = int((time.perf_counter() - rot_start) * 1000)
                rot_result = RotationResult(
                    reached=not self.abandon.is_set(),
                    target_deg=rotation_deg,
                    actual_deg=self._sim_motor_deg,
                    rotation_time_ms=rotation_ms,
                    stall_events=[],
                )
            else:
                rot_result = self.hub.rotate_with_stall_detection(
                    rotation_deg, self.cfg.motion, self.cfg.stall,
                    on_progress=on_progress,
                    on_stall=on_stall,
                    abort=self.abandon,
                )

            if self.abandon.is_set():
                break

            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_ROTATION_REACHED,
                "design_file": self.design_path.name,
                "bar_id": self.bar_id,
                "joint_id": joint_id, "joint_type": joint_type_raw,
                "joint_subtype": joint_subtype,
                "ori": joint_ori, "joint_seq": ji,
                "target_deg": rotation_deg,
                "actual_deg": rot_result.actual_deg,
                "tol_deg": self.cfg.motion.rot_tol_deg,
                "rot_speed_rpm": self.cfg.motion.rot_speed_rpm,
                "rot_acc": self.cfg.motion.rot_acc,
                "rotation_time_ms": rot_result.rotation_time_ms,
                "motor_reached_flag": rot_result.reached,
            })

            self._post("rotation_done", reached=rot_result.reached,
                        actual_deg=rot_result.actual_deg,
                        rotation_time_ms=rot_result.rotation_time_ms)

            # ── INSTALL JOINT ────────────────────────────────────
            self._post("step", phase="install", joint_index=ji,
                        joint_type=jtype_key,
                        joint_type_display=f"{joint_type_raw} {joint_subtype}".strip(),
                        joint_ori=joint_ori)
            self.confirm_install.clear()
            install_start = time.perf_counter()

            while not self.confirm_install.is_set() and not self.abandon.is_set():
                time.sleep(0.1)

            if self.abandon.is_set():
                break

            install_ms = int((time.perf_counter() - install_start) * 1000)
            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_INSTALL_CONFIRM,
                "design_file": self.design_path.name,
                "bar_id": self.bar_id,
                "joint_id": joint_id, "joint_type": joint_type_raw,
                "joint_subtype": joint_subtype,
                "ori": joint_ori, "joint_seq": ji,
                "install_time_ms": install_ms,
            })
            joints_completed += 1

        # ── Bar finished (complete or abandoned) ─────────────────
        bar_ms = int((time.perf_counter() - bar_start) * 1000)

        if self.abandon.is_set():
            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_BAR_ABANDONED,
                "design_file": self.design_path.name,
                "bar_id": self.bar_id,
                "bar_time_ms": bar_ms,
                "joints_completed": joints_completed,
                "joints_total": len(self.joints),
            })
            self._post("done", abandoned=True, bar_id=self.bar_id,
                        bar_time_ms=bar_ms)
        else:
            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_BAR_INSTALL_COMPLETE,
                "design_file": self.design_path.name,
                "bar_id": self.bar_id,
                "bar_time_ms": bar_ms,
            })
            self._post("done", abandoned=False, bar_id=self.bar_id,
                        bar_time_ms=bar_ms)


# ═══════════════════════════════════════════════════════════════
#  Main application window
# ═══════════════════════════════════════════════════════════════

class JigApp(tk.Tk):
    """Top-level Tk window for the joint jig controller."""

    def __init__(self) -> None:
        super().__init__()
        self.title(WINDOW_TITLE)
        self.minsize(*WINDOW_MIN_SIZE)

        # ── Shared state ─────────────────────────────────────────
        self.base_dir = Path(__file__).resolve().parent
        self.settings_path = self.base_dir / "settings.json"
        self.raw_settings: dict = {}
        self.cfg: Optional[JigSettings] = None
        self.hub: Optional[HardwareHub] = None  # replaces hw + bus_lock

        # Design / log state
        self.design_path: Optional[Path] = None
        self.design: Optional[dict] = None
        self.bars: List[dict] = []
        self.log_path: Optional[Path] = None
        self.offset_map: Dict = {}

        # Queue for assembly worker → GUI messages
        self.assembly_queue: queue.Queue = queue.Queue()

        # Assembly thread control events
        self.abandon_event = threading.Event()
        self.confirm_linear_event = threading.Event()
        self.confirm_install_event = threading.Event()
        self.stall_retry_event = threading.Event()
        self.assembly_worker: Optional[AssemblyWorker] = None

        # Latest linear error for tolerance check
        self._last_linear_error: float = 999.0

        # Label-printer state
        self._label_printer_present: bool = False
        self._label_print_in_progress: bool = False
        self._selection_label_print_in_progress: bool = False
        self.simulate_mode_var = tk.BooleanVar(value=False)
        self._sim_motor_display_deg: float = 0.0
        self._jog_up_button_held: bool = False
        self._jog_down_button_held: bool = False
        self._jog_up_key_held: bool = False
        self._jog_down_key_held: bool = False
        self._jog_direction: int = 0
        self._jog_target_deg: Optional[float] = None
        self._jog_state_lock = threading.Lock()
        self._jog_stop_event = threading.Event()
        self._jog_thread = threading.Thread(
            target=self._jog_loop,
            daemon=True,
            name="motor-jog-loop",
        )
        self._jog_thread.start()
        # Cached info about the bar currently being assembled, captured
        # at start so we can print a label after completion.
        self._current_bar_id: str = ""
        self._current_bar_seq: int = 0
        self._current_bar_length_mm: float = 0.0

        # ── Layout ───────────────────────────────────────────────
        self._build_ui()

        # ── Load settings + connect hardware ─────────────────────
        self.after(50, self._startup)

    # ─────────────────────────────────────────────────────────────
    #  UI construction
    # ─────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        """Create all widgets.  Left panel frames are built but only
        the selection frame is initially visible."""

        # Top-level horizontal split
        self.left_container = tk.Frame(self, bg="#f0f0f0")
        self.left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.right_panel = tk.Frame(self, width=280, bg="#e8e8e8",
                                     relief=tk.GROOVE, borderwidth=1)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.Y)
        self.right_panel.pack_propagate(False)

        # ── Selection screen (Screen A) ──────────────────────────
        self.selection_frame = tk.Frame(self.left_container, bg="#f0f0f0")

        # Design file row
        file_row = tk.Frame(self.selection_frame, bg="#f0f0f0")
        file_row.pack(fill=tk.X, padx=10, pady=(10, 5))
        tk.Label(file_row, text="Design File:", bg="#f0f0f0",
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.design_file_var = tk.StringVar(value="(none)")
        tk.Label(file_row, textvariable=self.design_file_var, bg="#f0f0f0",
                 font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=(5, 10))
        tk.Button(file_row, text="Open...", command=self._on_open_design,
                  font=("Segoe UI", 9)).pack(side=tk.LEFT)

        # Bar table
        table_frame = tk.Frame(self.selection_frame, bg="#f0f0f0")
        table_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        columns = ("id", "length", "joints", "status")
        self.bar_tree = ttk.Treeview(table_frame, columns=columns,
                                       show="headings", selectmode="browse")
        self.bar_tree.heading("id", text="Bar ID",
                                command=lambda: self._sort_tree("id"))
        self.bar_tree.heading("length", text="Length (mm)",
                                command=lambda: self._sort_tree("length"))
        self.bar_tree.heading("joints", text="Joints",
                                command=lambda: self._sort_tree("joints"))
        self.bar_tree.heading("status", text="Last Assembled",
                                command=lambda: self._sort_tree("status"))
        self.bar_tree.column("id", width=100)
        self.bar_tree.column("length", width=100, anchor=tk.E)
        self.bar_tree.column("joints", width=70, anchor=tk.E)
        self.bar_tree.column("status", width=250)

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL,
                                   command=self.bar_tree.yview)
        self.bar_tree.configure(yscrollcommand=scrollbar.set)
        self.bar_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.bar_tree.bind("<<TreeviewSelect>>", self._on_bar_select)

        # Bar details text (shown below table when a row is selected)
        self.detail_label = tk.Label(
            self.selection_frame, text="", bg="#f0f0f0", justify=tk.LEFT,
            font=("Consolas", 9), anchor=tk.NW, wraplength=600,
        )
        self.detail_label.pack(fill=tk.X, padx=10, pady=(0, 5))

        # Primary selection-screen actions
        action_row = tk.Frame(self.selection_frame, bg="#f0f0f0")
        action_row.pack(pady=(5, 10))

        self.start_btn = tk.Button(
            action_row, text="\u25b6  Start Assembly",
            font=("Segoe UI", 12, "bold"), state=tk.DISABLED,
            command=self._on_start_assembly, bg="#4CAF50", fg="white",
            activebackground="#45a049", padx=20, pady=8,
        )
        self.start_btn.pack(side=tk.LEFT, padx=(0, 8))

        self.print_label_btn = tk.Button(
            action_row, text="Print Label",
            font=("Segoe UI", 11), state=tk.DISABLED,
            command=self._on_print_label_from_selection,
            bg="#1976D2", fg="white", activebackground="#1565C0",
            padx=16, pady=8,
        )
        self.print_label_btn.pack(side=tk.LEFT)

        # Show selection by default
        self.selection_frame.pack(fill=tk.BOTH, expand=True)

        # ── Assembly wizard (Screen B) ───────────────────────────
        self.assembly_frame = tk.Frame(self.left_container, bg="#f0f0f0")

        # Top bar: bar/joint info + abandon button
        top_bar = tk.Frame(self.assembly_frame, bg="#ddd")
        top_bar.pack(fill=tk.X)

        # Title uses a frame with separate labels so only IDs are bold
        title_frame = tk.Frame(top_bar, bg="#ddd")
        title_frame.pack(side=tk.LEFT, padx=10, pady=5)
        self._title_prefix = tk.Label(title_frame, text="Bar: ", bg="#ddd",
                                       font=("Segoe UI", 12))
        self._title_prefix.pack(side=tk.LEFT)
        self._title_bar_id = tk.Label(title_frame, text="", bg="#ddd",
                                        font=("Segoe UI", 12, "bold"))
        self._title_bar_id.pack(side=tk.LEFT)
        self._title_mid = tk.Label(title_frame, text="", bg="#ddd",
                                    font=("Segoe UI", 12))
        self._title_mid.pack(side=tk.LEFT)
        self._title_joint_id = tk.Label(title_frame, text="", bg="#ddd",
                                         font=("Segoe UI", 12, "bold"))
        self._title_joint_id.pack(side=tk.LEFT)

        self.abandon_btn = tk.Button(
            top_bar, text="✕ Abandon Bar", fg="red",
            font=("Segoe UI", 10), command=self._on_abandon,
        )
        self.abandon_btn.pack(side=tk.RIGHT, padx=10, pady=5)

        # Joint image — placed right below the title, visible across
        # all steps (linear → rotation → install) for the current joint
        self.image_label = tk.Label(self.assembly_frame, bg="#f0f0f0")
        self.image_label.pack(pady=(5, 0))
        self._current_photo = None  # prevent garbage collection

        # Instruction label (large, central)
        self.instruction_var = tk.StringVar(value="")
        tk.Label(self.assembly_frame, textvariable=self.instruction_var,
                 bg="#f0f0f0", font=("Segoe UI", 14),
                 wraplength=550, justify=tk.CENTER).pack(pady=(10, 5))

        # Linear position colour bar
        self.bar_frame = tk.Frame(self.assembly_frame, bg="#f0f0f0")
        self.bar_frame.pack(fill=tk.X, padx=30, pady=5)
        self.bar_canvas = tk.Canvas(self.bar_frame, height=40, bg="white",
                                     highlightthickness=1,
                                     highlightbackground="#ccc")
        self.bar_canvas.pack(fill=tk.X)
        self.linear_info_var = tk.StringVar(value="")
        tk.Label(self.assembly_frame, textvariable=self.linear_info_var,
                 bg="#f0f0f0", font=("Consolas", 11)).pack()

        # Rotation progress bar (ttk.Progressbar)
        self.rot_progress_frame = tk.Frame(self.assembly_frame, bg="#f0f0f0")
        self.rot_progress_frame.pack(fill=tk.X, padx=30, pady=5)
        self.rot_progress = ttk.Progressbar(
            self.rot_progress_frame, orient=tk.HORIZONTAL,
            mode="determinate", maximum=100,
        )
        self.rot_progress.pack(fill=tk.X)
        self.rot_progress_info_var = tk.StringVar(value="")
        tk.Label(self.assembly_frame, textvariable=self.rot_progress_info_var,
                 bg="#f0f0f0", font=("Consolas", 11)).pack()

        # Confirm button (used for linear confirm, install confirm, stall retry)
        self.confirm_btn = tk.Button(
            self.assembly_frame, text="Confirm", state=tk.DISABLED,
            font=("Segoe UI", 12, "bold"), command=self._on_confirm,
            bg="#2196F3", fg="white", activebackground="#1976D2",
            padx=20, pady=6,
        )
        self.confirm_btn.pack(pady=10)
        # Bind Enter key globally for confirm
        self.bind("<Return>", lambda e: self._on_confirm())
        self.bind("<KeyPress-Up>", self._on_jog_keypress_up)
        self.bind("<KeyRelease-Up>", self._on_jog_keyrelease_up)
        self.bind("<KeyPress-Down>", self._on_jog_keypress_down)
        self.bind("<KeyRelease-Down>", self._on_jog_keyrelease_down)

        # Status text area (rotation result, stall messages, etc.)
        self.status_var = tk.StringVar(value="")
        tk.Label(self.assembly_frame, textvariable=self.status_var,
                 bg="#f0f0f0", font=("Segoe UI", 10),
                 wraplength=550, justify=tk.CENTER).pack(pady=5)

        # Motor jog controls (shown only during linear positioning step).
        self.jog_frame = tk.Frame(self.assembly_frame, bg="#f0f0f0")
        tk.Label(
            self.jog_frame,
            text="Motor Jog (hold button or Up/Down key)",
            bg="#f0f0f0",
            font=("Segoe UI", 9, "bold"),
            fg="#555",
        ).pack()
        jog_btn_row = tk.Frame(self.jog_frame, bg="#f0f0f0")
        jog_btn_row.pack(pady=(4, 0))
        self.jog_down_btn = tk.Button(
            jog_btn_row, text="\u25bc Jog -", font=("Segoe UI", 10),
            padx=14, pady=4,
        )
        self.jog_down_btn.pack(side=tk.LEFT, padx=(0, 8))
        self.jog_up_btn = tk.Button(
            jog_btn_row, text="\u25b2 Jog +", font=("Segoe UI", 10),
            padx=14, pady=4,
        )
        self.jog_up_btn.pack(side=tk.LEFT)
        self.jog_down_btn.bind("<ButtonPress-1>", self._on_jog_btn_press_down)
        self.jog_down_btn.bind("<ButtonRelease-1>", self._on_jog_btn_release_down)
        self.jog_down_btn.bind("<Leave>", self._on_jog_btn_release_down)
        self.jog_up_btn.bind("<ButtonPress-1>", self._on_jog_btn_press_up)
        self.jog_up_btn.bind("<ButtonRelease-1>", self._on_jog_btn_release_up)
        self.jog_up_btn.bind("<Leave>", self._on_jog_btn_release_up)

        # ── Engineer panel (right side, always visible) ──────────
        tk.Label(self.right_panel, text="Engineer Panel", bg="#e8e8e8",
                 font=("Segoe UI", 10, "bold")).pack(pady=(10, 5))
        ttk.Separator(self.right_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=5)

        # Sensor reading
        self._add_eng_label("Distance Sensor")
        self.eng_sensor_var = tk.StringVar(value="— mm")
        tk.Label(self.right_panel, textvariable=self.eng_sensor_var,
                 bg="#e8e8e8", font=("Consolas", 12)).pack(anchor=tk.W, padx=10)

        # Motor position
        self._add_eng_label("Motor Position")
        self.eng_motor_var = tk.StringVar(value="— °")
        tk.Label(self.right_panel, textvariable=self.eng_motor_var,
                 bg="#e8e8e8", font=("Consolas", 12)).pack(anchor=tk.W, padx=10)

        # Status flags
        self._add_eng_label("Status Flags")
        self.eng_flags_var = tk.StringVar(value="—")
        tk.Label(self.right_panel, textvariable=self.eng_flags_var,
                 bg="#e8e8e8", font=("Consolas", 9)).pack(anchor=tk.W, padx=10)

        # Stall config
        self._add_eng_label("Stall Config")
        self.eng_stall_var = tk.StringVar(value="—")
        tk.Label(self.right_panel, textvariable=self.eng_stall_var,
                 bg="#e8e8e8", font=("Consolas", 9)).pack(anchor=tk.W, padx=10)

        # Connection info
        self._add_eng_label("Connection")
        self.eng_conn_var = tk.StringVar(value="Not connected")
        tk.Label(self.right_panel, textvariable=self.eng_conn_var,
                 bg="#e8e8e8", font=("Consolas", 9)).pack(anchor=tk.W, padx=10)

        # Reconnect button — hidden by default, shown only on disconnect
        self.reconnect_btn = tk.Button(
            self.right_panel, text="Reconnect", command=self._on_reconnect,
            bg="#FF9800", activebackground="#F57C00", fg="white",
            font=("Segoe UI", 9, "bold"), padx=10, pady=2,
        )
        # Not packed yet — shown only when connection is lost

        # Engineer-only simulation toggle.
        self._add_eng_label("Simulation")
        self.sim_mode_chk = tk.Checkbutton(
            self.right_panel,
            text="Enable simulation mode",
            variable=self.simulate_mode_var,
            command=self._on_simulation_mode_toggled,
            bg="#e8e8e8",
            activebackground="#e8e8e8",
            font=("Segoe UI", 9),
        )
        self.sim_mode_chk.pack(anchor=tk.W, padx=10)

        # Motion settings display
        self._add_eng_label("Motion Settings")
        self.eng_motion_var = tk.StringVar(value="—")
        tk.Label(self.right_panel, textvariable=self.eng_motion_var,
                 bg="#e8e8e8", font=("Consolas", 9)).pack(anchor=tk.W, padx=10)

        # Label-printer status (auto-detected over USB)
        self._add_eng_label("Label Printer")
        self.eng_label_printer_var = tk.StringVar(value="Detecting...")
        self.eng_label_printer_lbl = tk.Label(
            self.right_panel, textvariable=self.eng_label_printer_var,
            bg="#e8e8e8", font=("Consolas", 9), fg="#555",
        )
        self.eng_label_printer_lbl.pack(anchor=tk.W, padx=10)

    def _is_simulation_mode(self) -> bool:
        return bool(self.simulate_mode_var.get())

    def _can_jog_now(self) -> bool:
        return getattr(self, "_confirm_action", None) == "linear"

    def _on_jog_btn_press_up(self, _event=None) -> None:
        self._jog_up_button_held = True
        self._recompute_jog_direction()

    def _on_jog_btn_release_up(self, _event=None) -> None:
        self._jog_up_button_held = False
        self._recompute_jog_direction()

    def _on_jog_btn_press_down(self, _event=None) -> None:
        self._jog_down_button_held = True
        self._recompute_jog_direction()

    def _on_jog_btn_release_down(self, _event=None) -> None:
        self._jog_down_button_held = False
        self._recompute_jog_direction()

    def _on_jog_keypress_up(self, _event=None) -> None:
        self._jog_up_key_held = True
        self._recompute_jog_direction()

    def _on_jog_keyrelease_up(self, _event=None) -> None:
        self._jog_up_key_held = False
        self._recompute_jog_direction()

    def _on_jog_keypress_down(self, _event=None) -> None:
        self._jog_down_key_held = True
        self._recompute_jog_direction()

    def _on_jog_keyrelease_down(self, _event=None) -> None:
        self._jog_down_key_held = False
        self._recompute_jog_direction()

    def _recompute_jog_direction(self) -> None:
        up_active = self._jog_up_button_held or self._jog_up_key_held
        down_active = self._jog_down_button_held or self._jog_down_key_held
        direction = (1 if up_active else 0) + (-1 if down_active else 0)
        if not self._can_jog_now():
            direction = 0
        with self._jog_state_lock:
            self._jog_direction = direction

    def _stop_jog_inputs(self) -> None:
        self._jog_up_button_held = False
        self._jog_down_button_held = False
        self._jog_up_key_held = False
        self._jog_down_key_held = False
        with self._jog_state_lock:
            self._jog_direction = 0
        self._jog_target_deg = None

    def _show_jog_controls(self, show: bool) -> None:
        if show:
            if not self.jog_frame.winfo_ismapped():
                self.jog_frame.pack(pady=(0, 8))
        else:
            if self.jog_frame.winfo_ismapped():
                self.jog_frame.pack_forget()
            self._stop_jog_inputs()

    def _jog_loop(self) -> None:
        while not self._jog_stop_event.is_set():
            with self._jog_state_lock:
                direction = self._jog_direction
            if direction == 0 or not self._can_jog_now():
                self._jog_target_deg = None
                time.sleep(JOG_INTERVAL_S)
                continue

            try:
                if self._is_simulation_mode():
                    self._sim_motor_display_deg += direction * JOG_STEP_DEG
                    if (
                        self.assembly_worker is not None
                        and getattr(self.assembly_worker, "simulate_mode", False)
                    ):
                        self.assembly_worker._sim_motor_deg = self._sim_motor_display_deg
                elif self.hub is not None and self.cfg is not None:
                    if self._jog_target_deg is None:
                        base = self.hub.motor_deg
                        if base is None:
                            base = self.hub.read_motor_deg_sync()
                        self._jog_target_deg = float(base)
                    self._jog_target_deg += direction * JOG_STEP_DEG
                    self.hub.send_move_absolute(
                        self._jog_target_deg,
                        speed_rpm=self.cfg.motion.rot_speed_rpm,
                        acc=self.cfg.motion.rot_acc,
                    )
            except Exception:
                pass

            time.sleep(JOG_INTERVAL_S)

    def _on_simulation_mode_toggled(self) -> None:
        """Handle simulation mode toggle from the engineer panel."""
        if self._is_simulation_mode():
            if self.hub and self.hub.connected:
                self.eng_conn_var.set(
                    f"Port: {self.hub.port}\n"
                    "Status: Connected (SIMULATION BYPASS)"
                )
            else:
                self.eng_conn_var.set("Status: SIMULATION (no hardware required)")
            self.eng_flags_var.set("SIMULATION")
            self.eng_motor_var.set(f"{self._sim_motor_display_deg:.3f}°")
            self.status_var.set(
                "Simulation mode enabled: hardware motion and label "
                "printing are virtual."
            )
        else:
            self.status_var.set("")
            self.eng_flags_var.set("—")
            if self.hub and self.hub.connected:
                self.eng_conn_var.set(
                    f"Port: {self.hub.port}\n"
                    "Status: Connected"
                )
            elif not self.hub:
                self.eng_conn_var.set("DISCONNECTED")
        self._refresh_label_status_text()
        self._update_selection_action_buttons()

    def _add_eng_label(self, text: str) -> None:
        """Add a small section header in the engineer panel."""
        tk.Label(self.right_panel, text=text, bg="#e8e8e8",
                 font=("Segoe UI", 8, "bold"), fg="#555").pack(
            anchor=tk.W, padx=10, pady=(8, 0))

    # ─────────────────────────────────────────────────────────────
    #  Startup sequence
    # ─────────────────────────────────────────────────────────────

    def _startup(self) -> None:
        """Load settings, connect hardware, create HardwareHub."""
        if not self.settings_path.exists():
            messagebox.showerror("Error",
                f"settings.json not found:\n{self.settings_path}")
            self.destroy()
            return

        self.raw_settings = load_json(self.settings_path)
        self.cfg = JigSettings.from_dict(self.raw_settings)

        # Update engineer panel with static settings info
        self.eng_stall_var.set(
            f"speed < {self.cfg.stall.speed_rpm} RPM\n"
            f"current > {self.cfg.stall.current_ma} mA\n"
            f"time > {self.cfg.stall.time_ms} ms\n"
            f"enabled: {self.cfg.stall.enabled}"
        )
        self.eng_motion_var.set(
            f"speed: {self.cfg.motion.rot_speed_rpm} RPM\n"
            f"acc: {self.cfg.motion.rot_acc}\n"
            f"tol: {self.cfg.motion.rot_tol_deg}°"
        )

        # Connect hardware (may block briefly)
        self.eng_conn_var.set("Connecting...")
        self.update_idletasks()
        try:
            hw = connect_hardware(self.cfg, log=lambda *a, **kw: None)
            # Wrap the raw hardware in a HardwareHub
            self.hub = HardwareHub(hw, poll_interval_s=POLL_INTERVAL_S)
            self.eng_conn_var.set(
                f"Port: {self.hub.port}\n"
                f"Status: Connected"
            )
        except RuntimeError as e:
            # Keep startup non-blocking when hardware is missing.
            self.eng_conn_var.set("DISCONNECTED")
            self.eng_sensor_var.set("DISCONNECTED")
            self.reconnect_btn.pack(anchor=tk.W, padx=10, pady=(4, 0))

        # Start the GUI refresh loop (reads from hub + drains queues)
        self.after(GUI_DRAIN_MS, self._gui_tick)

        # Start the label-printer USB presence poller (hot-plug)
        self.after(200, self._label_printer_tick)

    # ─────────────────────────────────────────────────────────────
    #  Reconnect
    # ─────────────────────────────────────────────────────────────

    def _on_reconnect(self) -> None:
        """Attempt to reconnect to the hardware."""
        self.reconnect_btn.configure(state=tk.DISABLED, text="Reconnecting...")
        self.eng_conn_var.set("Reconnecting...")
        self.update_idletasks()

        def _do_reconnect() -> None:
            try:
                if self.hub:
                    self.hub.reconnect(self.cfg)
                else:
                    hw = connect_hardware(self.cfg, log=lambda *a, **kw: None)
                    self.hub = HardwareHub(hw, poll_interval_s=POLL_INTERVAL_S)
                self.after(0, self._reconnect_success)
            except Exception as e:
                self.after(0, lambda: self._reconnect_failure(str(e)))

        threading.Thread(target=_do_reconnect, daemon=True).start()

    def _reconnect_success(self) -> None:
        """Called on the main thread after a successful reconnect."""
        self.eng_conn_var.set(
            f"Port: {self.hub.port}\n"
            f"Status: Connected"
        )
        self.reconnect_btn.pack_forget()
        self.reconnect_btn.configure(state=tk.NORMAL, text="Reconnect")

    def _reconnect_failure(self, error: str) -> None:
        """Called on the main thread after a failed reconnect."""
        self.eng_conn_var.set("DISCONNECTED")
        self.reconnect_btn.configure(state=tk.NORMAL, text="Reconnect")
        messagebox.showerror("Reconnect Failed", error)

    # ─────────────────────────────────────────────────────────────
    #  GUI tick — periodic refresh (replaces separate poll/drain)
    # ─────────────────────────────────────────────────────────────

    def _gui_tick(self) -> None:
        """Called every GUI_DRAIN_MS.  Updates engineer panel from hub
        cached values and drains the assembly message queue."""

        # ── Refresh engineer panel from hub ──────────────────────
        if self._is_simulation_mode():
            self.eng_flags_var.set("SIMULATION")
            self.eng_motor_var.set(f"{self._sim_motor_display_deg:.3f}°")
            if self.hub and self.hub.connected:
                self.eng_conn_var.set(
                    f"Port: {self.hub.port}\n"
                    "Status: Connected (SIMULATION BYPASS)"
                )
            else:
                self.eng_conn_var.set("Status: SIMULATION (no hardware required)")
        elif self.hub:
            if not self.hub.connected:
                self.eng_sensor_var.set("DISCONNECTED")
                self.eng_conn_var.set("DISCONNECTED")
                if not self.reconnect_btn.winfo_ismapped():
                    self.reconnect_btn.pack(anchor=tk.W, padx=10, pady=(4, 0))
            elif self.hub.poll_error:
                self.eng_sensor_var.set(f"ERR: {self.hub.poll_error[:40]}")
            else:
                if self.hub.sensor_mm is not None:
                    self.eng_sensor_var.set(f"{self.hub.sensor_mm} mm")
                if self.hub.motor_deg is not None:
                    self.eng_motor_var.set(f"{self.hub.motor_deg:.3f}°")
                flag_parts: List[str] = []
                if self.hub.enabled:
                    flag_parts.append("ENABLED")
                if self.hub.reached:
                    flag_parts.append("REACHED")
                if self.hub.stall_detected:
                    flag_parts.append("STALL_DET")
                if self.hub.stall_protected:
                    flag_parts.append("STALL_PROT")
                self.eng_flags_var.set(
                    "  ".join(flag_parts) if flag_parts else "(none)"
                )

        # ── Drain assembly messages ──────────────────────────────
        while not self.assembly_queue.empty():
            try:
                msg: AssemblyMsg = self.assembly_queue.get_nowait()
                self._handle_assembly_msg(msg)
            except queue.Empty:
                break

        self.after(GUI_DRAIN_MS, self._gui_tick)

    # ─────────────────────────────────────────────────────────────
    #  Label-printer hot-plug detection
    # ─────────────────────────────────────────────────────────────

    def _label_printer_tick(self) -> None:
        """Periodically check whether the Brother label printer is on USB.

        Runs even while a print job is in progress — only updates the
        engineer-panel label and an internal flag; the done-page status
        text is refreshed from this flag the next time it is shown.
        """
        if self._is_simulation_mode():
            self._label_printer_present = True
            self.eng_label_printer_var.set("SIMULATED")
            self.eng_label_printer_lbl.configure(fg="#2e7d32")
        elif not label_printer.is_dependencies_available():
            self._label_printer_present = False
            err = label_printer.import_error_message() or "unavailable"
            short = err.split(":")[0]
            self.eng_label_printer_var.set(f"DISABLED ({short})")
            self.eng_label_printer_lbl.configure(fg="#888")
        else:
            present = label_printer.is_printer_present()
            if present != self._label_printer_present:
                self._label_printer_present = present
                # If the done page is currently showing the "printer
                # missing" state and the printer just appeared, refresh.
                if (getattr(self, "_confirm_action", None) ==
                        "label_print"):
                    self._refresh_label_status_text()
            if present:
                self.eng_label_printer_var.set("Connected (Brother PT)")
                self.eng_label_printer_lbl.configure(fg="#2e7d32")
            else:
                self.eng_label_printer_var.set("Not detected")
                self.eng_label_printer_lbl.configure(fg="#b71c1c")

        self._update_selection_action_buttons()

        self.after(LABEL_PRINTER_POLL_MS, self._label_printer_tick)

    # ─────────────────────────────────────────────────────────────
    #  Assembly message handler
    # ─────────────────────────────────────────────────────────────

    def _handle_assembly_msg(self, msg: AssemblyMsg) -> None:
        d = msg.data

        if msg.kind == "joint_start":
            ji = d["joint_index"]
            jt = d["joint_total"]
            jid = d["joint_id"]
            jtype = d["joint_type"]         # concatenated key, e.g. "T20_Female"
            jtype_display = d.get("joint_type_display", jtype)
            jori = d["joint_ori"]

            # Update title: only bar ID and joint ID are bold
            self._title_prefix.configure(text="Bar: ")
            self._title_bar_id.configure(text=self._current_bar_id)
            self._title_mid.configure(
                text=f"  —  Joint {ji+1}/{jt}  (")
            # We add the closing ')' by appending to joint_id for simplicity
            self._title_joint_id.configure(text=f"{jid})")

            # Load joint image immediately — stays visible across all steps
            self._load_joint_image(jtype, jori)

        elif msg.kind == "step":
            phase = d["phase"]
            if phase == "linear":
                self.instruction_var.set(
                    "Move the carriage to the target position.\n"
                    "Press Confirm (or Enter) when in tolerance."
                )
                self.confirm_btn.configure(state=tk.NORMAL,
                                           text="Confirm Position")
                self._confirm_action = "linear"
                self.bar_canvas.delete("all")
                self.linear_info_var.set("")
                self.status_var.set("")
                # Show linear bar, hide rotation progress bar
                self.bar_frame.pack(fill=tk.X, padx=30, pady=5)
                self.rot_progress_frame.pack_forget()
                self.rot_progress_info_var.set("")
                self._show_jog_controls(True)

            elif phase == "rotating":
                target_deg = d.get("target_deg", 0.0)
                self.instruction_var.set(
                    f"Rotating motor to {target_deg:.1f}°...")
                self.confirm_btn.configure(state=tk.DISABLED)
                self._confirm_action = None
                self.linear_info_var.set("")
                self.bar_canvas.delete("all")
                # Hide linear bar, show rotation progress bar
                self.bar_frame.pack_forget()
                self._show_jog_controls(False)
                self.rot_progress_frame.pack(fill=tk.X, padx=30, pady=5)
                self.rot_progress["value"] = 0
                self.rot_progress_info_var.set("")
                # Store target for progress calculation
                self._rotation_start_deg = (
                    self.hub.motor_deg
                    if self.hub and self.hub.motor_deg is not None
                    else self._sim_motor_display_deg
                )
                self._rotation_target_deg = target_deg

            elif phase == "install":
                jtype_display = d.get("joint_type_display", d.get("joint_type", ""))
                jori = d.get("joint_ori", "")
                self.instruction_var.set(
                    f"Install joint now:\n"
                    f"Type: {jtype_display}   Orientation: {jori}\n\n"
                    f"Press Confirm (or Enter) when installed."
                )
                self.confirm_btn.configure(state=tk.NORMAL,
                                           text="Joint Installed")
                self._confirm_action = "install"
                self.linear_info_var.set("")
                # Hide both bars during install
                self.bar_frame.pack_forget()
                self._show_jog_controls(False)
                self.rot_progress_frame.pack_forget()
                self.rot_progress_info_var.set("")

        elif msg.kind == "linear_update":
            error = d["error_mm"]
            target = d["target_mm"]
            measured = d["measured_mm"]
            raw = d["raw_mm"]
            self._last_linear_error = error
            tol = self.cfg.linear.linear_tol_mm

            self.linear_info_var.set(
                f"raw={raw}  measured={measured:.1f}  "
                f"target={target:.1f}  error={error:+.1f} mm  "
                f"(tol ±{tol})"
            )
            self._draw_linear_bar(error, tol)

        elif msg.kind == "rotation_progress":
            actual = d["actual_deg"]
            target = d["target_deg"]
            if self._is_simulation_mode():
                self._sim_motor_display_deg = float(actual)
            start = getattr(self, "_rotation_start_deg", 0.0)
            total_span = abs(target - start)
            if total_span > 0.001:
                done_span = abs(actual - start)
                pct = min(100.0, (done_span / total_span) * 100)
            else:
                pct = 100.0
            self.rot_progress["value"] = pct
            self.rot_progress_info_var.set(
                f"{actual:.2f}° / {target:.1f}°  ({pct:.0f}%)"
            )

        elif msg.kind == "rotation_done":
            actual = d["actual_deg"]
            reached = d["reached"]
            ms = d["rotation_time_ms"]
            if self._is_simulation_mode():
                self._sim_motor_display_deg = float(actual)
            self.rot_progress["value"] = 100
            self.rot_progress_info_var.set("")
            self.status_var.set(
                f"Rotation complete: {actual:.2f}°  "
                f"reached={reached}  time={ms}ms"
            )

        elif msg.kind == "stall":
            actual = d["actual_deg"]
            target = d["target_deg"]
            self.instruction_var.set(
                f"⚠  STALL DETECTED\n"
                f"Position: {actual:.2f}° (target: {target:.2f}°)\n\n"
                "Clear the obstruction, then press Confirm to retry."
            )
            self.confirm_btn.configure(state=tk.NORMAL,
                                       text="Retry Rotation")
            self._confirm_action = "stall_retry"

        elif msg.kind == "done":
            abandoned = d.get("abandoned", False)
            bar_ms = d.get("bar_time_ms", 0)
            bid = d.get("bar_id", "")
            # Show an in-wizard completion page instead of a popup
            self._show_done_page(bid, bar_ms, abandoned)

        elif msg.kind == "error":
            messagebox.showerror("Assembly Error",
                                 d.get("message", "Unknown"))
            self._show_selection()

    # ─────────────────────────────────────────────────────────────
    #  Bar done / abandoned page (replaces popup dialog)
    # ─────────────────────────────────────────────────────────────

    def _show_done_page(self, bar_id: str, bar_ms: int,
                        abandoned: bool) -> None:
        """Replace the assembly wizard content with a summary page.

        On a successful (non-abandoned) bar, show the label-printing
        sub-step: pressing Enter prints the label, ``Skip`` continues to
        the selection screen, and any printer error / missing-device
        condition is reported inline so the user can retry after
        plugging in the device.
        """

        # Hide all variable assembly widgets
        self.image_label.configure(image="", text="")
        self._current_photo = None
        self.bar_frame.pack_forget()
        self._show_jog_controls(False)
        self.rot_progress_frame.pack_forget()
        self.rot_progress_info_var.set("")
        self.linear_info_var.set("")
        self.confirm_btn.configure(state=tk.DISABLED)
        self._confirm_action = None
        self.abandon_btn.configure(state=tk.DISABLED)

        # Title bar
        self._title_prefix.configure(text="Bar: ")
        self._title_bar_id.configure(text=bar_id)
        self._title_mid.configure(text="")
        self._title_joint_id.configure(text="")

        duration = format_duration_ms(bar_ms)
        if abandoned:
            self.instruction_var.set(
                f"Bar abandoned.\n\n"
                f"Time elapsed: {duration}"
            )
            self.status_var.set("")
            # Just a back button — no label printing for abandoned bars.
            self.confirm_btn.configure(
                state=tk.NORMAL, text="Back to Selection",
                bg="#607D8B", activebackground="#546E7A",
            )
            self._confirm_action = "back_to_selection"
            self._hide_skip_label_btn()
            return

        # Bar completed successfully — offer label printing as a step.
        self.instruction_var.set(
            f"✓  Assembly complete!\n\n"
            f"Total time: {duration}\n\n"
            f"Bar: {bar_id}   Length: {self._current_bar_length_mm:g} mm   "
            f"Seq: #{self._current_bar_seq}"
        )
        self._enter_label_print_step()

    def _enter_label_print_step(self) -> None:
        """Configure the done page in 'ready to print label' state."""
        self.confirm_btn.configure(
            state=tk.NORMAL, text="Print Label  (Enter)",
            bg="#4CAF50", activebackground="#43A047",
        )
        self._confirm_action = "label_print"
        self._show_skip_label_btn()
        self._refresh_label_status_text()

    def _show_skip_label_btn(self) -> None:
        """Lazy-create and show the 'Skip Label' button next to confirm."""
        if not hasattr(self, "skip_label_btn"):
            self.skip_label_btn = tk.Button(
                self.assembly_frame, text="Skip Label",
                font=("Segoe UI", 10), command=self._on_skip_label,
                bg="#9E9E9E", fg="white",
                activebackground="#757575", padx=12, pady=4,
            )
        if not self.skip_label_btn.winfo_ismapped():
            self.skip_label_btn.pack(pady=(0, 8))
        self.skip_label_btn.configure(state=tk.NORMAL)

    def _hide_skip_label_btn(self) -> None:
        if hasattr(self, "skip_label_btn") and self.skip_label_btn.winfo_ismapped():
            self.skip_label_btn.pack_forget()

    def _refresh_label_status_text(self) -> None:
        """Update the done-page sub-status line based on printer state."""
        if self._label_print_in_progress:
            self.status_var.set("Printing label...")
            return
        if self._is_simulation_mode():
            self.status_var.set(
                "Simulation mode: label print is virtual. "
                "Press Enter to simulate print, or Skip Label."
            )
            self.confirm_btn.configure(state=tk.NORMAL)
            return
        if not label_printer.is_dependencies_available():
            err = label_printer.import_error_message() or "unavailable"
            self.status_var.set(
                f"Label printer support is disabled ({err}).\n"
                f"Press Skip Label to continue."
            )
            self.confirm_btn.configure(state=tk.DISABLED)
            return
        if not self._label_printer_present:
            self.status_var.set(
                "⚠  Label printer not detected.\n"
                "Plug in the Brother PT printer and press Enter to retry, "
                "or press Skip Label."
            )
            # Still allow Enter — it will re-check and print if available.
            self.confirm_btn.configure(state=tk.NORMAL)
            return
        self.status_var.set(
            "Label printer ready. Press Enter to print, or Skip Label."
        )
        self.confirm_btn.configure(state=tk.NORMAL)

    def _on_skip_label(self) -> None:
        """Operator chose to skip printing for this bar."""
        if self._label_print_in_progress:
            return
        self._hide_skip_label_btn()
        self.status_var.set("Label skipped.")
        self.confirm_btn.configure(
            state=tk.NORMAL, text="Back to Selection",
            bg="#607D8B", activebackground="#546E7A",
        )
        self._confirm_action = "back_to_selection"

    def _start_label_print(self) -> None:
        """Trigger label printing in a background thread."""
        if self._label_print_in_progress:
            return
        if self._is_simulation_mode():
            self._label_print_in_progress = True
            self.confirm_btn.configure(state=tk.DISABLED, text="Simulating...")
            if hasattr(self, "skip_label_btn"):
                self.skip_label_btn.configure(state=tk.DISABLED)
            self.status_var.set("Simulating label print...")
            self.after(
                250,
                lambda: self._label_print_done(
                    True,
                    {"media_width_mm": "SIM", "tape_color": "virtual"},
                    None,
                ),
            )
            return
        # Re-check presence right before printing in case device was
        # unplugged between the last poll and the click.
        if not label_printer.is_dependencies_available():
            self._refresh_label_status_text()
            return
        if not label_printer.is_printer_present():
            self._label_printer_present = False
            self._refresh_label_status_text()
            return

        self._label_print_in_progress = True
        self.confirm_btn.configure(state=tk.DISABLED, text="Printing...")
        if hasattr(self, "skip_label_btn"):
            self.skip_label_btn.configure(state=tk.DISABLED)
        self.status_var.set("Printing label...")

        bar_id = self._current_bar_id
        length_mm = self._current_bar_length_mm
        seq = self._current_bar_seq

        def _worker() -> None:
            try:
                info = label_printer.print_bar_label(
                    bar_id=bar_id, length_mm=length_mm,
                    assembly_seq=seq,
                )
                self.after(0, lambda: self._label_print_done(True, info, None))
            except Exception as exc:
                err = str(exc)
                self.after(0, lambda: self._label_print_done(False, None, err))

        threading.Thread(target=_worker, daemon=True,
                         name="label-print-worker").start()

    def _label_print_done(self, ok: bool, info: Optional[dict],
                          error: Optional[str]) -> None:
        """Called on the main thread after the print attempt."""
        self._label_print_in_progress = False
        if ok:
            media_w = (info or {}).get("media_width_mm", "?")
            color = (info or {}).get("tape_color", "?")
            self.status_var.set(
                f"✓ Label printed (tape: {media_w} mm {color})."
            )
            self._hide_skip_label_btn()
            self.confirm_btn.configure(
                state=tk.NORMAL, text="Back to Selection",
                bg="#607D8B", activebackground="#546E7A",
            )
            self._confirm_action = "back_to_selection"
        else:
            self.status_var.set(
                f"⚠  Label print failed: {error}\n"
                "Fix the issue (e.g. plug printer in / close cover) and "
                "press Enter to retry, or press Skip Label."
            )
            self.confirm_btn.configure(
                state=tk.NORMAL, text="Retry Print  (Enter)",
                bg="#4CAF50", activebackground="#43A047",
            )
            if hasattr(self, "skip_label_btn"):
                self.skip_label_btn.configure(state=tk.NORMAL)
            self._confirm_action = "label_print"

    # ─────────────────────────────────────────────────────────────
    #  Linear position colour bar drawing
    # ─────────────────────────────────────────────────────────────

    def _draw_linear_bar(self, error: float, tol: float) -> None:
        """Draw a horizontal bar showing position error vs tolerance."""
        self.bar_canvas.delete("all")
        w = self.bar_canvas.winfo_width()
        h = self.bar_canvas.winfo_height()
        if w < 10:
            return

        display_range = 20.0
        centre_x = w / 2

        # Tolerance zone (green background)
        tol_px = (tol / display_range) * (w / 2)
        self.bar_canvas.create_rectangle(
            centre_x - tol_px, 0, centre_x + tol_px, h,
            fill="#c8e6c9", outline="",
        )

        # Centre line
        self.bar_canvas.create_line(centre_x, 0, centre_x, h,
                                     fill="#888", width=1)

        # Error indicator
        clamped = max(-display_range, min(display_range, error))
        indicator_x = centre_x + (clamped / display_range) * (w / 2)
        in_tol = abs(error) <= tol
        colour = "#4CAF50" if in_tol else "#f44336"
        self.bar_canvas.create_rectangle(
            indicator_x - 4, 2, indicator_x + 4, h - 2,
            fill=colour, outline=colour,
        )

    # ─────────────────────────────────────────────────────────────
    #  Joint image loading
    # ─────────────────────────────────────────────────────────────

    def _load_joint_image(self, joint_type: str, joint_ori: str) -> None:
        """Load and display the reference image for a joint.

        Called at ``joint_start`` so the image is visible throughout
        all three steps (linear → rotation → install).
        """
        self._current_photo = None
        self.image_label.configure(image="", text="")

        images_cfg = safe_get(self.raw_settings,
                              ["images", "joint_images"], default=[])
        base_dir = safe_get(self.raw_settings,
                            ["images", "base_dir"], default="images")

        rel_path = None
        for entry in images_cfg:
            if (entry.get("type") == joint_type
                    and entry.get("ori") == joint_ori):
                rel_path = entry.get("rel_path")
                break
        if not rel_path:
            return

        img_path = self.base_dir / base_dir / rel_path
        if not img_path.exists():
            return

        try:
            from PIL import Image, ImageTk
            img = Image.open(img_path)
            img.thumbnail((400, 250))
            photo = ImageTk.PhotoImage(img)
            self.image_label.configure(image=photo)
            self._current_photo = photo
        except ImportError:
            try:
                photo = tk.PhotoImage(file=str(img_path))
                self.image_label.configure(image=photo)
                self._current_photo = photo
            except tk.TclError:
                pass

    # ─────────────────────────────────────────────────────────────
    #  Bar table: sorting
    # ─────────────────────────────────────────────────────────────

    def _sort_tree(self, col: str) -> None:
        """Sort the Treeview by the given column (toggle asc/desc)."""
        items = [(self.bar_tree.set(iid, col), iid)
                 for iid in self.bar_tree.get_children("")]

        try:
            items.sort(key=lambda t: float(t[0]))
        except ValueError:
            items.sort(key=lambda t: t[0])

        current = [iid for _, iid in items]
        existing = list(self.bar_tree.get_children(""))
        if current == existing:
            items.reverse()

        for idx, (_, iid) in enumerate(items):
            self.bar_tree.move(iid, "", idx)

    # ─────────────────────────────────────────────────────────────
    #  Design file selection
    # ─────────────────────────────────────────────────────────────

    def _on_open_design(self) -> None:
        """Open a file dialog to select a design JSON file."""
        initial_dir = get_last_design_dir(self.raw_settings)
        if not initial_dir or not Path(initial_dir).is_dir():
            initial_dir = str(self.base_dir / "design")

        path = filedialog.askopenfilename(
            title="Select Design File",
            initialdir=initial_dir,
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return

        design_path = Path(path).resolve()
        try:
            design = load_json(design_path)
            ensure_schema_version(design, expected=1)
            bars = design.get("bars", [])
            if not isinstance(bars, list) or len(bars) == 0:
                messagebox.showerror("Error",
                    "Design file contains no bars[].")
                return
        except Exception as e:
            messagebox.showerror("Error",
                f"Failed to load design:\n{e}")
            return

        self.design_path = design_path
        self.design = design
        self.bars = bars
        self.log_path = design_log_path(design_path, self.raw_settings)
        self.offset_map = build_offset_map(self.raw_settings)
        self.design_file_var.set(design_path.name)

        set_last_design_dir(self.raw_settings, str(design_path.parent))
        save_settings(self.settings_path, self.raw_settings)

        self._refresh_bar_table()

    def _refresh_bar_table(self) -> None:
        """Rebuild the bar Treeview from the current design + log."""
        self.bar_tree.delete(*self.bar_tree.get_children())
        self.detail_label.configure(text="")
        self._update_selection_action_buttons()

        if not self.bars or not self.log_path:
            return

        events = parse_log_events(self.log_path)
        cmap = get_bar_completion_map(events)

        for idx, bar in enumerate(self.bars):
            bid = bar.get("bar_id", f"bar_{idx}")
            length = bar.get("length_mm", "?")
            nj = len(bar.get("joints", []) or [])
            info = cmap.get(str(bid))
            if info:
                ago = format_time_ago(info.completed_at)
                dur = format_duration_ms(info.bar_time_ms)
                status = f"{ago} ({dur})"
            else:
                status = "—"
            self.bar_tree.insert("", tk.END, iid=str(idx),
                                   values=(bid, length, nj, status))

    def _on_bar_select(self, event) -> None:
        """Called when the user clicks a row in the bar table."""
        sel = self.bar_tree.selection()
        if not sel:
            self.detail_label.configure(text="")
            self._update_selection_action_buttons()
            return

        idx = int(sel[0])
        bar = self.bars[idx]
        bid = bar.get("bar_id", f"bar_{idx}")
        length = bar.get("length_mm", "?")
        joints = bar.get("joints", []) or []

        lines = [f"Bar {bid}:  {length} mm,  {len(joints)} joint(s)\n"]
        for j in sorted(joints,
                         key=lambda x: float(x.get("position_mm", 0))):
            jid = j.get("joint_id", "?")
            jt = j.get("type", "")
            jst = j.get("subtype", "")
            jo = j.get("ori", "?")
            pos = j.get("position_mm", "?")
            rot = j.get("rotation_deg", "?")
            type_display = f"{jt} {jst}".strip()
            lines.append(
                f"  {jid:<8}  {type_display:<16}  ori={jo:<4}  "
                f"pos={pos}mm  rot={rot}°"
            )
        self.detail_label.configure(text="\n".join(lines))
        self._update_selection_action_buttons()

    def _update_selection_action_buttons(self) -> None:
        """Update selection-screen button enable/disable states."""
        sel = self.bar_tree.selection()
        has_selection = bool(sel)

        # Simulation mode allows assembly without real hardware.
        can_start = has_selection and (
            self._is_simulation_mode() or (self.hub is not None)
        )
        self.start_btn.configure(state=tk.NORMAL if can_start else tk.DISABLED)

        # Simulation mode allows virtual label printing.
        can_print = has_selection and not self._selection_label_print_in_progress
        if not self._is_simulation_mode():
            can_print = (
                can_print
                and label_printer.is_dependencies_available()
                and self._label_printer_present
            )
        self.print_label_btn.configure(
            state=tk.NORMAL if can_print else tk.DISABLED,
            text="Print Label" if not self._selection_label_print_in_progress
            else "Printing...",
        )

    def _on_print_label_from_selection(self) -> None:
        """Print a label for the selected bar directly from selection UI."""
        if self._selection_label_print_in_progress:
            return

        sel = self.bar_tree.selection()
        if not sel:
            return

        idx = int(sel[0])
        bar = self.bars[idx]
        bar_id = str(bar.get("bar_id", f"bar_{idx}"))
        seq = idx + 1
        try:
            length_mm = float(bar.get("length_mm", 0.0))
        except (TypeError, ValueError):
            length_mm = 0.0

        if self._is_simulation_mode():
            self._selection_label_print_in_progress = True
            self._update_selection_action_buttons()
            self.after(
                250,
                lambda: self._selection_print_done(
                    True,
                    bar_id,
                    seq,
                    {"media_width_mm": "SIM", "tape_color": "virtual"},
                    None,
                ),
            )
            return

        # Re-check availability right before print in case of hot unplug.
        if not label_printer.is_dependencies_available():
            messagebox.showerror(
                "Label Printer",
                f"Label printer support is unavailable:\n"
                f"{label_printer.import_error_message() or 'unknown error'}",
            )
            self._update_selection_action_buttons()
            return
        if not label_printer.is_printer_present():
            self._label_printer_present = False
            self._update_selection_action_buttons()
            messagebox.showwarning(
                "Label Printer Not Detected",
                "Brother label printer not detected. Plug it in and try again.",
            )
            return

        self._selection_label_print_in_progress = True
        self._update_selection_action_buttons()

        def _worker() -> None:
            try:
                info = label_printer.print_bar_label(
                    bar_id=bar_id,
                    length_mm=length_mm,
                    assembly_seq=seq,
                )
                self.after(0, lambda: self._selection_print_done(
                    True, bar_id, seq, info, None
                ))
            except Exception as exc:
                self.after(0, lambda: self._selection_print_done(
                    False, bar_id, seq, None, str(exc)
                ))

        threading.Thread(
            target=_worker, daemon=True,
            name="selection-label-print-worker",
        ).start()

    def _selection_print_done(
        self,
        ok: bool,
        bar_id: str,
        seq: int,
        info: Optional[dict],
        error: Optional[str],
    ) -> None:
        """Handle completion of selection-screen label print."""
        self._selection_label_print_in_progress = False
        self._update_selection_action_buttons()

        if ok:
            media_w = (info or {}).get("media_width_mm", "?")
            color = (info or {}).get("tape_color", "?")
            messagebox.showinfo(
                "Label Printed",
                f"Printed label for bar {bar_id} (seq #{seq}).\n"
                f"Tape: {media_w} mm {color}",
            )
        else:
            messagebox.showwarning(
                "Label Print Failed",
                f"Could not print label for bar {bar_id} (seq #{seq}).\n\n"
                f"{error}\n\n"
                "Plug in / check the printer and try again.",
            )

    # ─────────────────────────────────────────────────────────────
    #  Screen switching helpers
    # ─────────────────────────────────────────────────────────────

    def _show_selection(self) -> None:
        """Switch the left panel to the selection screen."""
        self._show_jog_controls(False)
        self.assembly_frame.pack_forget()
        self.selection_frame.pack(fill=tk.BOTH, expand=True)
        self._refresh_bar_table()

    def _show_assembly(self) -> None:
        """Switch the left panel to the assembly wizard screen."""
        self.selection_frame.pack_forget()
        self.assembly_frame.pack(fill=tk.BOTH, expand=True)
        self.instruction_var.set("Preparing assembly...")
        self.status_var.set("")
        self.linear_info_var.set("")
        self.bar_canvas.delete("all")
        self.rot_progress["value"] = 0
        self.rot_progress_info_var.set("")
        self.confirm_btn.configure(state=tk.DISABLED)
        self._show_jog_controls(False)
        self._current_photo = None
        self.image_label.configure(image="", text="")
        # Reset title labels
        self._title_bar_id.configure(text="")
        self._title_mid.configure(text="")
        self._title_joint_id.configure(text="")

    # ─────────────────────────────────────────────────────────────
    #  Start assembly
    # ─────────────────────────────────────────────────────────────

    def _on_start_assembly(self) -> None:
        """Validate, then switch to assembly screen and launch worker."""
        sel = self.bar_tree.selection()
        if not sel:
            return
        if not self._is_simulation_mode() and self.hub is None:
            return

        idx = int(sel[0])
        bar = self.bars[idx]
        bar_id = str(bar.get("bar_id", f"bar_{idx}"))

        # Capture bar metadata for the post-assembly label print step.
        try:
            self._current_bar_length_mm = float(bar.get("length_mm", 0.0))
        except (TypeError, ValueError):
            self._current_bar_length_mm = 0.0
        # Use 1-based design-file position as the assembly sequence.
        self._current_bar_seq = idx + 1

        try:
            validate_offsets_complete(bar, self.offset_map)
        except RuntimeError as e:
            messagebox.showerror("Missing Offsets", str(e))
            return

        joints = list(bar.get("joints", []) or [])
        joints.sort(key=lambda j: float(j.get("position_mm", 0.0)))

        # Reset control state
        self.abandon_event.clear()
        self.confirm_linear_event.clear()
        self.confirm_install_event.clear()
        self.stall_retry_event.clear()
        self._confirm_action = None
        self._current_bar_id = bar_id
        self._last_linear_error = 999.0

        self._show_assembly()
        self._title_prefix.configure(text="Bar: ")
        self._title_bar_id.configure(text=bar_id)
        self._title_mid.configure(text="  —  Starting...")
        self._title_joint_id.configure(text="")

        self.assembly_worker = AssemblyWorker(
            hub=self.hub,
            msg_queue=self.assembly_queue,
            bar=bar,
            bar_id=bar_id,
            joints=joints,
            offset_map=self.offset_map,
            cfg=self.cfg,
            design_path=self.design_path,
            log_path=self.log_path,
            raw_settings=self.raw_settings,
            abandon_event=self.abandon_event,
            confirm_linear_event=self.confirm_linear_event,
            confirm_install_event=self.confirm_install_event,
            stall_retry_event=self.stall_retry_event,
            simulate_mode=self._is_simulation_mode(),
        )
        self.assembly_worker.start()

    # ─────────────────────────────────────────────────────────────
    #  Button handlers
    # ─────────────────────────────────────────────────────────────

    def _on_confirm(self) -> None:
        """Handle the multi-purpose Confirm button (or Enter key)."""
        action = getattr(self, "_confirm_action", None)
        if action == "linear":
            if abs(self._last_linear_error) <= self.cfg.linear.linear_tol_mm:
                self.confirm_linear_event.set()
                self.confirm_btn.configure(state=tk.DISABLED)
            else:
                self.status_var.set(
                    "⚠  Outside tolerance — keep adjusting."
                )
        elif action == "install":
            self.confirm_install_event.set()
            self.confirm_btn.configure(state=tk.DISABLED)
        elif action == "stall_retry":
            self.stall_retry_event.set()
            self.confirm_btn.configure(state=tk.DISABLED)
        elif action == "label_print":
            self._start_label_print()
        elif action == "back_to_selection":
            # Restore default confirm button colours before switching
            self.confirm_btn.configure(
                bg="#2196F3", activebackground="#1976D2",
            )
            self.abandon_btn.configure(state=tk.NORMAL)
            self._hide_skip_label_btn()
            self._show_selection()

    def _on_abandon(self) -> None:
        """Signal the assembly worker to stop and return to selection."""
        if messagebox.askyesno("Abandon Bar",
                                "Are you sure you want to abandon this bar?"):
            self.abandon_event.set()
            self.confirm_linear_event.set()
            self.confirm_install_event.set()
            self.stall_retry_event.set()

    # ─────────────────────────────────────────────────────────────
    #  Cleanup
    # ─────────────────────────────────────────────────────────────

    def destroy(self) -> None:
        """Clean up hardware on window close."""
        self.abandon_event.set()
        self.confirm_linear_event.set()
        self.confirm_install_event.set()
        self.stall_retry_event.set()
        self._jog_stop_event.set()
        if self._jog_thread.is_alive():
            self._jog_thread.join(timeout=1.0)
        if self.hub:
            try:
                self.hub.close()
            except Exception:
                pass
        super().destroy()


# ═══════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════

def main() -> None:
    app = JigApp()
    app.mainloop()


if __name__ == "__main__":
    main()
