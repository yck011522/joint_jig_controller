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
  A) Selection screen: design-file picker, tube table, tube details,
     "Start Assembly" button.
  B) Assembly wizard: joint image at top (visible across all steps),
     step-by-step workflow with linear colour bar, rotation progress bar,
     and "Abandon Tube" button.

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

from jig_controller import (
    EVENT_INSTALL_CONFIRM,
    EVENT_LINEAR_CONFIRM,
    EVENT_ROTATION_REACHED,
    EVENT_STALL_DETECTED,
    EVENT_TUBE_ABANDONED,
    EVENT_TUBE_INSTALL_COMPLETE,
    EVENT_TUBE_START,
    HardwareContext,
    HardwareHub,
    JigSettings,
    MotionSettings,
    RotationResult,
    StallSettings,
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
    safe_get,
    save_settings,
    set_last_design_dir,
    validate_offsets_complete,
)


# ═══════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════

POLL_INTERVAL_S = 0.5       # HardwareHub background poll interval
GUI_DRAIN_MS = 100          # how often the GUI drains the message queue
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
    """Drives the tube assembly workflow in a background thread.

    All hardware I/O goes through the ``HardwareHub``.  Posts
    ``AssemblyMsg`` objects so the GUI can update.  The GUI can set
    ``abandon_event`` to signal the worker to stop early.
    """

    def __init__(
        self,
        hub: HardwareHub,
        msg_queue: queue.Queue,
        tube: dict,
        tube_id: str,
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
    ):
        super().__init__(daemon=True, name="assembly-worker")
        self.hub = hub
        self.q = msg_queue
        self.tube = tube
        self.tube_id = tube_id
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
        # Zero motor at the start of each tube
        self.hub.zero_here()

        tube_start = time.perf_counter()

        self._log_event({
            "ts": now_iso_local(),
            "event": EVENT_TUBE_START,
            "design_file": self.design_path.name,
            "tube_id": self.tube_id,
            "tube_length_mm": float(self.tube.get("length", 0)),
            "num_joints": len(self.joints),
        })

        joints_completed = 0

        for ji, joint in enumerate(self.joints):
            if self.abandon.is_set():
                break

            joint_id = str(joint.get("id", f"joint_{ji}"))
            joint_type = str(joint.get("type", "")).strip()
            joint_ori = str(joint.get("ori", "")).strip()
            position_mm = float(joint.get("position_mm", 0.0))
            rotation_deg = float(joint.get("rotation_deg", 0.0))

            joint_offset = self.offset_map.get((joint_type, joint_ori), 0.0)
            target_mm = position_mm + self.cfg.linear.sensor_global_offset_mm + joint_offset

            # Tell the GUI we're starting a new joint (image + title)
            self._post("joint_start", joint_index=ji, joint_total=len(self.joints),
                        joint_id=joint_id, joint_type=joint_type, joint_ori=joint_ori,
                        position_mm=position_mm, target_mm=target_mm,
                        rotation_deg=rotation_deg)

            # ── LINEAR POSITIONING ───────────────────────────────
            self._post("step", phase="linear", joint_index=ji)
            self.confirm_linear.clear()
            linear_start = time.perf_counter()

            # Poll sensor via hub until the GUI signals confirmation
            while not self.confirm_linear.is_set() and not self.abandon.is_set():
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
            raw_mm = self.hub.read_sensor_mm_sync()
            measured = float(raw_mm) + self.cfg.linear.sensor_global_offset_mm
            error = measured - target_mm
            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_LINEAR_CONFIRM,
                "design_file": self.design_path.name,
                "tube_id": self.tube_id,
                "joint_id": joint_id, "joint_type": joint_type,
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
                    "tube_id": self.tube_id,
                    "joint_id": joint_id, "joint_type": joint_type,
                    "ori": joint_ori, "joint_seq": ji,
                    "target_deg": target, "actual_deg": actual_deg,
                    "time_since_motion_start_ms": elapsed_ms,
                })
                self._post("stall", actual_deg=actual_deg, target_deg=target,
                           elapsed_ms=elapsed_ms)
                self.stall_retry.clear()
                while not self.stall_retry.is_set() and not self.abandon.is_set():
                    time.sleep(0.1)

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
                "tube_id": self.tube_id,
                "joint_id": joint_id, "joint_type": joint_type,
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
                        joint_type=joint_type, joint_ori=joint_ori)
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
                "tube_id": self.tube_id,
                "joint_id": joint_id, "joint_type": joint_type,
                "ori": joint_ori, "joint_seq": ji,
                "install_time_ms": install_ms,
            })
            joints_completed += 1

        # ── Tube finished (complete or abandoned) ────────────────
        tube_ms = int((time.perf_counter() - tube_start) * 1000)

        if self.abandon.is_set():
            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_TUBE_ABANDONED,
                "design_file": self.design_path.name,
                "tube_id": self.tube_id,
                "tube_time_ms": tube_ms,
                "joints_completed": joints_completed,
                "joints_total": len(self.joints),
            })
            self._post("done", abandoned=True, tube_id=self.tube_id,
                        tube_time_ms=tube_ms)
        else:
            self._log_event({
                "ts": now_iso_local(),
                "event": EVENT_TUBE_INSTALL_COMPLETE,
                "design_file": self.design_path.name,
                "tube_id": self.tube_id,
                "tube_time_ms": tube_ms,
            })
            self._post("done", abandoned=False, tube_id=self.tube_id,
                        tube_time_ms=tube_ms)


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
        self.tubes: List[dict] = []
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

        # Tube table
        table_frame = tk.Frame(self.selection_frame, bg="#f0f0f0")
        table_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        columns = ("id", "length", "joints", "status")
        self.tube_tree = ttk.Treeview(table_frame, columns=columns,
                                       show="headings", selectmode="browse")
        self.tube_tree.heading("id", text="Tube ID",
                                command=lambda: self._sort_tree("id"))
        self.tube_tree.heading("length", text="Length (mm)",
                                command=lambda: self._sort_tree("length"))
        self.tube_tree.heading("joints", text="Joints",
                                command=lambda: self._sort_tree("joints"))
        self.tube_tree.heading("status", text="Last Assembled",
                                command=lambda: self._sort_tree("status"))
        self.tube_tree.column("id", width=100)
        self.tube_tree.column("length", width=100, anchor=tk.E)
        self.tube_tree.column("joints", width=70, anchor=tk.E)
        self.tube_tree.column("status", width=250)

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL,
                                   command=self.tube_tree.yview)
        self.tube_tree.configure(yscrollcommand=scrollbar.set)
        self.tube_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.tube_tree.bind("<<TreeviewSelect>>", self._on_tube_select)

        # Tube details text (shown below table when a row is selected)
        self.detail_label = tk.Label(
            self.selection_frame, text="", bg="#f0f0f0", justify=tk.LEFT,
            font=("Consolas", 9), anchor=tk.NW, wraplength=600,
        )
        self.detail_label.pack(fill=tk.X, padx=10, pady=(0, 5))

        # Start Assembly button
        self.start_btn = tk.Button(
            self.selection_frame, text="▶  Start Assembly",
            font=("Segoe UI", 12, "bold"), state=tk.DISABLED,
            command=self._on_start_assembly, bg="#4CAF50", fg="white",
            activebackground="#45a049", padx=20, pady=8,
        )
        self.start_btn.pack(pady=(5, 10))

        # Show selection by default
        self.selection_frame.pack(fill=tk.BOTH, expand=True)

        # ── Assembly wizard (Screen B) ───────────────────────────
        self.assembly_frame = tk.Frame(self.left_container, bg="#f0f0f0")

        # Top bar: tube/joint info + abandon button
        top_bar = tk.Frame(self.assembly_frame, bg="#ddd")
        top_bar.pack(fill=tk.X)

        # Title uses a frame with separate labels so only IDs are bold
        title_frame = tk.Frame(top_bar, bg="#ddd")
        title_frame.pack(side=tk.LEFT, padx=10, pady=5)
        self._title_prefix = tk.Label(title_frame, text="Tube: ", bg="#ddd",
                                       font=("Segoe UI", 12))
        self._title_prefix.pack(side=tk.LEFT)
        self._title_tube_id = tk.Label(title_frame, text="", bg="#ddd",
                                        font=("Segoe UI", 12, "bold"))
        self._title_tube_id.pack(side=tk.LEFT)
        self._title_mid = tk.Label(title_frame, text="", bg="#ddd",
                                    font=("Segoe UI", 12))
        self._title_mid.pack(side=tk.LEFT)
        self._title_joint_id = tk.Label(title_frame, text="", bg="#ddd",
                                         font=("Segoe UI", 12, "bold"))
        self._title_joint_id.pack(side=tk.LEFT)

        self.abandon_btn = tk.Button(
            top_bar, text="✕ Abandon Tube", fg="red",
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

        # Status text area (rotation result, stall messages, etc.)
        self.status_var = tk.StringVar(value="")
        tk.Label(self.assembly_frame, textvariable=self.status_var,
                 bg="#f0f0f0", font=("Segoe UI", 10),
                 wraplength=550, justify=tk.CENTER).pack(pady=5)

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

        # Motion settings display
        self._add_eng_label("Motion Settings")
        self.eng_motion_var = tk.StringVar(value="—")
        tk.Label(self.right_panel, textvariable=self.eng_motion_var,
                 bg="#e8e8e8", font=("Consolas", 9)).pack(anchor=tk.W, padx=10)

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
            messagebox.showerror("Hardware Error", str(e))
            self.eng_conn_var.set("DISCONNECTED")
            # Allow running without hardware for UI testing
            return

        # Start the GUI refresh loop (reads from hub + drains queues)
        self.after(GUI_DRAIN_MS, self._gui_tick)

    # ─────────────────────────────────────────────────────────────
    #  GUI tick — periodic refresh (replaces separate poll/drain)
    # ─────────────────────────────────────────────────────────────

    def _gui_tick(self) -> None:
        """Called every GUI_DRAIN_MS.  Updates engineer panel from hub
        cached values and drains the assembly message queue."""

        # ── Refresh engineer panel from hub ──────────────────────
        if self.hub:
            if self.hub.poll_error:
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
    #  Assembly message handler
    # ─────────────────────────────────────────────────────────────

    def _handle_assembly_msg(self, msg: AssemblyMsg) -> None:
        d = msg.data

        if msg.kind == "joint_start":
            ji = d["joint_index"]
            jt = d["joint_total"]
            jid = d["joint_id"]
            jtype = d["joint_type"]
            jori = d["joint_ori"]

            # Update title: only tube ID and joint ID are bold
            self._title_prefix.configure(text="Tube: ")
            self._title_tube_id.configure(text=self._current_tube_id)
            self._title_mid.configure(text=f"  —  Joint {ji+1}/{jt}  (")
            self._title_joint_id.configure(text=jid)
            # Close the parenthesis in a non-bold label after joint_id
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
                self.rot_progress_frame.pack(fill=tk.X, padx=30, pady=5)
                self.rot_progress["value"] = 0
                self.rot_progress_info_var.set("")
                # Store target for progress calculation
                self._rotation_start_deg = self.hub.motor_deg or 0.0
                self._rotation_target_deg = target_deg

            elif phase == "install":
                jtype = d.get("joint_type", "")
                jori = d.get("joint_ori", "")
                self.instruction_var.set(
                    f"Install joint now:\n"
                    f"Type: {jtype}   Orientation: {jori}\n\n"
                    f"Press Confirm (or Enter) when installed."
                )
                self.confirm_btn.configure(state=tk.NORMAL,
                                           text="Joint Installed")
                self._confirm_action = "install"
                self.linear_info_var.set("")
                # Hide both bars during install
                self.bar_frame.pack_forget()
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
            tube_ms = d.get("tube_time_ms", 0)
            tid = d.get("tube_id", "")
            if abandoned:
                messagebox.showinfo("Tube Abandoned",
                    f"Tube {tid} was abandoned.\n"
                    f"Time elapsed: {format_duration_ms(tube_ms)}")
            else:
                messagebox.showinfo("Tube Complete",
                    f"Tube {tid} assembly complete!\n"
                    f"Total time: {format_duration_ms(tube_ms)}")
            self._show_selection()

        elif msg.kind == "error":
            messagebox.showerror("Assembly Error",
                                 d.get("message", "Unknown"))
            self._show_selection()

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
    #  Tube table: sorting
    # ─────────────────────────────────────────────────────────────

    def _sort_tree(self, col: str) -> None:
        """Sort the Treeview by the given column (toggle asc/desc)."""
        items = [(self.tube_tree.set(iid, col), iid)
                 for iid in self.tube_tree.get_children("")]

        try:
            items.sort(key=lambda t: float(t[0]))
        except ValueError:
            items.sort(key=lambda t: t[0])

        current = [iid for _, iid in items]
        existing = list(self.tube_tree.get_children(""))
        if current == existing:
            items.reverse()

        for idx, (_, iid) in enumerate(items):
            self.tube_tree.move(iid, "", idx)

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
            tubes = design.get("tubes", [])
            if not isinstance(tubes, list) or len(tubes) == 0:
                messagebox.showerror("Error",
                    "Design file contains no tubes[].")
                return
        except Exception as e:
            messagebox.showerror("Error",
                f"Failed to load design:\n{e}")
            return

        self.design_path = design_path
        self.design = design
        self.tubes = tubes
        self.log_path = design_log_path(design_path, self.raw_settings)
        self.offset_map = build_offset_map(self.raw_settings)
        self.design_file_var.set(design_path.name)

        set_last_design_dir(self.raw_settings, str(design_path.parent))
        save_settings(self.settings_path, self.raw_settings)

        self._refresh_tube_table()

    def _refresh_tube_table(self) -> None:
        """Rebuild the tube Treeview from the current design + log."""
        self.tube_tree.delete(*self.tube_tree.get_children())
        self.detail_label.configure(text="")
        self.start_btn.configure(state=tk.DISABLED)

        if not self.tubes or not self.log_path:
            return

        events = parse_log_events(self.log_path)
        cmap = get_tube_completion_map(events)

        for idx, tube in enumerate(self.tubes):
            tid = tube.get("id", f"tube_{idx}")
            length = tube.get("length", "?")
            nj = len(tube.get("joints", []) or [])
            info = cmap.get(str(tid))
            if info:
                ago = format_time_ago(info.completed_at)
                dur = format_duration_ms(info.tube_time_ms)
                status = f"{ago} ({dur})"
            else:
                status = "—"
            self.tube_tree.insert("", tk.END, iid=str(idx),
                                   values=(tid, length, nj, status))

    def _on_tube_select(self, event) -> None:
        """Called when the user clicks a row in the tube table."""
        sel = self.tube_tree.selection()
        if not sel:
            self.start_btn.configure(state=tk.DISABLED)
            self.detail_label.configure(text="")
            return

        idx = int(sel[0])
        tube = self.tubes[idx]
        tid = tube.get("id", f"tube_{idx}")
        length = tube.get("length", "?")
        joints = tube.get("joints", []) or []

        lines = [f"Tube {tid}:  {length} mm,  {len(joints)} joint(s)\n"]
        for j in sorted(joints,
                         key=lambda x: float(x.get("position_mm", 0))):
            jid = j.get("id", "?")
            jt = j.get("type", "?")
            jo = j.get("ori", "?")
            pos = j.get("position_mm", "?")
            rot = j.get("rotation_deg", "?")
            lines.append(
                f"  {jid:<8}  type={jt:<10}  ori={jo:<4}  "
                f"pos={pos}mm  rot={rot}°"
            )
        self.detail_label.configure(text="\n".join(lines))

        if self.hub is not None:
            self.start_btn.configure(state=tk.NORMAL)

    # ─────────────────────────────────────────────────────────────
    #  Screen switching helpers
    # ─────────────────────────────────────────────────────────────

    def _show_selection(self) -> None:
        """Switch the left panel to the selection screen."""
        self.assembly_frame.pack_forget()
        self.selection_frame.pack(fill=tk.BOTH, expand=True)
        self._refresh_tube_table()

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
        self._current_photo = None
        self.image_label.configure(image="", text="")
        # Reset title labels
        self._title_tube_id.configure(text="")
        self._title_mid.configure(text="")
        self._title_joint_id.configure(text="")

    # ─────────────────────────────────────────────────────────────
    #  Start assembly
    # ─────────────────────────────────────────────────────────────

    def _on_start_assembly(self) -> None:
        """Validate, then switch to assembly screen and launch worker."""
        sel = self.tube_tree.selection()
        if not sel or self.hub is None:
            return

        idx = int(sel[0])
        tube = self.tubes[idx]
        tube_id = str(tube.get("id", f"tube_{idx}"))

        try:
            validate_offsets_complete(tube, self.offset_map)
        except RuntimeError as e:
            messagebox.showerror("Missing Offsets", str(e))
            return

        joints = list(tube.get("joints", []) or [])
        joints.sort(key=lambda j: float(j.get("position_mm", 0.0)))

        # Reset control state
        self.abandon_event.clear()
        self.confirm_linear_event.clear()
        self.confirm_install_event.clear()
        self.stall_retry_event.clear()
        self._confirm_action = None
        self._current_tube_id = tube_id
        self._last_linear_error = 999.0

        self._show_assembly()
        self._title_prefix.configure(text="Tube: ")
        self._title_tube_id.configure(text=tube_id)
        self._title_mid.configure(text="  —  Starting...")
        self._title_joint_id.configure(text="")

        self.assembly_worker = AssemblyWorker(
            hub=self.hub,
            msg_queue=self.assembly_queue,
            tube=tube,
            tube_id=tube_id,
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

    def _on_abandon(self) -> None:
        """Signal the assembly worker to stop and return to selection."""
        if messagebox.askyesno("Abandon Tube",
                                "Are you sure you want to abandon this tube?"):
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
