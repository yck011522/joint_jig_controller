# Log file format v1 (JSON Lines)

## File naming

- If design file is: `MyDesign.json`
- Log file is: `MyDesign.log.jsonl` (same folder as design)

Each line is one JSON object.

------

## Common fields in every event

These make filtering/analysis easy and robust:

- `ts` — ISO timestamp string (e.g. `"2026-03-03T21:15:23.123"` - local time without timezone offset)
- `event` — string event name
- `design_file` — basename only (optional but helpful)
- `tube_id` — string
- `joint_id` — string or `null` (tube-level events use null)
- `joint_type` — string or `null`
- `ori` — string or `null`
- `joint_seq` — integer or `null` (0..N-1, after sorting)

That’s the “index” you removed from design, but it’s still useful in logs as the *execution order index*.

------

## Event types (v1)

### 1) `tube_start`

Written when operator presses **Start** (after tube mounted + motor zeroed).

Fields:

- common fields +:
- `tube_length_mm`
- `num_joints`

Example:

```
{
  "ts": "2026-03-03T21:15:23.123",
  "event": "tube_start",
  "design_file": "MyDesign.json",
  "tube_id": "TUBE_001",
  "joint_id": null,
  "joint_type": null,
  "ori": null,
  "joint_seq": null,
  "tube_length_mm": 2400.0,
  "num_joints": 10
}
```

------

### 2) `linear_confirm`

Written when operator confirms the **linear slide position** (only when within tolerance).

Fields:

- common fields +:
- `target_mm` (design position + offsets)
- `measured_mm_raw` (sensor raw read)
- `measured_mm` (raw + global offset) *(or omit if you prefer raw-only; but I’d keep both)*
- `error_mm` (measured - target)
- `tol_mm`
- `linear_time_ms` (time spent in POSITIONING state for this joint)

Example:

```
{
  "ts": "2026-03-03T21:16:02.100",
  "event": "linear_confirm",
  "design_file": "MyDesign.json",
  "tube_id": "TUBE_001",
  "joint_id": "J003",
  "joint_type": "T20-5-M",
  "ori": "L",
  "joint_seq": 2,
  "target_mm": 360.0,
  "measured_mm_raw": 358.7,
  "measured_mm": 358.7,
  "error_mm": -1.3,
  "tol_mm": 2.0,
  "linear_time_ms": 18450
}
```

*(If you want “Confirm even when outside tol but show popup”: you still only log `linear_confirm` after a valid confirm; invalid confirm attempts don’t need to be logged in v1.)*

------

### 3) `rotation_reached`

Written when motor reports **target rotation reached**.

Fields:

- common fields +:
- `target_deg`
- `actual_deg`
- `tol_deg`
- `rot_speed_rpm` (the commanded speed in RPM)
- `rot_acc_rpm_s` (the commanded acceleration in RPM/s)
- `rotation_time_ms`
- `motor_reached_flag` (boolean: true if motor reached target, false otherwise)

Example:

```
{
  "ts": "2026-03-03T21:16:05.420",
  "event": "rotation_reached",
  "design_file": "MyDesign.json",
  "tube_id": "TUBE_001",
  "joint_id": "J003",
  "joint_type": "T20-5-M",
  "ori": "L",
  "joint_seq": 2,
  "target_deg": 90.0,
  "actual_deg": 90.4,
  "tol_deg": 1.0,
  "rot_speed_rpm": 200.0,
  "rot_acc_rpm_s": 50.0,
  "rotation_time_ms": 3120,
  "motor_reached_flag": true
}
```

------

### 4) `install_confirm`

Written when operator presses **Installed**.

Fields:

- common fields +:
- `install_time_ms`

Optionally:

- `note` (string) for future manual notes (can omit)

Example:

```
{
  "ts": "2026-03-03T21:17:12.000",
  "event": "install_confirm",
  "design_file": "MyDesign.json",
  "tube_id": "TUBE_001",
  "joint_id": "J003",
  "joint_type": "T20-5-M",
  "ori": "L",
  "joint_seq": 2,
  "install_time_ms": 66400
}
```

------

### 5) `stall_detected`

Written when the motor stall protection triggers during rotation.

Fields:

- common fields +:
- `target_deg`
- `actual_deg`
- `time_since_motion_start_ms`

------

### 6) `tube_install_complete`

Written when the last joint is installed successfully.

> **Note:** The code also accepts the legacy event name `tube_complete` when
> reading existing logs, but new events are always written as
> `tube_install_complete`.

Fields:

- common fields +:
- `tube_time_ms`

Example:

```
{
  "ts": "2026-03-03T21:34:55.010",
  "event": "tube_install_complete",
  "design_file": "MyDesign.json",
  "tube_id": "TUBE_001",
  "joint_id": null,
  "joint_type": null,
  "ori": null,
  "joint_seq": null,
  "tube_time_ms": 1180000
}
```

------

### 7) `tube_abandoned`

Written when the operator abandons a tube mid-assembly.

Fields:

- common fields +:
- `tube_time_ms`
- `completed_joints` — number of joints finished before abandoning

------

# How “production status” is derived for the table

When loading a design:

- read the log file line-by-line
- for each `tube_id`, look for the **latest** `tube_install_complete` (or legacy `tube_complete`)
- status column = “Done (timestamp)” or “Not done”

This stays stable even if you later add more event types.