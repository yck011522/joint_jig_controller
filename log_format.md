# Log file format v1 (JSON Lines)

## File naming

- If design file is: `MyDesign.json`
- Log file is: `MyDesign.log.jsonl` (same folder as design)

Each line is one JSON object.

------

## Common fields in every event

- `ts` - ISO timestamp string (local time, e.g. `"2026-03-03T21:15:23.123"`)
- `event` - string event name
- `design_file` - basename only
- `bar_id` - string
- `joint_id` - string or `null` (bar-level events use null)
- `joint_type` - string or `null` (the base type, e.g. `"T20"`)
- `joint_subtype` - string or `null` (the variant, e.g. `"Female"`)
- `ori` - string or `null` (`"P"` / `"N"`)
- `joint_seq` - integer or `null` (0..N-1, execution order after sorting)

------

## Event types (v1)

### 1) `bar_start`

Written when operator presses **Start** (after bar mounted + motor zeroed).

Fields:

- common fields +:
- `bar_length_mm`
- `num_joints`

Example:

``json
{
  "ts": "2026-03-03T21:15:23.123",
  "event": "bar_start",
  "design_file": "MyDesign.json",
  "bar_id": "B1",
  "bar_length_mm": 2400.0,
  "num_joints": 10
}
``

------

### 2) `linear_confirm`

Written when operator confirms the **linear slide position** (within tolerance).

Fields:

- common fields +:
- `target_mm` (design position + offsets)
- `measured_mm_raw` (sensor raw read)
- `measured_mm` (raw + global offset)
- `error_mm` (measured - target)
- `tol_mm`
- `linear_time_ms`

Example:

``json
{
  "ts": "2026-03-03T21:16:02.100",
  "event": "linear_confirm",
  "design_file": "MyDesign.json",
  "bar_id": "B1",
  "joint_id": "J3",
  "joint_type": "T20",
  "joint_subtype": "Female",
  "ori": "P",
  "joint_seq": 2,
  "target_mm": 360.0,
  "measured_mm_raw": 358.7,
  "measured_mm": 358.7,
  "error_mm": -1.3,
  "tol_mm": 2.0,
  "linear_time_ms": 18450
}
``

------

### 3) `rotation_reached`

Written when motor reports **target rotation reached**.

Fields:

- common fields +:
- `target_deg`
- `actual_deg`
- `tol_deg`
- `rot_speed_rpm`
- `rot_acc`
- `rotation_time_ms`
- `motor_reached_flag` (boolean)

Example:

``json
{
  "ts": "2026-03-03T21:16:05.420",
  "event": "rotation_reached",
  "design_file": "MyDesign.json",
  "bar_id": "B1",
  "joint_id": "J3",
  "joint_type": "T20",
  "joint_subtype": "Female",
  "ori": "P",
  "joint_seq": 2,
  "target_deg": 90.0,
  "actual_deg": 90.4,
  "tol_deg": 1.0,
  "rot_speed_rpm": 200.0,
  "rot_acc": 50.0,
  "rotation_time_ms": 3120,
  "motor_reached_flag": true
}
``

------

### 4) `install_confirm`

Written when operator presses **Installed**.

Fields:

- common fields +:
- `install_time_ms`

Example:

``json
{
  "ts": "2026-03-03T21:17:12.000",
  "event": "install_confirm",
  "design_file": "MyDesign.json",
  "bar_id": "B1",
  "joint_id": "J3",
  "joint_type": "T20",
  "joint_subtype": "Female",
  "ori": "P",
  "joint_seq": 2,
  "install_time_ms": 66400
}
``

------

### 5) `stall_detected`

Written when the motor stall protection triggers during rotation.

Fields:

- common fields +:
- `target_deg`
- `actual_deg`
- `time_since_motion_start_ms`

------

### 6) `bar_install_complete`

Written when the last joint is installed successfully.

> **Note:** The code also accepts legacy event names `tube_complete` and
> `tube_install_complete` when reading existing logs, but new events are
> always written as `bar_install_complete`.

Fields:

- common fields +:
- `bar_time_ms`

Example:

``json
{
  "ts": "2026-03-03T21:34:55.010",
  "event": "bar_install_complete",
  "design_file": "MyDesign.json",
  "bar_id": "B1",
  "bar_time_ms": 1180000
}
``

------

### 7) `bar_abandoned`

Written when the operator abandons a bar mid-assembly.

Fields:

- common fields +:
- `bar_time_ms`
- `joints_completed` - number of joints finished before abandoning
- `joints_total` - total number of joints in the bar

------

# How "production status" is derived for the table

When loading a design:

- read the log file line-by-line
- for each `bar_id`, look for the **latest** `bar_install_complete`
  (or legacy `tube_install_complete` / `tube_complete`)
- status column = "Done (timestamp)" or "Not done"

This stays stable even if you later add more event types.