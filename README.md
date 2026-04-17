# Bar Joint Placement Jig  
*A semi-automated 2-DoF human–machine cooperative assembly system*

---

# 1. Overview

This project is a semi-automated mechanical jig designed to assist an operator in placing 3D-printed joints onto a metal bar with controlled:

- **Axial position (Joint 1)**
- **Rotational orientation (Joint 2)**

The system combines:

- Manual actuation (linear carriage)
- Motorized rotation (FOC-controlled stepper + harmonic drive)
- Distance sensing (axial measurement)
- Python-based supervisory control software
- JSON-based design input
- Production logging and statistical analysis

The machine enforces assembly order, assists positioning accuracy, and generates structured production data.

---

# 2. Mechanical System Definition

The system has two true mechanical degrees of freedom:

---

## Joint 1 — Linear Position (Manual Axis)

**Unit:** millimeters (mm)  
**Actuation:** manual sliding carriage  
**Measurement:** distance sensor  

### Characteristics

- Operator moves carriage along linear rail.
- Position continuously measured by distance sensor.
- Mechanical lock fixes carriage before installation.

### Expected Accuracy

- Target tolerance: ±2 mm
- Accuracy limited primarily by:
  - Manual positioning
  - Sensor precision
  - Mechanical play in slide

This axis is intentionally human-actuated to preserve flexibility and tactile control.

---

## Joint 2 — Rotation (Motorized Axis)

**Unit:** degrees (°)  
**Actuation:** FOC-controlled stepper motor  
**Transmission:** Harmonic drive gearbox (1:30)

### Motor Specification

- Motor size: 42 mm frame, 60 mm length
- Step resolution: 200 steps/rev
- Continuous current rating: 2 A
- Rated torque: 0.8 Nm
- Manufacturer: Zhangdatou
- Controller board: X42S_V1.0 (FOC control)

### Gearbox Specification

- Type: Harmonic drive
- Ratio: 1:30
- Rated torque: 3.4 Nm
- Rated input speed: 3500 rpm

---

### Rotational Accuracy Considerations

The FOC-controlled stepper theoretically offers very high positioning resolution.  
However, practical accuracy will be limited by:

- Harmonic drive compliance
- Mechanical backlash (minimal but non-zero)
- Tube clamping rigidity
- Structural deflection
- Assembly torque loads during joint tightening

Therefore, achievable rotational accuracy is expected to be limited by mechanical factors rather than theoretical motor resolution.

---

# 3. Operational Workflow

## 3.1 Setup Phase

1. Operator loads metal bar.
2. Bar length verified against design specification.
3. Bar clamped in motor chuck.
4. Stepper rotation set to zero reference.
5. Operator confirms installation.

---

## 3.2 Joint Installation Loop

For each joint (strict left-to-right order):

### Step 1 — Linear Positioning

System displays:

- Current axial position
- Target position
- Directional correction (← / →)
- Distance error

Operator:

- Unlocks carriage
- Slides to target
- Locks carriage

System verifies position within tolerance (±2 mm).

Operator confirms position lock.

---

### Step 2 — Rotational Alignment

System automatically:

- Calculates required motor steps
- Compensates for 1:30 gearbox ratio
- Rotates tube to required angle

Display shows:

- Joint type
- Required orientation
- Installation instruction

---

### Step 3 — Joint Installation

Operator:

- Installs joint
- Tightens screws
- Confirms completion

System logs:

- Target position
- Measured position
- Position error
- Target rotation
- Motor state
- Timestamp

---

### Step 4 — Advance

Operator unlocks carriage and proceeds to next joint.

---

## 3.3 Completion

System displays:

- Bar complete message
- Bar ID

Future extension: automatic label printing  
Current method: manual labeling

---

# 4. Software Architecture

## Environment Setup

```
conda create -n joint_jig python=3.11 pip -c conda-forge
conda activate joint_jig
pip install -r requirements.txt
```

### Dependencies

| Package    | Purpose                                    |
|------------|--------------------------------------------|
| pymodbus   | Modbus RTU communication (motor + sensor)  |
| pyserial   | Serial port enumeration and auto-detection |
| Pillow     | JPEG joint reference images in the GUI     |

All dependencies are listed in `requirements.txt`.

## 4.1 Supervisory Control Software (Python)

Runs on PC and communicates via RS-485 USB adapters to:

- Stepper motor controller
- Distance sensor

### Responsibilities

- Load JSON design file
- Manage workflow state machine
- Enforce assembly order
- Monitor sensor data
- Perform tolerance checking
- Control motor rotation
- Log production data
- Provide statistics dashboard

No embedded microcontroller layer is currently required.

---

## 4.2 Design Interface (Grasshopper → JSON)

Tube designs are authored in Rhino/Grasshopper.

Exported JSON describes:

- Multiple bars
- Bar length
- Ordered joint list
- Joint type and subtype
- Axial position (mm)
- Rotation angle (deg)
- Orientation variant (P/N)

JSON acts as interface between design and production.

---

# 5. JSON Design Schema

Design files follow schema v1 (see [json_schema.md](json_schema.md) for full spec).

```json
{
  "schema_version": 1,
  "project_id": "Batch_2026_04",
  "bars": [
    {
      "bar_id": "B1",
      "length_mm": 1000,
      "joints": [
        {
          "joint_id": "J1",
          "type": "T20",
          "subtype": "Female",
          "ori": "P",
          "position_mm": 120.0,
          "rotation_deg": 45.0
        }
      ]
    }
  ]
}
```

Joints are sorted by `position_mm` at runtime — file order does not matter.
# 6. Configuration File

Control Software settings file (settings.json) defines:

- Distance sensor offset
- Tolerance threshold (default ±2 mm)
- Gearbox ratio
- Motor step resolution
- Joint-type-specific offsets

Hardware Settings (Distance Sensor)

- Baud Rate = 115200
- RS485 Address = 01

Hardware Settings (ZDT X42S V1.0 FOC Stepper Motor)

- Driver Type = Emm
- Baud Rate = 115200
- RS485 Address = 02
- Checksum = ModBus


# 7. Production Logging

Each JSON file has a corresponding log file.

Logged per joint:

- Target axial position
- Measured axial position
- Position error
- Target rotation
- Motor steps
- Timestamp
- Linear positioning time
- Joint fastening time

# 8. Software Components

| File | Purpose |
|------|---------|
| `controller_gui.py` | Tkinter GUI — operator wizard + engineer debug panel |
| `controller_cli.py` | Terminal-based CLI for headless operation |
| `jig_controller.py` | Shared module — settings, hardware, workflow logic, logging |
| `motor_comm.py` | ZDT Emm motor driver (Modbus RTU) |
| `distance_sensor.py` | JK-LRD distance sensor driver (Modbus RTU) |
| `settings.json` | Runtime configuration (serial, motion, stall, offsets) |
| `requirements.txt` | Python dependencies |

### GUI features

- Split-panel layout: operator assembly wizard (left) + engineer panel (right)
- Joint reference images displayed during each installation step
- Rotation progress bar during motor movement
- Bar completion status table with time-ago display
- Stall detection with retry prompt
- Automatic reconnect button when the USB adapter is unplugged

### CLI features

- Interactive terminal workflow with live sensor readout
- Design file selection with bar completion status
- Abandon bar support

### Reconnect behaviour

If the RS-485 USB adapter is disconnected mid-session, the system detects
the failure (3 consecutive poll errors) and shows a **Reconnect** button in
the engineer panel (GUI) or an error message (CLI). Re-plugging the adapter
and pressing Reconnect re-establishes the connection without restarting.

# 9. System Philosophy

This is not a fully automated system.

It is a human–machine cooperative jig where:

Human provides manipulation and assembly skill.

Machine enforces geometric precision and sequence.

Data logging transforms manual fabrication into measurable production.

# 10. Future Extensions

- Automatic label printer
- Statistics & analytics module (per-bar metrics, operator trends, build-time prediction)
- Multi-operator analytics
- Time prediction model
- Real-time graphical tolerance visualization
- Cloud-based dashboard

QR-code tube tracking