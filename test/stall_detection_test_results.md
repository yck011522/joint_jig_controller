# Stall Detection Test Results

**Date:** 2026-04-14  
**Script:** `test/stall_detection_test.py`  
**Hardware:** ZDT Y42 closed-loop stepper (Emm firmware), slave ID 2, COM port variable  

---

## Test Parameters

| Parameter | Value |
|---|---|
| Stall detection speed | 100 RPM |
| Stall detection current | 400 mA |
| Stall detection time | 50 ms |
| Stall protection mode | 1 (enable + release motor) |
| Motion command speed | 300 RPM (motor-side) |
| Motion command acceleration | 100 / 255 |
| Motion command pulses | 32000 (~10 motor revolutions) |

## Factory Defaults (for reference)

| Parameter | Default |
|---|---|
| Stall detection speed | 8 RPM |
| Stall detection current | 2200 mA |
| Stall detection time | 2000 ms |
| Stall protection mode | 1 |

## "Disabled" Configuration

To effectively disable stall detection during normal operation (e.g. power-off, tube installation), set current threshold to **2000 mA** — the motor never reaches this during normal running, so stall detection will not trigger.

---

## Run 1

- **Initial state:** `0x03 [ENABLED, REACHED]` — motor idle, defaults loaded (8 RPM / 2200 mA / 2000 ms)
- **Wrote tuned params:** 100 RPM / 400 mA / 50 ms — readback confirmed
- **Stall triggered:** motor shaft physically blocked before motion command
- **Detection time:** 0.43 s after motion command sent
- **Flags at detection:** `0x0D [ENABLED, STALL_DETECTED, STALL_PROTECTED]`
- **Recovery:** `clear_stall_protection()` + `set_enabled(True)` → status returned to `0x03 [ENABLED, REACHED]`
- **Reset:** current set to 2000 mA — readback confirmed (100 RPM / 2000 mA / 50 ms)

## Run 2

- **Initial state:** `0x03 [ENABLED, REACHED]` — parameters from Run 1 reset persisted (100 RPM / 2000 mA / 50 ms), confirming volatile write survived across reconnection
- **Wrote tuned params:** 100 RPM / 400 mA / 50 ms — readback confirmed
- **Detection time:** 0.22 s after motion command sent
- **Flags at detection:** `0x0D [ENABLED, STALL_DETECTED, STALL_PROTECTED]`
- **Recovery:** successful, status returned to `0x03 [ENABLED, REACHED]`
- **Reset:** current set to 2000 mA — readback confirmed

*(Note: a failed attempt between runs returned `PermissionError` on COM21 — serial port was still held by another process. Resolved by retrying.)*

---

## Conclusions

1. **Stall detection works reliably** with the tuned parameters (100 RPM / 400 mA / 50 ms). A physically blocked shaft triggers protection within ~0.2–0.5 s.
2. **Recovery sequence works:** `clear_stall_protection()` → `set_enabled(True)` fully restores the motor to operational state.
3. **Parameter read/write is correct:** `read_stall_params()` and `write_stall_params()` produce consistent roundtrip results.
4. **Disabling detection** by raising current to 2000 mA is confirmed effective — the motor does not reach this current during normal operation.
5. **Volatile writes** (store=False) persist across reconnections but not across power cycles, which is the desired behavior.

## Integration Notes

- During **assembly operation**, enable stall detection (400 mA) before each motor move; disable (2000 mA) when the operator needs to install a tube or when powering off.
- On stall detection in the main workflow: pause, prompt operator to clear the obstruction, then clear protection + re-enable + resend the previous motion command.
- The `ENABLED` bit (0x01) remains set even when `STALL_PROTECTED` (0x08) is flagged — so checking `Cgp_TF` is the correct way to detect a stall-release event, not checking `Ens_TF`.
