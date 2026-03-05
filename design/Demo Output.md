```
[init] Opening Modbus RTU on COM3 @ 115200 ...
[init] Modbus connected.
[probe] Sensor OK: 1967 mm | Motor OK: 0.000 deg (output)

=== Tube List ===
  [0] id=T1  length=1000mm  joints=4
Select tube index and press Enter: 0

=== Ready ===
Tube: T1  length=1000.0mm  joints=4
Design file: C:\Users\Victor\Documents\GitHub\joint_jig_controller\design\dummy.json
Log file:    C:\Users\Victor\Documents\GitHub\joint_jig_controller\design\dummy.log.jsonl
Tolerance:   linear ±2.0 mm | rotation ±1.0 deg

Mount tube, then press Enter to START (will zero motor here)...

------------------------------------------------------------
Joint 1/4 | id=J1-4 | type=T20-5-M ori=L
Target: position_mm=120.0mm  -> target_mm=120.0mm (global+joint offsets applied)
Target rotation: rotation_deg=0.0°

Move carriage to target. Press ENTER to attempt confirm (non-blocking on Windows).
Live: current_mm_raw | measured_mm | target_mm | error_mm
  raw=   120 mm | meas=  120.00 | target=  120.00 | err=   0.00   (tol ±2.0)00   (tol ±2.0)
[ok] Linear position confirmed.
[rot] reached=True target=0.00° actual=0.00° time=10ms

Install joint now: type=T20-5-M ori=L. Press ENTER when installed.


------------------------------------------------------------
Joint 2/4 | id=J002 | type=T20-5-F ori=L
Target: position_mm=320.0mm  -> target_mm=320.0mm (global+joint offsets applied)
Target rotation: rotation_deg=45.0°

Move carriage to target. Press ENTER to attempt confirm (non-blocking on Windows).
Live: current_mm_raw | measured_mm | target_mm | error_mm
  raw=   321 mm | meas=  321.00 | target=  320.00 | err=   1.00   (tol ±2.0)
[ok] Linear position confirmed.
[rot] reached=True target=45.00° actual=45.00° time=3007ms

Install joint now: type=T20-5-F ori=L. Press ENTER when installed.


------------------------------------------------------------
Joint 3/4 | id=J003 | type=T20-5-M ori=R
Target: position_mm=580.0mm  -> target_mm=580.0mm (global+joint offsets applied)
Target rotation: rotation_deg=90.0°

Move carriage to target. Press ENTER to attempt confirm (non-blocking on Windows).
Live: current_mm_raw | measured_mm | target_mm | error_mm
  raw=   581 mm | meas=  581.00 | target=  580.00 | err=   1.00   (tol ±2.0)
[ok] Linear position confirmed.
[rot] reached=True target=90.00° actual=90.02° time=3002ms

Install joint now: type=T20-5-M ori=R. Press ENTER when installed.


------------------------------------------------------------
Joint 4/4 | id=J004 | type=T20-5-F ori=R
Target: position_mm=860.0mm  -> target_mm=860.0mm (global+joint offsets applied)
Target rotation: rotation_deg=135.0°

Move carriage to target. Press ENTER to attempt confirm (non-blocking on Windows).
Live: current_mm_raw | measured_mm | target_mm | error_mm
  raw=   863 mm | meas=  863.00 | target=  860.00 | err=   3.00   (tol ±2.0)
[warn] Outside tolerance. Keep adjusting and press ENTER again.
  raw=   861 mm | meas=  861.00 | target=  860.00 | err=   1.00   (tol ±2.0)
[ok] Linear position confirmed.
[rot] reached=True target=135.00° actual=135.02° time=3007ms

Install joint now: type=T20-5-F ori=R. Press ENTER when installed.


============================================================
Tube complete: T1 | total time = 127.9s
Log written to: C:\Users\Victor\Documents\GitHub\joint_jig_controller\design\dummy.log.jsonl
[exit] Serial port closed.
```

