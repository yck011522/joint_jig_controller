"""
motor_comm.py — ZDT closed-loop stepper (EMM firmware) over Modbus RTU.

Design goals:
- The MotorComm object consumes an *already opened* pymodbus client.
- No locks inside (upper layer owns concurrency).
- All calls rely on Modbus timeouts so failures bubble up cleanly.

Key registers/commands used (from the vendor Modbus protocol PDF):
- Read realtime position: 0x0036, qty=3 (sign + 32-bit value)  [03/04]  :contentReference[oaicite:4]{index=4}
- Read motor status flags: 0x003A, qty=1; Prf_TF bit indicates reached :contentReference[oaicite:5]{index=5}
- Zero current angle: write single reg 0x000A = 0x0001            :contentReference[oaicite:6]{index=6}
- Position mode control (Emm): write multiple regs starting 0x00FD :contentReference[oaicite:7]{index=7}
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional, Tuple

from pymodbus.client import ModbusSerialClient



class MotorCommError(RuntimeError):
    pass


@dataclass
class MotorKinematics:
    motor_steps_per_rev: int = 200
    microsteps: int = 16
    gearbox_ratio: float = 30.0  # motor_rev : output_rev (1:30 gearbox => 30)

    @property
    def pulses_per_motor_rev(self) -> float:
        return float(self.motor_steps_per_rev * self.microsteps)

    @property
    def pulses_per_output_rev(self) -> float:
        return self.pulses_per_motor_rev * float(self.gearbox_ratio)

    @property
    def pulses_per_output_deg(self) -> float:
        return self.pulses_per_output_rev / 360.0


class ZDTEmmMotor:
    """
    Minimal Modbus-RTU wrapper for ZDT EMM firmware.

    Notes:
    - The motor's 0x0036 "realtime position" is documented as:
        value 0..65535 maps to 0..360 degrees (Emm) for one turn :contentReference[oaicite:8]{index=8}
      But the response format we observe/expect is: [sign, hi, lo] and we treat (hi<<16|lo)
      as the magnitude, matching your working logs (regs=[0,0,107]).
    - Reached detection uses status flag register 0x003A, Prf_TF bit (0x02). :contentReference[oaicite:9]{index=9}
    """

    # Read registers
    REG_REALTIME_POS = 0x0036
    REG_STATUS_FLAGS = 0x003A

    # Action command registers
    REG_ZERO_HERE = 0x000A

    # Position-mode control block (Emm)
    REG_POSMODE_BASE = 0x00FD

    def __init__(
        self,
        client,
        *,
        slave_id: int = 2,
        kinematics: Optional[MotorKinematics] = None,
        poll_period_s: float = 0.05,
    ):
        self.client = client  # type: ModbusSerialClient
        self.slave_id = slave_id
        self.kin = kinematics or MotorKinematics()
        self.poll_period_s = float(poll_period_s)

    # -------------------------
    # Low-level helpers
    # -------------------------
    def _require_ok(self, resp, action: str):
        if resp is None:
            raise MotorCommError(f"{action} failed: no response (None)")
        if hasattr(resp, "isError") and resp.isError():
            raise MotorCommError(f"{action} failed: {resp}")
        return resp

    @staticmethod
    def _u32_from_regs(hi16: int, lo16: int) -> int:
        return ((hi16 & 0xFFFF) << 16) | (lo16 & 0xFFFF)

    # -------------------------
    # Reads
    # -------------------------
    def read_realtime_position_regs(self) -> Tuple[int, int, int]:
        """Returns (sign_reg, reg_hi, reg_lo) from 0x0036 qty=3. :contentReference[oaicite:10]{index=10}"""
        resp = self.client.read_holding_registers(self.REG_REALTIME_POS, count=3, slave=self.slave_id)
        self._require_ok(resp, "read_realtime_position_regs")
        regs = list(resp.registers)
        if len(regs) != 3:
            raise MotorCommError(f"read_realtime_position_regs: expected 3 regs, got {len(regs)}: {regs}")
        return int(regs[0]), int(regs[1]), int(regs[2])

    def read_realtime_position_u32_signed(self) -> int:
        """Returns signed 32-bit-ish position value using sign_reg (0=+,1=-) like your logs."""
        sign, hi, lo = self.read_realtime_position_regs()
        mag = self._u32_from_regs(hi, lo)
        return -int(mag) if (sign & 0xFFFF) == 1 else int(mag)

    def read_realtime_position_deg_motor(self) -> float:
        """
        Converts the signed raw value to motor degrees using the Emm mapping:
        angle = (value * 360) / 65536. :contentReference[oaicite:11]{index=11}
        """
        signed = self.read_realtime_position_u32_signed()
        return float(signed) * 360.0 / 65536.0

    def read_realtime_position_deg_output(self) -> float:
        """Signed output shaft angle in degrees (uses gearbox_ratio)."""
        signed_counts = self.read_realtime_position_u32_signed()
        return float(signed_counts) * 360.0 / (65536.0 * float(self.kin.gearbox_ratio))

    def read_realtime_position_rev_output(self) -> float:
        """Signed output shaft angle in revolutions."""
        signed_counts = self.read_realtime_position_u32_signed()
        return float(signed_counts) / (65536.0 * float(self.kin.gearbox_ratio))
    
    def read_status_flags(self) -> int:
        """Reads status flags register 0x003A qty=1. :contentReference[oaicite:12]{index=12}"""
        resp = self.client.read_holding_registers(self.REG_STATUS_FLAGS, count=1, slave=self.slave_id)
        self._require_ok(resp, "read_status_flags")
        regs = list(resp.registers)
        if len(regs) != 1:
            raise MotorCommError(f"read_status_flags: expected 1 reg, got {len(regs)}: {regs}")
        return int(regs[0]) & 0xFFFF

    def is_enabled(self) -> bool:
        flags = self.read_status_flags()
        return bool(flags & 0x01)  # Ens_TF bit0 :contentReference[oaicite:13]{index=13}

    def is_reached(self) -> bool:
        flags = self.read_status_flags()
        return bool(flags & 0x02)  # Prf_TF bit1 :contentReference[oaicite:14]{index=14}

    # -------------------------
    # Actions
    # -------------------------
    def zero_here(self) -> None:
        """Clears current angle (target + realtime) by writing 0x000A = 1. :contentReference[oaicite:15]{index=15}"""
        resp = self.client.write_register(self.REG_ZERO_HERE, 0x0001, slave=self.slave_id)
        self._require_ok(resp, "zero_here")

    def move_relative_pulses_motor(
        self,
        pulses_motor: int,
        *,
        speed_rpm: int = 200,
        acc: int = 50,
        sync: int = 0,
        motion_mode: int = 0,
    ) -> None:
        """
        Position mode control (Emm) via write_multiple_registers starting at 0x00FD. :contentReference[oaicite:16]{index=16}

        Register packing (based on the table layout):
        - reg1: high byte = direction (00 CW / 01 CCW), low byte = acc
        - reg2: speed (RPM)
        - reg3+reg4: pulse count (32-bit magnitude)
        - reg5: high byte = motion_mode, low byte = sync

        motion_mode:
          0 = relative (incremental)   (matches the PDF’s “相对位置模式运动” idea for relative moves) :contentReference[oaicite:17]{index=17}
        """
        direction = 0 if pulses_motor >= 0 else 1
        mag = abs(int(pulses_motor)) & 0xFFFFFFFF

        reg1 = ((direction & 0xFF) << 8) | (int(acc) & 0xFF)
        reg2 = int(speed_rpm) & 0xFFFF
        reg3 = (mag >> 16) & 0xFFFF
        reg4 = mag & 0xFFFF
        reg5 = ((int(motion_mode) & 0xFF) << 8) | (int(sync) & 0xFF)

        resp = self.client.write_registers(
            self.REG_POSMODE_BASE,
            [reg1, reg2, reg3, reg4, reg5],
            slave=self.slave_id,
        )
        self._require_ok(resp, "move_relative_pulses_motor")

    def move_relative_deg_output(
        self,
        deg_output: float,
        *,
        speed_rpm: int = 200,
        acc: int = 50,
        sync: int = 0,
    ) -> int:
        """
        Moves the *output shaft* by deg_output using your kinematics assumption:
        motor pulses per output rev = 200 * microsteps * gearbox_ratio.

        Returns the integer motor pulses commanded.
        """
        pulses = int(round(float(deg_output) * self.kin.pulses_per_output_deg))
        self.move_relative_pulses_motor(pulses, speed_rpm=speed_rpm, acc=acc, sync=sync, motion_mode=0)
        return pulses

    def wait_until_reached(self, *, timeout_s: float = 10.0) -> bool:
        """
        Polls Prf_TF until reached or timeout. :contentReference[oaicite:18]{index=18}
        """
        t0 = time.time()
        while True:
            if self.is_reached():
                return True
            if (time.time() - t0) >= float(timeout_s):
                return False
            time.sleep(self.poll_period_s)


# -------------------------
# Demo mode
# -------------------------
def _demo():
    if ModbusSerialClient is None:
        raise RuntimeError("pymodbus is not available in this environment.")

    # These are YOUR settings:
    port = "COM3"
    baud = 115200
    slave_id = 2

    client = ModbusSerialClient(
        port=port,
        baudrate=baud,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=0.2,
    )

    if not client.connect():
        raise RuntimeError(f"Failed to connect ModbusSerialClient on {port} @ {baud}")

    motor = ZDTEmmMotor(
        client,
        slave_id=slave_id,
        kinematics=MotorKinematics(motor_steps_per_rev=200, microsteps=16, gearbox_ratio=30.0),
        poll_period_s=0.05,
    )

    try:
        print(f"[demo] connected. enabled={motor.is_enabled()} reached={motor.is_reached()}")

        print("[demo] zero_here() ...")
        motor.zero_here()

        # Move +360° output, wait, then back to 0 (i.e., -360° relative)
        print("[demo] move +360deg output ...")
        pulses = motor.move_relative_deg_output(+360.0, speed_rpm=500, acc=500)
        ok = motor.wait_until_reached(timeout_s=20.0)
        motor_deg = motor.read_realtime_position_deg_motor()
        out_deg = motor.read_realtime_position_deg_output()
        print(
            f"[demo] commanded pulses_motor={pulses} reached={ok} "
            f"motor_deg_now={motor_deg:.3f}  output_deg_now={out_deg:.3f}"
        )
        time.sleep(0.3)

        print("[demo] move -360deg output ...")
        pulses = motor.move_relative_deg_output(-360.0, speed_rpm=500, acc=500)
        ok = motor.wait_until_reached(timeout_s=20.0)
        motor_deg = motor.read_realtime_position_deg_motor()
        out_deg = motor.read_realtime_position_deg_output()
        print(
            f"[demo] commanded pulses_motor={pulses} reached={ok} "
            f"motor_deg_now={motor_deg:.3f}  output_deg_now={out_deg:.3f}"
        )
        print("[demo] done.")

    finally:
        client.close()


if __name__ == "__main__":
    _demo()