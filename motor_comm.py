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

    # Driver config block (Emm): read at 0x0042 (15 regs), write at 0x0048 (15 regs)
    REG_DRIVER_CONFIG_READ = 0x0042
    REG_DRIVER_CONFIG_WRITE = 0x0048
    DRIVER_CONFIG_REG_COUNT = 15

    # Action command registers
    REG_ZERO_HERE = 0x000A
    REG_CLEAR_PROTECTION = 0x000E
    REG_ENABLE_CONTROL = 0x00F3

    # Position-mode control block (Emm)
    REG_POSMODE_BASE = 0x00FD

    # Status flag bitmasks
    FLAG_ENABLED = 0x01       # Ens_TF bit0
    FLAG_REACHED = 0x02       # Prf_TF bit1
    FLAG_STALL_DETECTED = 0x04   # Cgi_TF bit2 — stall condition met
    FLAG_STALL_PROTECTED = 0x08  # Cgp_TF bit3 — stall protection triggered, motor released

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
        return bool(flags & self.FLAG_ENABLED)

    def is_reached(self) -> bool:
        flags = self.read_status_flags()
        return bool(flags & self.FLAG_REACHED)

    def is_stall_detected(self) -> bool:
        """True when real-time speed < threshold AND current > threshold (Cgi_TF bit2)."""
        flags = self.read_status_flags()
        return bool(flags & self.FLAG_STALL_DETECTED)

    def is_stall_protection_triggered(self) -> bool:
        """True when stall persisted beyond time threshold and motor was released (Cgp_TF bit3)."""
        flags = self.read_status_flags()
        return bool(flags & self.FLAG_STALL_PROTECTED)

    # -------------------------
    # Actions
    # -------------------------
    def zero_here(self) -> None:
        """Clears current angle (target + realtime) by writing 0x000A = 1. :contentReference[oaicite:15]{index=15}"""
        resp = self.client.write_register(self.REG_ZERO_HERE, 0x0001, slave=self.slave_id)
        self._require_ok(resp, "zero_here")

    # -------------------------
    # Stall protection & enable control
    # -------------------------
    def clear_stall_protection(self) -> None:
        """Release stall/overheat/overcurrent protection (register 0x000E)."""
        resp = self.client.write_register(self.REG_CLEAR_PROTECTION, 0x0052, slave=self.slave_id)
        self._require_ok(resp, "clear_stall_protection")

    def set_enabled(self, enable: bool, *, sync: int = 0) -> None:
        """Enable (True) or disable (False) the motor driver via register 0x00F3."""
        enable_byte = 0x01 if enable else 0x00
        reg1 = (0xAB << 8) | enable_byte
        reg2 = int(sync) & 0xFFFF
        resp = self.client.write_registers(
            self.REG_ENABLE_CONTROL, [reg1, reg2], slave=self.slave_id
        )
        self._require_ok(resp, f"set_enabled({enable})")

    def read_driver_config(self) -> list[int]:
        """Read the full 15-register Emm driver config block starting at 0x0042."""
        resp = self.client.read_holding_registers(
            self.REG_DRIVER_CONFIG_READ, count=self.DRIVER_CONFIG_REG_COUNT, slave=self.slave_id
        )
        self._require_ok(resp, "read_driver_config")
        regs = list(resp.registers)
        if len(regs) != self.DRIVER_CONFIG_REG_COUNT:
            raise MotorCommError(
                f"read_driver_config: expected {self.DRIVER_CONFIG_REG_COUNT} regs, got {len(regs)}: {regs}"
            )
        return regs

    def read_stall_params(self) -> dict:
        """Read current stall protection parameters from the driver config.

        Returns dict with keys: mode, speed_rpm, current_ma, time_ms.
        """
        regs = self.read_driver_config()
        return {
            "mode": regs[10] & 0xFF,           # low byte of reg[10]
            "speed_rpm": regs[11],              # reg[11] full 16-bit
            "current_ma": regs[12],             # reg[12] full 16-bit
            "time_ms": regs[13],                # reg[13] full 16-bit
        }

    def write_stall_params(
        self,
        *,
        mode: int = 0x01,
        speed_rpm: int = 100,
        current_ma: int = 400,
        time_ms: int = 50,
        store: bool = False,
    ) -> None:
        """Write stall protection parameters via read-modify-write of the driver config.

        Args:
            mode: 0x00=off, 0x01=enable (release motor on stall), 0x02=reset-to-zero
            speed_rpm: stall detection speed threshold (0–3000)
            current_ma: stall detection current threshold (0–5000)
            time_ms: stall detection duration threshold (0–65535)
            store: if True, persist to EEPROM; if False, volatile only
        """
        regs = self.read_driver_config()

        # Prepare the write block: reg[0] gets aux code 0xD1 + store flag
        store_flag = 0x01 if store else 0x00
        write_regs = list(regs)
        write_regs[0] = (0xD1 << 8) | store_flag

        # Modify stall parameters (regs 10–13)
        # reg[10]: preserve response_mode (high byte), set stall_protection_mode (low byte)
        write_regs[10] = (regs[10] & 0xFF00) | (int(mode) & 0xFF)
        write_regs[11] = int(speed_rpm) & 0xFFFF
        write_regs[12] = int(current_ma) & 0xFFFF
        write_regs[13] = int(time_ms) & 0xFFFF

        # reg[9]: clear high byte (reserved) per spec for writes
        write_regs[9] = write_regs[9] & 0x00FF

        resp = self.client.write_registers(
            self.REG_DRIVER_CONFIG_WRITE, write_regs, slave=self.slave_id
        )
        self._require_ok(resp, "write_stall_params")

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

    def move_absolute_pulses_motor(
        self,
        target_pulses_motor: int,
        *,
        speed_rpm: int = 200,
        acc: int = 50,
        sync: int = 0,
    ) -> None:
        """
        Moves to an *absolute* target position given in motor pulses.

        Uses the same Emm position-mode control block, but motion_mode=1 (absolute).
        """
        self.move_relative_pulses_motor(
            pulses_motor=int(target_pulses_motor),
            speed_rpm=speed_rpm,
            acc=acc,
            sync=sync,
            motion_mode=1,  # absolute
        )

    def move_absolute_deg_output(
        self,
        target_deg_output: float,
        *,
        speed_rpm: int = 200,
        acc: int = 50,
        sync: int = 0,
    ) -> int:
        """
        Moves the *output shaft* to an absolute target angle in degrees.

        Returns the integer motor pulses commanded as the absolute target.
        """
        target_pulses = int(round(float(target_deg_output) * self.kin.pulses_per_output_deg))
        # motor_rpm = int(round(float(speed_rpm) * self.kin.gearbox_ratio))
        self.move_absolute_pulses_motor(
            target_pulses,
            speed_rpm=speed_rpm,
            acc=acc,
            sync=sync,
        )
        return target_pulses


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

        # Move to +360° output (absolute)
        print("[demo] move to +360deg output (absolute) ...")
        motor.move_absolute_deg_output(+360.0, speed_rpm=200, acc=50)
        ok = motor.wait_until_reached(timeout_s=20.0)

        # Read back the position in both motor and output degrees
        motor_deg = motor.read_realtime_position_deg_motor()
        out_deg = motor.read_realtime_position_deg_output()
        print(
            f"[demo] commanded reached={ok} "
            f"motor_deg_now={motor_deg:.3f}  output_deg_now={out_deg:.3f}"
        )
        time.sleep(0.3)

        # Move back to 0° output (absolute)
        print("[demo] move back to 0deg output (absolute) ...")
        motor.move_absolute_deg_output(0.0, speed_rpm=200, acc=50)
        ok = motor.wait_until_reached(timeout_s=20.0)

        # Read back the position in both motor and output degrees
        motor_deg = motor.read_realtime_position_deg_motor()
        out_deg = motor.read_realtime_position_deg_output()
        print(
            f"[demo] commanded reached={ok} "
            f"motor_deg_now={motor_deg:.3f}  output_deg_now={out_deg:.3f}"
        )
        print("[demo] done.")

    finally:
        client.close()


if __name__ == "__main__":
    _demo()