# distance_sensor.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class DistanceReading:
    """Convenience container for one sensor read."""
    regs: list[int]          # raw 16-bit register values
    raw_u32: int             # combined 32-bit unsigned
    mm: int                  # interpreted as millimeters
    m: float                 # interpreted as meters


class DistanceSensor:
    """
    JK-LRD distance sensor wrapper.

    - Consumes an *already connected* pymodbus ModbusSerialClient (or any object
      that provides read_holding_registers(...)).
    - Does not open/close the port.
    - Lets timeouts/errors bubble up as pymodbus error results or exceptions,
      so upper-level software can handle failures gracefully.
    """

    def __init__(
        self,
        modbus_client,
        slave_id: int = 1,
        start_reg: int = 0x0000,
        reg_count: int = 2,
        mm_scale: float = 1.0,   # 1.0 means raw == mm (matches your test)
    ):
        self.client = modbus_client
        self.slave_id = slave_id
        self.start_reg = start_reg
        self.reg_count = reg_count
        self.mm_scale = mm_scale

    @staticmethod
    def _regs_to_u32(regs: list[int]) -> int:
        if len(regs) != 2:
            raise ValueError(f"Expected 2 registers, got {len(regs)}")
        return ((regs[0] & 0xFFFF) << 16) | (regs[1] & 0xFFFF)

    def read_registers(self) -> list[int]:
        """
        Returns a list of reg_count 16-bit registers.

        Raises:
          - Timeout / serial exceptions depending on transport
          - RuntimeError if pymodbus returns an error response
        """
        rr = self.client.read_holding_registers(
            address=self.start_reg,
            count=self.reg_count,
            slave=self.slave_id,
        )
        if rr.isError():
            # This covers timeouts and Modbus exception responses.
            # Keep it explicit so upper-level can catch and decide.
            raise RuntimeError(f"Modbus read error: {rr!r}")
        return list(rr.registers)

    def read_raw_u32(self) -> int:
        """Reads 2 regs and combines into a 32-bit unsigned int."""
        regs = self.read_registers()
        return self._regs_to_u32(regs)

    def read_mm(self) -> int:
        """
        Returns distance in millimeters as int.

        Your device currently returns regs=[0, 1973] => 1973 mm.
        If your unit ever changes (e.g., 0.1mm), adjust mm_scale.
        """
        raw_u32 = self.read_raw_u32()
        mm = int(round(raw_u32 * self.mm_scale))
        return mm

    def read_m(self) -> float:
        """Returns distance in meters as float."""
        return self.read_mm() / 1000.0

    def read(self) -> DistanceReading:
        """One-shot read returning all representations."""
        regs = self.read_registers()

        # If reg_count != 2, we still provide something sensible
        raw_u32 = self._regs_to_u32(regs) if len(regs) == 2 else 0
        mm = int(round(raw_u32 * self.mm_scale))
        m = mm / 1000.0
        return DistanceReading(regs=regs, raw_u32=raw_u32, mm=mm, m=m)


# --- Optional small demo (does not own the bus; expects external setup) ---
def demo(modbus_client, slave_id: int = 1, n: int = 10):
    sensor = DistanceSensor(modbus_client, slave_id=slave_id)
    for _ in range(n):
        r = sensor.read()
        print(f"regs={r.regs} raw_u32={r.raw_u32} mm={r.mm} m={r.m:.3f}")


 # ----------------------------------------------------------------------
# Demo mode (only runs if file executed directly)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import time
    from pymodbus.client import ModbusSerialClient

    PORT = "COM3"
    BAUD = 115200
    SLAVE_ID = 1

    print("DistanceSensor demo mode")
    print(f"Opening Modbus RTU on {PORT} @ {BAUD}...")

    client = ModbusSerialClient(
        port=PORT,
        baudrate=BAUD,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=0.4,
    )

    if not client.connect():
        raise RuntimeError("Failed to open serial port")

    sensor = DistanceSensor(client, slave_id=SLAVE_ID)

    try:
        while True:
            r = sensor.read()
            print(f"regs={r.regs}  raw={r.raw_u32}  {r.mm} mm  ({r.m:.3f} m)")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping demo")

    finally:
        client.close()
        print("Serial port closed")