# ==========================================================
# Modbus RTU hardware Serial Interface
# Main App needs to create a single ModbusSerialClient and pass it to each device interface class
# ==========================================================

from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(
    port="COM3",
    baudrate=115200,
    parity="N",
    stopbits=1,
    bytesize=8,
    timeout=0.4
)

client.connect()

# ==========================================================
# Distance Sensor Interface
# File: distance_sensor.py
# Device: JK-LRD laser distance sensor
# Modbus slave address: 1
# Output unit: millimeters
# ==========================================================

from distance_sensor import DistanceSensor

sensor = DistanceSensor(client, slave_id=1)


# Returns distance measurement in millimeters (int)
sensor.read_mm()


# Returns distance measurement in meters (float)
sensor.read_m()


# Returns raw 32-bit value from sensor registers
sensor.read_raw_u32()


# Returns full reading structure containing raw registers and converted values
sensor.read()


# ==========================================================
# Motor Controller Interface
# File: motor_comm.py
# Device: ZDT closed-loop stepper motor (EMM firmware)
# Modbus slave address: 2
# Motor configuration used:
# 200 steps, 16 microsteps, 30:1 gearbox
# ==========================================================

from motor_comm import ZDTEmmMotor

motor = ZDTEmmMotor(client, slave_id=2)


# Returns internal motor position counter (signed 32-bit counts)
motor.read_realtime_position_u32_signed()


# Returns motor shaft angle in degrees
motor.read_realtime_position_deg_motor()


# Returns output shaft angle in degrees (after gearbox)
motor.read_realtime_position_deg_output()


# Returns output shaft position in revolutions
motor.read_realtime_position_rev_output()


# Returns motor status bitmask (driver enabled, motion complete flags)
motor.read_status_flags()


# Returns True if motor driver is enabled
motor.is_enabled()


# Returns True if the commanded move has finished
motor.is_reached()


# Sets the current motor position as zero
motor.zero_here()


# Moves the output shaft by a relative angle in degrees
deg = 45.0
motor.move_relative_deg_output(deg)


# Waits until the motor reports the move is completed
motor.wait_until_reached(timeout_s=10)



# ==========================================================
# System Notes
# ==========================================================

# All devices share the same RS485 Modbus RTU bus.

# The application must ensure that only one Modbus request is active at a time.

# The primary motor feedback value used by the application should be:
motor.read_realtime_position_deg_output()