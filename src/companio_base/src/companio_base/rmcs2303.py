"""
Rhino RMCS-2303 encoder motor driver (Modbus ASCII over USB-UART).

The RMCS-2303 exposes a small holding-register map.  Every register write is
serialised through a lock so that a velocity command and an encoder read can
never be interleaved on the same serial port.

Register / command values follow the RMCS-2303 datasheet.
"""

import logging
import threading
import time

import minimalmodbus

LOG = logging.getLogger("rmcs2303")

# ---- Holding registers -------------------------------------------------------
REG_MODE = 2
REG_POSITION_P_GAIN = 4
REG_POSITION_I_GAIN = 6
REG_VELOCITY_FF_GAIN = 8
REG_LINES_PER_ROTATION = 10
REG_ACCELERATION = 12
REG_SPEED_RPM = 14
REG_POSITION_CMD_LSB = 16
REG_POSITION_CMD_MSB = 18
REG_POSITION_FB_LSB = 20
REG_POSITION_FB_MSB = 22
REG_SPEED_FB = 24

# ---- Mode-register commands --------------------------------------------------
CMD_ENABLE_CW = 257
CMD_DISABLE_CW = 256
CMD_BRAKE_CW = 260
CMD_ENABLE_CCW = 265
CMD_DISABLE_CCW = 264
CMD_BRAKE_CCW = 268
CMD_HOME = 2048          # zero the encoder counter
CMD_EMERGENCY_STOP = 1792
CMD_STOP = 1793

FC_WRITE_REGISTER = 6
FC_READ_REGISTERS = 3

SPEED_RPM_MIN = 0
SPEED_RPM_MAX = 65535
ACCEL_MIN = 0
ACCEL_MAX = 65535

RAD_S_TO_RPM = 9.549297


class Rmcs2303:
    """Thread-safe driver for one RMCS-2303 controller on one serial port."""

    CW = 1
    IDLE = 0
    CCW = -1

    def __init__(self, port, slave_id,
                 baudrate=9600,
                 lines_per_rotation=334,
                 gear_ratio=100.0,
                 acceleration=10000,
                 initial_speed_rpm=0,
                 io_delay=0.1,
                 retries=5):
        self.port = port
        self.slave_id = slave_id
        self.name = port.rstrip("/").split("/")[-1]
        self.gear_ratio = float(gear_ratio)
        self.io_delay = io_delay
        self.retries = retries
        self.direction = self.IDLE
        self._lock = threading.Lock()

        inst = minimalmodbus.Instrument(port, slave_id, minimalmodbus.MODE_ASCII)
        inst.serial.baudrate = baudrate
        inst.serial.parity = minimalmodbus.serial.PARITY_NONE
        inst.serial.bytesize = 8
        inst.serial.stopbits = 1
        inst.serial.timeout = 3.0
        inst.serial.write_timeout = 5.0
        inst.clear_buffers_before_each_transaction = True
        inst.close_port_after_each_call = False
        self._inst = inst

        # Controller initialisation sequence
        self.set_lines_per_rotation(lines_per_rotation)
        self.zero_encoder()
        self.set_acceleration(acceleration)
        self.set_speed_rpm(initial_speed_rpm)

    # ------------------------------------------------------------------ I/O --
    def _write(self, register, value):
        for attempt in range(1, self.retries + 1):
            try:
                with self._lock:
                    self._inst.write_register(register, int(value), 0, FC_WRITE_REGISTER)
                    time.sleep(self.io_delay)
                return True
            except Exception as exc:  # serial / modbus errors
                LOG.warning("%s: write reg %d failed (attempt %d/%d): %s",
                            self.name, register, attempt, self.retries, exc)
                time.sleep(self.io_delay)
        LOG.error("%s: giving up writing register %d", self.name, register)
        return False

    def _read_u32(self, register_lsb):
        for attempt in range(1, self.retries + 1):
            try:
                with self._lock:
                    lsb, msb = self._inst.read_registers(register_lsb, 2, FC_READ_REGISTERS)
                    time.sleep(self.io_delay)
                return (msb << 16) | lsb
            except Exception as exc:
                LOG.warning("%s: read reg %d failed (attempt %d/%d): %s",
                            self.name, register_lsb, attempt, self.retries, exc)
                time.sleep(self.io_delay)
        return None

    # ------------------------------------------------------------ helpers ----
    @classmethod
    def rad_s_to_rpm(cls, rad_s, gear_ratio):
        return rad_s * RAD_S_TO_RPM * gear_ratio

    # ----------------------------------------------------------- commands ----
    def set_lines_per_rotation(self, lines):
        return self._write(REG_LINES_PER_ROTATION, lines)

    def zero_encoder(self):
        return self._write(REG_MODE, CMD_HOME)

    def set_acceleration(self, rpm_per_s):
        rpm_per_s = max(ACCEL_MIN, min(ACCEL_MAX, int(rpm_per_s)))
        return self._write(REG_ACCELERATION, rpm_per_s)

    def set_speed_rpm(self, rpm):
        rpm = max(SPEED_RPM_MIN, min(SPEED_RPM_MAX, abs(int(rpm))))
        return self._write(REG_SPEED_RPM, rpm)

    def set_speed(self, rad_s):
        """Set the speed target from a shaft speed in rad/s (sign ignored)."""
        return self.set_speed_rpm(self.rad_s_to_rpm(abs(rad_s), self.gear_ratio))

    def enable_cw(self):
        ok = self._write(REG_MODE, CMD_ENABLE_CW)
        self.direction = self.CW
        return ok

    def enable_ccw(self):
        ok = self._write(REG_MODE, CMD_ENABLE_CCW)
        self.direction = self.CCW
        return ok

    def brake(self):
        """Brake in the direction currently active (no-op when idle)."""
        if self.direction == self.CW:
            ok = self._write(REG_MODE, CMD_BRAKE_CW)
        elif self.direction == self.CCW:
            ok = self._write(REG_MODE, CMD_BRAKE_CCW)
        else:
            ok = True
        self.direction = self.IDLE
        return ok

    def stop(self):
        ok = self._write(REG_MODE, CMD_STOP)
        self.direction = self.IDLE
        return ok

    def emergency_stop(self):
        ok = self._write(REG_MODE, CMD_EMERGENCY_STOP)
        self.direction = self.IDLE
        return ok

    # ----------------------------------------------------------- feedback ----
    def read_encoder(self):
        """Raw 32-bit unsigned encoder count, or None if the read failed."""
        return self._read_u32(REG_POSITION_FB_LSB)

    def read_encoder_signed(self):
        raw = self.read_encoder()
        if raw is None:
            return None
        return raw - 2147483648
