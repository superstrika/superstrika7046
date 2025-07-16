"""
MPU6050 Gyroscope & Accelerometer Driver Class
Author: SuperStrika#7046 

This class provides an interface to the MPU6050 IMU sensor
via I2C. It allows reading raw temperature, acceleration,
and angular velocity (gyroscope) data. It also handles gyro
calibration and integrates angular velocity over time to
compute angular position (\u03b8). Sensitivity constants are set
for \u00b1250\u00b0/s and \u00b12g by default, and can be modified.

Functions:
  - get_temp(): returns temperature in Celsius.
  - get_accel(): returns acceleration in g (gravity units).
  - get_omega(): returns calibrated angular velocity in \u00b0/s.
  - get_theta(): returns integrated angle from gyro.
  - reset_theta(): resets the angle.
  - set_gyro_offset(offset): manually set gyro calibration.
  - reset_gyro_offset(): recalibrate gyro to current position.
  - set_gyro_sensitivity(factor): update gyro sensitivity.
  - set_accel_sensitivity(factor): update accel sensitivity.
  - update(): manually call to keep theta updated.
"""

import time

try:
    from machine import I2C
except ImportError:
    from smbus2 import SMBus as I2C  # For RPi compatibility



class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr

        self._PWR_MGMT_1 = 0x6B
        self._TEMP_OUT_H = 0x41
        self._ACCEL_XOUT_H = 0x3B
        self._GYRO_XOUT_H = 0x43

        try:
            self._write_register(self._PWR_MGMT_1, 0x00)  # Wake up sensor
        except Exception as e:
            raise RuntimeError("MPU6050 initialization failed: {}".format(e))

        time.sleep(0.1)

        self.omega_sensitivity_factor = 131.0  # deg/s
        self.accel_sensitivity_factor = 16384.0  # g

        self.theta = {'x': 0, 'y': 0, 'z': 0}
        self._theta_f = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_update = time.ticks_ms() if hasattr(time, 'ticks_ms') else int(time.time() * 1000)

        self.omega_offset = self._calibrate_gyro()

    def _write_register(self, reg, value):
        if hasattr(self.i2c, 'writeto_mem'):
            self.i2c.writeto_mem(self.addr, reg, bytes([value]))
        else:
            self.i2c.write_byte_data(self.addr, reg, value)

    def _read_register_pair(self, reg):
        try:
            if hasattr(self.i2c, 'readfrom_mem'):
                high = self.i2c.readfrom_mem(self.addr, reg, 1)[0]
                low = self.i2c.readfrom_mem(self.addr, reg + 1, 1)[0]
            else:
                high = self.i2c.read_byte_data(self.addr, reg)
                low = self.i2c.read_byte_data(self.addr, reg + 1)
            value = (high << 8) | low
            return value - 65536 if value > 32767 else value
        except Exception as e:
            raise IOError(f"I2C read failed at reg 0x{reg:02X}: {e}")

    def get_temp(self):
        raw = self._read_register_pair(self._TEMP_OUT_H)
        return raw / 340.0 + 36.53

    def get_accel(self):
        return {
            'x': self._read_register_pair(self._ACCEL_XOUT_H) / self.accel_sensitivity_factor,
            'y': self._read_register_pair(self._ACCEL_XOUT_H + 2) / self.accel_sensitivity_factor,
            'z': self._read_register_pair(self._ACCEL_XOUT_H + 4) / self.accel_sensitivity_factor
        }

    def get_omega(self):
        gx = self._read_register_pair(self._GYRO_XOUT_H) / self.omega_sensitivity_factor
        gy = self._read_register_pair(self._GYRO_XOUT_H + 2) / self.omega_sensitivity_factor
        gz = self._read_register_pair(self._GYRO_XOUT_H + 4) / self.omega_sensitivity_factor
        return {
            'x': gx - self.omega_offset['x'],
            'y': gy - self.omega_offset['y'],
            'z': gz - self.omega_offset['z']
        }

    def _calibrate_gyro(self, samples=200):
        print("Calibrating gyro (\u03c9), keep device still...")
        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        for _ in range(samples):
            omega = self.get_omega()
            for axis in offset:
                offset[axis] += omega[axis]
            time.sleep(0.01)
        offset = {axis: val / samples for axis, val in offset.items()}
        print("Calibration complete. \u03c9 offset:", offset)
        return offset

    def update(self):
        now = time.ticks_ms() if hasattr(time, 'ticks_ms') else int(time.time() * 1000)
        dt = (now - self.last_update) / 1000
        self.last_update = now

        omega = self.get_omega()
        for axis in ['x', 'y', 'z']:
            self._theta_f[axis] += omega[axis] * dt
            self.theta[axis] = int(self._theta_f[axis])

    def get_theta(self):
        self.update()
        return self.theta

    def reset_theta(self):
        self.theta = {'x': 0, 'y': 0, 'z': 0}
        self._theta_f = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_update = time.ticks_ms() if hasattr(time, 'ticks_ms') else int(time.time() * 1000)

    def reset_gyro_offset(self):
        self.omega_offset = self._calibrate_gyro()

    def set_gyro_offset(self, offset):
        if all(axis in offset for axis in ['x', 'y', 'z']):
            self.omega_offset = offset
        else:
            raise ValueError("Offset must include keys: 'x', 'y', and 'z'.")

    def set_gyro_sensitivity(self, factor):
        self.omega_sensitivity_factor = factor

    def set_accel_sensitivity(self, factor):
        self.accel_sensitivity_factor = factor
