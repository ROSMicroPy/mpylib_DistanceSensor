"""Minimal VL53L4CD ULD port using MicroPython's native I2C API.

Register values and initialization sequence are derived from STMicroelectronics'
VL53L4CD Ultra Lite Driver. No CircuitPython ``I2CDevice`` dependency is used.
"""

import time


DEFAULT_CONFIGURATION = bytes((
    0x00, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10,
    0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21, 0x00,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF,
    0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05,
    0x00, 0xA0, 0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00, 0x0F,
    0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05,
    0x06, 0x06, 0x00, 0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00,
))


class VL53L4CD:
    MODEL_ID = 0xEBAA

    def __init__(self, i2c, address=0x29, io_timeout_ms=1000):
        if not 0x08 <= address <= 0x77:
            raise ValueError("address must be a 7-bit I2C address")
        self.i2c = i2c
        self.address = int(address)
        self.io_timeout_ms = int(io_timeout_ms)
        self.ranging = False

    def _write(self, register, data):
        payload = bytes(((register >> 8) & 0xFF, register & 0xFF)) + bytes(data)
        self.i2c.writeto(self.address, payload)

    def _read(self, register, length=1):
        pointer = bytes(((register >> 8) & 0xFF, register & 0xFF))
        self.i2c.writeto(self.address, pointer, False)
        if hasattr(self.i2c, "readfrom"):
            return self.i2c.readfrom(self.address, length)
        data = bytearray(length)
        self.i2c.readfrom_into(self.address, data)
        return data

    def _read_u8(self, register):
        return self._read(register, 1)[0]

    def _read_u16(self, register):
        data = self._read(register, 2)
        return (data[0] << 8) | data[1]

    def _read_u32(self, register):
        data = self._read(register, 4)
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]

    def _write_u8(self, register, value):
        self._write(register, bytes((value & 0xFF,)))

    def _write_u16(self, register, value):
        self._write(register, bytes(((value >> 8) & 0xFF, value & 0xFF)))

    def _write_u32(self, register, value):
        self._write(register, bytes(((value >> 24) & 0xFF, (value >> 16) & 0xFF,
                                     (value >> 8) & 0xFF, value & 0xFF)))

    def _wait_until(self, predicate, message):
        start = time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.monotonic() * 1000)
        while not predicate():
            time.sleep_ms(1) if hasattr(time, "sleep_ms") else time.sleep(0.001)
            now = time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.monotonic() * 1000)
            elapsed = time.ticks_diff(now, start) if hasattr(time, "ticks_diff") else now - start
            if elapsed >= self.io_timeout_ms:
                raise TimeoutError(message)

    def initialize(self, timing_budget_ms=50, inter_measurement_ms=0):
        if self._read_u16(0x010F) != self.MODEL_ID:
            raise RuntimeError("VL53L4CD model ID did not match 0xEBAA")
        self._wait_until(lambda: self._read_u8(0x00E5) == 0x03,
                         "VL53L4CD boot timed out")
        self._write(0x002D, DEFAULT_CONFIGURATION)
        self._write_u8(0x0087, 0x40)
        self._wait_until(lambda: self.data_ready, "VL53L4CD VHV start timed out")
        self.clear_interrupt()
        self.stop_ranging()
        self._write_u8(0x0008, 0x09)
        self._write_u8(0x000B, 0x00)
        self._write_u16(0x0024, 0x0500)
        self.set_range_timing(timing_budget_ms, inter_measurement_ms)
        return True

    @property
    def data_ready(self):
        polarity = 0 if (self._read_u8(0x0030) & 0x10) else 1
        return (self._read_u8(0x0031) & 0x01) == polarity

    @property
    def distance_mm(self):
        return self._read_u16(0x0096)

    @property
    def range_status(self):
        raw = self._read_u8(0x0089) & 0x1F
        statuses = (255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255,
                    9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12)
        return statuses[raw] if raw < len(statuses) else 255

    def read_result(self):
        return {
            "range_status": self.range_status,
            "distance_mm": self._read_u16(0x0096),
            "sigma_mm": self._read_u16(0x0092) // 4,
            "signal_rate_kcps": self._read_u16(0x008E) * 8,
            "ambient_rate_kcps": self._read_u16(0x0090) * 8,
        }

    def set_range_timing(self, timing_budget_ms, inter_measurement_ms=0):
        timing_budget_ms = int(timing_budget_ms)
        inter_measurement_ms = int(inter_measurement_ms)
        if not 10 <= timing_budget_ms <= 200:
            raise ValueError("timing budget must be between 10 and 200 ms")
        if inter_measurement_ms and inter_measurement_ms <= timing_budget_ms:
            raise ValueError("inter-measurement period must exceed timing budget")
        osc_frequency = self._read_u16(0x0006)
        if not osc_frequency:
            raise RuntimeError("VL53L4CD oscillator frequency is zero")
        timing_us = timing_budget_ms * 1000
        macro_period_us = (2304 * (0x40000000 // osc_frequency)) >> 6
        if inter_measurement_ms == 0:
            self._write_u32(0x006C, 0)
            timing_us -= 2500
        else:
            clock_pll = self._read_u16(0x00DE) & 0x3FF
            self._write_u32(0x006C, int(1.055 * inter_measurement_ms * clock_pll))
            timing_us = (timing_us - 4300) // 2
        self._write_u16(0x005E, self._encode_timeout(timing_us, macro_period_us * 16))
        self._write_u16(0x0061, self._encode_timeout(timing_us, macro_period_us * 12))

    @staticmethod
    def _encode_timeout(timing_us, macro_period):
        timing_us <<= 12
        denominator = macro_period >> 6
        value = ((timing_us + (denominator >> 1)) // denominator) - 1
        exponent = 0
        while value & 0xFFFFFF00:
            value >>= 1
            exponent += 1
        return (exponent << 8) | (value & 0xFF)

    def start_ranging(self):
        self._write_u8(0x0087, 0x21 if self._read_u32(0x006C) == 0 else 0x40)
        self.ranging = True

    def stop_ranging(self):
        self._write_u8(0x0087, 0x80)
        self.ranging = False

    def clear_interrupt(self):
        self._write_u8(0x0086, 0x01)

    def set_address(self, new_address):
        if not 0x08 <= new_address <= 0x77:
            raise ValueError("address must be a 7-bit I2C address")
        self._write_u8(0x0001, new_address)
        self.address = int(new_address)
