import itertools
import sys
import threading
import unittest

sys.path.insert(0, ".")

from DistanceSensor import DistanceSensor, DistanceSensorDriver
from distance_drivers.hcsr04 import HCSR04Driver
from distance_drivers.vl53l4cd_core import DEFAULT_CONFIGURATION, VL53L4CD


class FakeDriver(DistanceSensorDriver):
    def __init__(self, readings):
        self.readings = iter(readings)
        self.active = False

    def initialize(self, **_):
        self.active = True
        return True

    def read_distance_mm(self):
        return next(self.readings)

    def set_active(self, active):
        self.active = bool(active)
        return True


class FakePin:
    def __init__(self):
        self.values = []

    def value(self, value):
        self.values.append(value)


class FakeI2C:
    def __init__(self):
        self.pointer = 0
        self.writes = []
        self.registers = {0x010F: b"\xeb\xaa"}

    def writeto(self, address, data, stop=True):
        self.writes.append((address, bytes(data), stop))
        self.pointer = (data[0] << 8) | data[1]
        if len(data) > 2:
            self.registers[self.pointer] = bytes(data[2:])

    def readfrom(self, address, length):
        return self.registers.get(self.pointer, bytes(length))[:length]


class DistanceSensorTests(unittest.TestCase):
    def make_sensor(self, readings):
        sensor = DistanceSensor("test", FakeDriver(readings))
        self.assertTrue(sensor.initialize())
        return sensor

    def test_read_rounds_to_integer_millimeters(self):
        self.assertEqual(self.make_sensor([12.6]).read_distance_mm(), 13)

    def test_alert_is_one_shot_unless_callback_returns_true(self):
        sensor = self.make_sensor([10, 10])
        calls = []
        sensor.add_alert(10, lambda distance: calls.append(distance))
        sensor.read_distance()
        sensor.read_distance()
        self.assertEqual(calls, [10])

    def test_background_polling(self):
        sensor = DistanceSensor("test", FakeDriver(itertools.repeat(10)))
        fired = threading.Event()
        sensor.add_alert(10, lambda _: fired.set())
        sensor.initialize(poll_frequency_hz=100)
        try:
            self.assertTrue(fired.wait(0.5))
        finally:
            sensor.shutdown()

    def test_hcsr04_converts_round_trip_time_to_distance(self):
        trigger = FakePin()
        driver = HCSR04Driver()
        driver.initialize(trigger, FakePin(), time_pulse_us=lambda *_: 1000)
        self.assertAlmostEqual(driver.read_distance_mm(), 171.71, places=2)
        self.assertEqual(trigger.values[-3:], [0, 1, 0])

    def test_vl53_uses_two_byte_register_pointer_and_native_i2c(self):
        i2c = FakeI2C()
        sensor = VL53L4CD(i2c)
        self.assertEqual(sensor._read_u16(0x010F), 0xEBAA)
        self.assertEqual(i2c.writes[-1], (0x29, b"\x01\x0f", False))
        self.assertEqual(len(DEFAULT_CONFIGURATION), 91)


if __name__ == "__main__":
    unittest.main()
