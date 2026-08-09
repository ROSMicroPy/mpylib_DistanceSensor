"""DistanceSensor adapter for the ST VL53L4CD time-of-flight sensor."""

import time

from DistanceSensor import DistanceSensorDriver
from DistanceSensor.distance_drivers.vl53l4cd_core import VL53L4CD


class VL53L4CDDriver(DistanceSensorDriver):
    def __init__(self):
        self.sensor = None
        self.active = False
        self.read_timeout_ms = 1000

    def initialize(self, i2c, address=0x29, timing_budget=50,
                   inter_measurement=0, read_timeout_ms=1000, **_):
        self.read_timeout_ms = int(read_timeout_ms)
        self.sensor = VL53L4CD(i2c, address, self.read_timeout_ms)
        self.sensor.initialize(timing_budget, inter_measurement)
        self.sensor.start_ranging()
        self.active = True
        return True

    def read_distance_mm(self):
        if not self.active:
            raise RuntimeError("VL53L4CD is inactive")
        start = time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.monotonic() * 1000)
        while not self.sensor.data_ready:
            time.sleep_ms(1) if hasattr(time, "sleep_ms") else time.sleep(0.001)
            now = time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.monotonic() * 1000)
            elapsed = time.ticks_diff(now, start) if hasattr(time, "ticks_diff") else now - start
            if elapsed >= self.read_timeout_ms:
                raise TimeoutError("VL53L4CD measurement timed out")
        result = self.sensor.read_result()
        self.sensor.clear_interrupt()
        if result["range_status"] != 0:
            raise RuntimeError("VL53L4CD invalid range status {}".format(result["range_status"]))
        return result["distance_mm"]

    def set_active(self, active):
        if self.sensor is None:
            self.active = False
            return not active
        if active and not self.active:
            self.sensor.start_ranging()
        elif not active and self.active:
            self.sensor.stop_ranging()
        self.active = bool(active)
        return True

    def shutdown(self):
        return self.set_active(False)

    def get_status(self):
        return {"active": self.active,
                "address": self.sensor.address if self.sensor else None,
                "sensor": "VL53L4CD"}


DRIVER_CLASS = VL53L4CDDriver
