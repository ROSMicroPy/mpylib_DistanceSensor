"""HC-SR04 compatible ultrasonic distance driver."""

try:
    from time import sleep_us
except ImportError:
    from time import sleep

    def sleep_us(value):
        sleep(value / 1000000.0)

from DistanceSensor import DistanceSensorDriver


def _write(pin, value):
    try:
        pin.value(value)
    except AttributeError:
        pin(value)


class HCSR04Driver(DistanceSensorDriver):
    def __init__(self):
        self.active = False
        self.trigger_pin = self.echo_pin = None

    def initialize(self, trigger_pin, echo_pin, pin_factory=None,
                   time_pulse_us=None, timeout_us=30000,
                   temperature_c=20.0, **_):
        if timeout_us <= 0:
            raise ValueError("timeout_us must be positive")
        output_mode, input_mode = "out", "in"
        if pin_factory is None and (isinstance(trigger_pin, int) or isinstance(echo_pin, int)):
            from machine import Pin
            pin_factory = lambda number, mode: Pin(number, mode)
            output_mode, input_mode = Pin.OUT, Pin.IN
        if time_pulse_us is None:
            from machine import time_pulse_us as machine_time_pulse_us
            time_pulse_us = machine_time_pulse_us
        self.trigger_pin = pin_factory(trigger_pin, output_mode) if isinstance(trigger_pin, int) else trigger_pin
        self.echo_pin = pin_factory(echo_pin, input_mode) if isinstance(echo_pin, int) else echo_pin
        self.time_pulse_us = time_pulse_us
        self.timeout_us = int(timeout_us)
        self.speed_mm_per_us = (331.3 + 0.606 * float(temperature_c)) / 1000.0
        _write(self.trigger_pin, 0)
        self.active = True
        return True

    def read_distance_mm(self):
        if not self.active:
            raise RuntimeError("ultrasonic sensor is inactive")
        _write(self.trigger_pin, 0)
        sleep_us(2)
        _write(self.trigger_pin, 1)
        sleep_us(10)
        _write(self.trigger_pin, 0)
        duration_us = self.time_pulse_us(self.echo_pin, 1, self.timeout_us)
        if duration_us < 0:
            raise TimeoutError("ultrasonic echo timed out")
        return duration_us * self.speed_mm_per_us / 2.0

    def set_active(self, active):
        self.active = bool(active)
        if not self.active and self.trigger_pin is not None:
            _write(self.trigger_pin, 0)
        return True

    def get_status(self):
        return {"active": self.active, "timeout_us": self.timeout_us}


DRIVER_CLASS = HCSR04Driver
