# DistanceSensor

Generic MicroPython distance-sensor abstraction with independently selectable
hardware drivers. It contains no system-specific positions or behavior.

Included drivers:

- `hcsr04`: trigger/echo ultrasonic sensors, with temperature-adjusted sound speed.
- `vl53l4cd`: ST VL53L4CD time-of-flight sensor using native MicroPython `I2C`.

```python
from DistanceSensor import DistanceSensorController

sensors = DistanceSensorController()
tof = sensors.create_sensor(
    "position",
    "vl53l4cd",
    i2c=i2c,
    address=0x29,
    timing_budget=50,
)
print(tof.read_distance_mm())
sensors.shutdown()
```

The VL53L4CD protocol is based on STMicroelectronics' Ultra Lite Driver and the
earlier Adafruit-derived port retained under `stuff/other-repos`. It performs
direct `writeto`/`readfrom` calls and has no CircuitPython `I2CDevice` dependency.

`component.yaml` defines the framework interface and driver configuration.
`package.json` is directly installable by MicroPython `mip`.
