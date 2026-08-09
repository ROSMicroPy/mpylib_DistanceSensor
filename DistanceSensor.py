"""Generic distance-sensor interface for MicroPython and host simulation."""

try:
    import importlib
except ImportError:
    import uimportlib as importlib

try:
    import threading
except ImportError:
    threading = None


class DistanceSensorDriver:
    def initialize(self, **config):
        raise NotImplementedError

    def read_distance_mm(self):
        raise NotImplementedError

    def set_active(self, active):
        raise NotImplementedError

    def shutdown(self):
        return self.set_active(False)

    def get_status(self):
        return {}


class DistanceSensor:
    def __init__(self, name, driver):
        self.name = name
        self.driver = driver
        self.active = False
        self.alerts = []
        self.last_distance_mm = None
        self.last_poll_error = None
        self._polling = False
        self._poll_thread = None

    def initialize(self, poll_frequency_hz=0, **config):
        if poll_frequency_hz < 0:
            raise ValueError("poll_frequency_hz cannot be negative")
        self.active = bool(self.driver.initialize(**config))
        if not self.active:
            return False
        if poll_frequency_hz:
            self.start_polling(poll_frequency_hz)
        return True

    def read_distance(self):
        if not self.active:
            raise RuntimeError("Distance sensor {} is inactive".format(self.name))
        distance = int(round(self.driver.read_distance_mm()))
        self.last_distance_mm = distance
        self._evaluate_alerts(distance)
        return distance

    read_distance_mm = read_distance

    def add_alert(self, target_mm, callback, tolerance_mm=0):
        if tolerance_mm < 0:
            raise ValueError("tolerance_mm cannot be negative")
        self.alerts.append({"target": int(target_mm), "tolerance": int(tolerance_mm),
                            "callback": callback})

    def remove_alert(self, callback):
        self.alerts = [alert for alert in self.alerts if alert["callback"] is not callback]

    def _evaluate_alerts(self, distance):
        retained = []
        for alert in self.alerts:
            if abs(distance - alert["target"]) <= alert["tolerance"]:
                if alert["callback"](distance) is True:
                    retained.append(alert)
            else:
                retained.append(alert)
        self.alerts = retained

    def set_active(self, active):
        if not active:
            self.stop_polling()
        result = bool(self.driver.set_active(bool(active)))
        if result:
            self.active = bool(active)
        return result

    def start_polling(self, frequency_hz):
        if threading is None:
            raise RuntimeError("background polling is unavailable; poll from an OnEvent task")
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        if self._polling:
            return
        self._polling = True
        interval = 1.0 / float(frequency_hz)

        def poll():
            import time
            while self._polling:
                try:
                    self.read_distance()
                    self.last_poll_error = None
                except Exception as error:
                    self.last_poll_error = error
                time.sleep(interval)

        self._poll_thread = threading.Thread(target=poll, daemon=True)
        self._poll_thread.start()

    def stop_polling(self):
        self._polling = False
        if self._poll_thread and self._poll_thread is not threading.current_thread():
            self._poll_thread.join(timeout=1)
        self._poll_thread = None

    def shutdown(self):
        self.stop_polling()
        result = bool(self.driver.shutdown())
        self.active = False
        return result

    def get_status(self):
        status = self.driver.get_status()
        status.update(name=self.name, active=self.active,
                      last_distance_mm=self.last_distance_mm)
        return status


class DistanceSensorController:
    def __init__(self, driver_package="distance_drivers"):
        self.driver_package = driver_package
        self.sensors = {}
        self._driver_cache = {}

    def _load_driver(self, driver_name):
        if driver_name not in self._driver_cache:
            module = importlib.import_module("{}.{}".format(self.driver_package, driver_name))
            driver_class = getattr(module, "DRIVER_CLASS", None)
            if driver_class is None:
                raise ImportError("{}.{} does not export DRIVER_CLASS".format(self.driver_package, driver_name))
            self._driver_cache[driver_name] = driver_class
        return self._driver_cache[driver_name]

    def create_sensor(self, name, driver_name=None, driver_class=None, **config):
        if name in self.sensors:
            raise ValueError("Sensor {} already exists".format(name))
        if driver_class is None:
            if not driver_name:
                raise ValueError("driver_name or driver_class is required")
            driver_class = self._load_driver(driver_name)
        sensor = DistanceSensor(name, driver_class())
        if not sensor.initialize(**config):
            raise RuntimeError("Driver failed to initialize sensor {}".format(name))
        self.sensors[name] = sensor
        return sensor

    def get_sensor(self, name):
        return self.sensors.get(name)

    def remove_sensor(self, name):
        sensor = self.sensors.pop(name, None)
        return sensor.shutdown() if sensor else False

    def shutdown(self):
        success = True
        for sensor in tuple(self.sensors.values()):
            try:
                success = sensor.shutdown() and success
            except Exception:
                success = False
        self.sensors.clear()
        return success
