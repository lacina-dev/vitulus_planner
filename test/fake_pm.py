"""Power module model: the sleep logic of powermodulev3_1 src/main.cpp, 1:1.

set_robot_sleep (1657), set_sleep_until_charged (1679), check_sleep (1135),
set_sleep_time & co. (EEPROM), get_charger_status (CHARGED edge restarts the
offset timer). Times in ms, like millis(). No ROS here.
"""
STARTUP, RUNNING, STANDBY = 'STARTUP', 'RUNNING', 'STANDBY'


class Status(object):
    """The fields of vitulus_msgs/Power_status the master reads."""


class FirmwareModel(object):
    def __init__(self, now_ms=0):
        self.t = now_ms
        self.module_status = RUNNING
        self.supply_status = 'ONLINE'
        self.charger_status = 'CHARGING'
        self.capacity = 80.0
        self.charge_pct_per_ms = 100.0 / (6 * 3600 * 1000.0)
        self.contact = True
        self.wait_ms = 15000
        self.sleep_ms = 28800000
        self.offset_ms = 120000
        self.sleep_status = False
        self.sleep_charged_status = False
        self.sleep_time_timer = 0
        self.wait_timer = 0
        self.charged_timer = 0
        self.eeprom_writes = 0
        self.log = []

    # --- commands ---------------------------------------------------------
    def set_robot_sleep(self, on):
        if on:
            self.sleep_status = True
            self.sleep_charged_status = False
            self.sleep_time_timer = self.t
            self.wait_timer = self.t
        else:
            self.sleep_status = False

    def set_sleep_until_charged(self, on):
        if on:
            self.sleep_charged_status = True
            self.sleep_status = False
            self.wait_timer = self.t
            self.charged_timer = self.t
        else:
            self.sleep_charged_status = False

    def _eeprom(self, name, value):
        # EEPROM.put -> update: unchanged bytes are not written.
        if getattr(self, name) != value:
            self.eeprom_writes += 1
        setattr(self, name, value)

    def set_sleep_time(self, ms):
        self._eeprom('sleep_ms', ms & 0xFFFFFFFF)   # unsigned long

    def set_sleep_wait_before_standby(self, ms):
        self._eeprom('wait_ms', ms & 0xFFFFFFFF)

    def set_sleep_wait_charged_offset(self, ms):
        self._eeprom('offset_ms', ms & 0xFFFFFFFF)

    def power_button(self):
        """Button 2-4 s: fw:577. check_sleep undoes it in the next loop."""
        if self.module_status != RUNNING:
            self.module_status = RUNNING
            self.log.append('button_on')

    # --- loop -------------------------------------------------------------
    def _charge(self, dt):
        if not self.contact:
            self.supply_status = 'OFFLINE'
            self.charger_status = 'DISCHARGING'
            return
        self.supply_status = 'ONLINE'
        if self.charger_status == 'CHARGED':
            return
        self.charger_status = 'CHARGING'
        self.capacity = min(100.0, self.capacity + dt * self.charge_pct_per_ms)
        if self.capacity >= 100.0:
            self.charger_status = 'CHARGED'
            self.charged_timer = self.t

    def tick(self, now_ms):
        dt = now_ms - self.t
        self.t = now_ms
        self._charge(dt)
        self._check_sleep()

    def _standby(self, label):
        if self.module_status == RUNNING:
            self.module_status = STANDBY
            self.log.append('cut:' + label)

    def _poweron(self, label):
        self.module_status = RUNNING
        self.log.append('wake:' + label)

    def _check_sleep(self):
        if self.sleep_status:
            if self.t - self.wait_timer >= self.wait_ms:
                self._standby('timed')
            if self.t - self.sleep_time_timer >= self.sleep_ms:
                self.sleep_status = False
                self._poweron('timed')
        if self.sleep_charged_status:
            if self.t - self.wait_timer >= self.wait_ms:
                self._standby('charged')
            if self.charger_status == 'CHARGED':
                if self.t - self.charged_timer >= self.offset_ms:
                    self._poweron('charged')
                    self.sleep_charged_status = False

    # --- telemetry --------------------------------------------------------
    @property
    def nuc_on(self):
        return self.module_status == RUNNING

    def status(self):
        s = Status()
        s.module_status = self.module_status
        s.supply_status = self.supply_status
        s.charger_status = self.charger_status
        s.battery_capacity = int(self.capacity)
        s.sleep_status = self.sleep_status
        s.sleep_charged_status = self.sleep_charged_status
        s.sleep_wait_standby = self.wait_ms
        s.sleep_time_interval = self.sleep_ms
        s.sleep_wait_charged = self.offset_ms
        return s
