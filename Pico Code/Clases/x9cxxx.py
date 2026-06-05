from machine import Pin
from time import sleep_us, sleep_ms


class X9Cxxx:
    """
    MicroPython driver for X9C102, X9C103, X9C104 and X9C503
    digital potentiometers.

    Pins:
        INC : increment input
        U/D : direction input
        CS  : chip select input

    Logical position:
        0  -> one end of the potentiometer
        99 -> opposite end of the potentiometer

    Important:
        The X9Cxxx has no position feedback. After power-up, this driver
        does not know the real wiper position unless you call reset().
    """

    MAX_VALUE = 99

    def __init__(self, inc_pin, ud_pin, cs_pin, pulse_us=5):
        self.inc = Pin(inc_pin, Pin.OUT)
        self.ud = Pin(ud_pin, Pin.OUT)
        self.cs = Pin(cs_pin, Pin.OUT)

        self.pulse_us = int(pulse_us)
        self.position = None

        self.inc.value(1)
        self.ud.value(0)
        self.cs.value(1)

    def _clip(self, value):
        value = int(value)
        if value < 0:
            return 0
        if value > self.MAX_VALUE:
            return self.MAX_VALUE
        return value

    def _pulse_inc(self):
        self.inc.value(0)
        sleep_us(self.pulse_us)
        self.inc.value(1)
        sleep_us(self.pulse_us)

    def change(self, direction, steps, store=False):
        """
        Move the wiper.

        Args:
            direction:
                1 -> increase
                0 -> decrease
            steps:
                Number of steps, from 0 to 99.
            store:
                False -> move wiper without storing in non-volatile memory.
                True  -> store final position in non-volatile memory.
        """

        steps = self._clip(steps)
        direction = 1 if direction else 0

        self.ud.value(direction)
        self.inc.value(1)
        self.cs.value(0)
        sleep_us(self.pulse_us)

        for _ in range(steps):
            self._pulse_inc()

            if self.position is not None:
                if direction:
                    self.position += 1
                else:
                    self.position -= 1

                self.position = self._clip(self.position)

        if store:
            # According to the X9Cxxx timing logic, raising CS while INC is HIGH
            # stores the current wiper position in non-volatile memory.
            self.inc.value(1)
            sleep_us(self.pulse_us)
            self.cs.value(1)
            sleep_ms(20)
        else:
            # Raising CS while INC is LOW changes the wiper position
            # without storing it in non-volatile memory.
            self.inc.value(0)
            sleep_us(self.pulse_us)
            self.cs.value(1)
            sleep_us(self.pulse_us)
            self.inc.value(1)

    def increase(self, steps=1, store=False):
        self.change(direction=1, steps=steps, store=store)

    def decrease(self, steps=1, store=False):
        self.change(direction=0, steps=steps, store=store)

    def reset(self, store=False):
        """
        Force the wiper to the lower end.

        Since there is no position feedback, this sends 99 downward pulses.
        """
        self.change(direction=0, steps=self.MAX_VALUE, store=store)
        self.position = 0

    def set(self, value, store=False):
        """
        Set logical position from 0 to 99.

        If the current position is unknown, the driver first calls reset().
        """
        value = self._clip(value)

        if self.position is None:
            self.reset(store=False)

        if value > self.position:
            self.increase(value - self.position, store=store)
        elif value < self.position:
            self.decrease(self.position - value, store=store)

        self.position = value

    def get(self):
        """
        Return the logical position known by the driver.

        Returns None if reset() or set() has not been called yet.
        """
        return self.position

    def save(self):
        """
        Store the current wiper position in non-volatile memory.
        """
        self.inc.value(1)
        self.cs.value(0)
        sleep_us(self.pulse_us)
        self.cs.value(1)
        sleep_ms(20)


def resistance_from_position(position, r_total_ohm=10000, r_wiper_ohm=40):
    """
    Approximate resistance between wiper and low terminal.

    Args:
        position:
            Integer from 0 to 99.
        r_total_ohm:
            Nominal end-to-end resistance.
            Example:
                X9C102 -> 1 kOhm
                X9C103 -> 10 kOhm
                X9C104 -> 100 kOhm
                X9C503 -> 50 kOhm
        r_wiper_ohm:
            Approximate internal wiper resistance.

    Returns:
        Approximate resistance in ohms.
    """
    position = int(position)
    if position < 0:
        position = 0
    if position > 99:
        position = 99

    return r_wiper_ohm + (position / 99) * r_total_ohm
