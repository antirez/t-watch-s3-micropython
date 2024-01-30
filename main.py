# Example program for the T-WATCH S3

import random

from machine import Pin, SPI
import st7789py as st7789
import time
from axp2101 import AXP2101

def main():
    # Setup the PMU chip.
    twatch_pmu = AXP2101()
    twatch_pmu.twatch_s3_poweron()
    print("[AXP2101] Battery voltage is", twatch_pmu.get_battery_voltage())

    # Power on the display backlight.
    bl = Pin(45,Pin.OUT)
    bl.on()

    # Our display does not have a MISO pin, but the MicroPython
    # SPI implementation does not allow to avoid specifying one, so
    # we use just a not used pin in the device.
    spi = SPI(1, baudrate=40000000, polarity=1, sck=18, mosi=13, miso=37)
    display = st7789.ST7789(
        spi, 240, 240,
        reset=False,
        dc=Pin(38, Pin.OUT),
        cs=Pin(12, Pin.OUT),
    )
    display.init()

    while True:
        start = time.ticks_ms()
        display.fill(
            display.color565(
                random.getrandbits(8),
                random.getrandbits(8),
                random.getrandbits(8),
            ),
        )
        for i in range(50):
            display.pixel(random.randint(0,240),
                          random.randint(0,240),
                          display.color565(255,255,255))
        display.show()
        elapsed = time.ticks_ms() - start
        print("Ticks per screen fill:", elapsed)

        # Pause 2 seconds.
        time.sleep(2)

main()
