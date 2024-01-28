# code for micropython 1.10 on esp8266

import random

from machine import Pin, SoftSPI
import st7789py as st7789
import time
import axp2101

def main():
    axp2101.pmu_setup()
    bl = Pin(45,Pin.OUT)
    bl.on()

    sck_pin = Pin(18)
    mosi_pin = Pin(13)

    spi = SoftSPI(baudrate=40000000, polarity=1, sck=sck_pin, mosi=mosi_pin, miso=Pin(16))
    display = st7789.ST7789(
        spi, 240, 240,
        reset=False,
        dc=Pin(38, Pin.OUT),
        cs=Pin(12, Pin.OUT),
    )
    display.init()

    while True:
        display.fill(
            st7789.color565(
                random.getrandbits(8),
                random.getrandbits(8),
                random.getrandbits(8),
            ),
        )
        # Pause 2 seconds.
        time.sleep(2)

main()
