# Example program for the T-WATCH S3

import random

from machine import Pin, SoftSPI
import st7789py as st7789
import time
from axp2101 import AXP2101
from haptic_motor import HAPTIC_MOTOR 

def main():
    # Setup the PMU chip.
    twatch_pmu = AXP2101()
    twatch_pmu.twatch_s3_poweron()
    print("[AXP2101] Battery voltage is", twatch_pmu.get_battery_voltage())

    # Power on the display backlight.
    bl = Pin(45,Pin.OUT)
    bl.on()

    sck_pin = Pin(18)
    mosi_pin = Pin(13)
    # Our display does not have a MISO pin, but the MicroPython
    # SoftSPI implementation does not allow to avoid specifying one, so
    # we use just a not used pin in the device.
    notused_miso_pin = Pin(37)

    spi = SoftSPI(baudrate=40000000, polarity=1, sck=sck_pin, mosi=mosi_pin, miso=notused_miso_pin)
    display = st7789.ST7789(
        spi, 240, 240,
        reset=False,
        dc=Pin(38, Pin.OUT),
        cs=Pin(12, Pin.OUT),
    )
    display.init()

    # vibrate using effect 14
    motor = HAPTIC_MOTOR(14)	
    motor.vibrate()	    
    

    print("displaying random colors")
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
