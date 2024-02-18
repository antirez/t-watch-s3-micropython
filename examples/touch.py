# Example program for the T-WATCH S3

import random

from machine import Pin, SPI, SoftI2C
import st7789_base, st7789_ext
import time
from axp2101 import AXP2101
from haptic_motor import HAPTIC_MOTOR 
from ft6x06 import FT6206

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
display = st7789_ext.ST7789(
    SPI(1, baudrate=40000000, phase=0, polarity=1, sck=18, mosi=13, miso=37),
    240, 240,
    reset=False,
    dc=Pin(38, Pin.OUT),
    cs=Pin(12, Pin.OUT),
)
display.init(landscape=False,mirror_y=True,mirror_x=True,inversion=True)

# Setup touch driver
def data_available(data):
    display.pixel(data[0]['x'],data[0]['y'],display.color(255,0,0))
    if len(data) == 2:
        display.pixel(data[1]['x'],data[1]['y'],display.color(0,255,0))

i2c = SoftI2C(scl=40,sda=39)
ft = FT6206(i2c,interrupt_pin=Pin(16,Pin.IN),callback=data_available)
while True: time.sleep(1)
