# MicroPython programming of Lilygo T-WATCH S3.

This repositry contains the minimal stuff needed to program the T-WATCH
S3 in MicroPython. For now there is code to:

1. Setup the AX2101 power manager chip in order to give current to the different subsystems, enable the TFT display backlight, setup charging, and so forth.
2. Configure SPI correctly in order to use a MicroPython ST7789 display driver that was already [available here](https://github.com/devbis/st7789py_mpy).

DISCLAIMER: I wrote this code based on the available information and the
C implementation of Lilygo. However this code is not well tested and may
ruin your device. Use it at your own risk.

## Installing MicroPython

**WARNING:** after installing MicroPython you will no longer be able to flash the device with `esptools` if you don't press the *boot* button inside the device, accessible under the battery, [as explained here](https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library/issues/223#issuecomment-1913183156).

I just used the generic MicroPython release for the S3 device.
The file name is `ESP32_GENERIC_S3-20231005-v1.21.0.bin`.

```
esptool.py --chip esp32s3 --port /dev/tty.usbmodem* erase_flash
esptool.py --chip esp32s3 --port /dev/tty.usbmodem* write_flash -z 0 ESP32_GENERIC_S3-20231005-v1.21.0.bin
```

## Transferring the example files on the device

Use [talk32](https://github.com/antirez/talk32) to transfer the .py
files to the device:

    talk32 /dev/tty.your.usb.serial.device put *.py

Then enter the device in REPL mode:


    talk32 /dev/tty.your.usb.serial.device repl

Hit Ctrl+D to reset the device and you should see a demo usign the
TFT screen. The screen should change color every 2 seconds.

## Display SPI setup

Please note that the MicroPython SoftSPI implementation is *very* slow.
It is important to use the hardware SPI of the ESP32-S3. In order to
setup the SPI for the display, use code like this:

```
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
```

The speedup is around 20x. A full fill of the display with pixels of the
same color takes around 29 milliseconds.

The ST7789 display driver in this repository was rewritten in order to
use the MicroPython framebuffer abstraction (at the cost of some memory).
It means that normally we write in the framebuffer, and only blit the
content to the actual screen when the `show` method is called.
This improves performances, allows to use the bitmap font available in
the framebuffer implementation, together with the other graphical primitives
available, and in general makes adapting other code (for example written
for the SD1306 oled display) to this display much simpler.

## Running the examples

### Scroller

The Scroller example shows how to use the TFT driver in monochrome
mode, to save memory in the framebuffer. To test it:

    talk32 /dev/tty.usbmodem1245661 put st7789py.py
    talk32 /dev/tty.usbmodem1245661 put axp2101.py
    talk32 /dev/tty.usbmodem1234561 run examples/scroller.py

You will see numbers on the screen like if it was a terminal
outputting a sequence, with vertical scrolling and so forth.
The Scroller also implements line wrapping, but that's not
visible in the example.

WARNING: right now this example will be very slow since the TFT driver was
updated but the local framebuffer support was removed to be rewritten. This wil lbe fixed in the future, hopefully.

## Work in progress

This is a work in progress, my goal is to just gain enough access to the
device in order to port [FreakWAN](https://github.com/antirez/FreakWAN)
into this device. My plans if I find some time is to write an SX126x
driver similar to the one I wrote for the SX127x LoRa chip (part of
FreakWAN), implement the terminal-style display code for the watch
and not much than this.

The problem is that there are no guarantees this device will be produced
in the future, so since I own two of these, I'll try to make them
working, but likely most of the future developments will focus more
on the T-BEAM and T3 devices that are sold for many years.
