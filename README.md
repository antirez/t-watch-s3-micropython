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


    mpremote cp *.py :

Then enter the device in REPL mode:

    mpremote repl

Hit Ctrl+D to reset the device and you should see a demo usign the
TFT screen. The screen should change color every 2 seconds, a few
random pixels are written.

# ST7789v Display

Right now we use [a driver](https://github.com/antirez/ST77xx-pure-MP) I wrote myself for a different project. The driver is conceived to use little memory, but it has an optional framebuffer target (240x240x2 bytes of memory used) that is much faster and is used in the example code here.

Please note that the MicroPython SoftSPI implementation is *very* slow.
It is important to use the hardware SPI of the ESP32-S3. In order to
setup the SPI for the display, use code like this:

```
# Our display does not have a MISO pin, but the MicroPython
# SPI implementation does not allow to avoid specifying one, so
# we use just a not used pin in the device.

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
```

Then you can use directly the graphics primitives (see driver documentation), or if you want more speed, you can enalbe the framebuffer, draw in the framebuffer, and then show the content with the show method:

```
display.enable_framebuffer()
display.fb.fill(display.db_color(0,0,0))
dispaly.fb.text("Hello world",10,10,10,display.fb_color(50,100,150))
display.show()
```

The speedup with hardware SPI is around 20x. Performances are much
better using the in-memory framebuffer. Using both, it is possible to write
quite fast graphics.

## Scroller example

The Scroller example shows how to use the TFT driver like a terminal.

    mpremote cp *.py :
    mpremote run examples/scroller.py

You will see numbers on the screen like if it was a terminal
outputting a sequence, with smooth vertical scrolling.
The Scroller also implements line wrapping, but that's not
visible in the example.

# Touch driver

You can find the code in `ft6x06.py`. There is a test main at the end
of the file.

    mpremote run ft6x06.py

Then touch the device screen to see updates.

Another exmaple, that let you write on the screen with your finger, can
be executed with:

    mpremote cp *.py :
    mpremote run examples/touch.py

# LoRa driver for the SX1262

Check the `sx1262.py` file. It contains a full implementation of the LoRa driver for this device. There is a small test program at the end of the file. If you want to see a more serious port, check the [FreakWAN](https://github.com/antirez/freakwan) project.
