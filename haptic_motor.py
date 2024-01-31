from machine import Pin, SoftI2C
import adafruit_drv2605

# The DRV2605 chip controls the haptic motor.  The motor can play many different waveforms each of which has a different effect id.
# These effect IDs range from 0 to 123.
#
# Suggested effect IDs are:
#   10 - short vibration
#   12 - vibration with a pause and then another vibration
#   14 - long vibrate
# 
# Detailed information is available from Texas Instruments at https://www.ti.com/lit/ds/symlink/drv2605.pdf?ts=1706485376379
#
# The adafruit_drv2605 library is sourced from https://github.com/VynDragon/Adafruit_MicroPython_DRV2605
# which is derived from https://github.com/VynDragon/Adafruit_MicroPython_DRV2605

class HAPTIC_MOTOR:
    def __init__(self, effect_id):    
        self.effect_id = effect_id
        print("[HAPTIC_MOTOR] initialized")

    def vibrate(self):
        # Initialize I2C bus and DRV2605 module.
        i2c = SoftI2C(Pin(11, Pin.OUT), Pin(10, Pin.OUT))
        motor = adafruit_drv2605.DRV2605(i2c)
    
        # the motor has different waveform effects which have different IDs ranging from 0 to 123
        motor.sequence[0] = adafruit_drv2605.Effect(self.effect_id)  # Set the effect on slot 0    
        motor.play()  # play the effect
        motor.stop()  # and then stop (if it's still running)
