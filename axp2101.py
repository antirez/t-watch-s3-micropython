from machine import Pin, SoftI2C

# Setup the AXP2101 to power the device
class AXP2101:
    def __init__(self):
        self.i2c = SoftI2C(sda=Pin(10), scl=Pin(11))
        self.slave_addr = 0x34 # AXP2101 i2c slave address.

        if False:
            # Set above to True if you want to list all the address
            # of reachable devices. In the t-watch S3 they should
            # be 25, 52, 81, 90. Corresponding (in random order) to
            # the haptic motor, RTC clock, accelerometer and the
            # AXP2101.
            print("i2c devices replying to SDA:10 SCL:11",i2c.scan())

    def read(self,reg):
        data = self.i2c.readfrom_mem(self.slave_addr,reg,1)
        return data[0]

    def write(self,reg,val):
        self.i2c.writeto_mem(self.slave_addr,reg,bytearray([val]))

    def setbit(self,reg,bit):
        oldval = self.read(reg)
        oldval |= 1<<bit
        self.write(reg,oldval)

    def clearbit(self,reg,bit):
        oldval = self.read(reg)
        oldval &= 0xff ^ (1<<bit)
        self.write(reg,oldval)

    # T-WATCH S3 specific power-on steps.
    def twatch_s3_poweron(self):
        # Read PMU STATUS 1
        pmu_status = self.read(0x00)
        print("[AXP2110] PMU status 1 at startup", bin(pmu_status))

        # Set vbus voltage limit to 4.36v
        # Register 0x15 is Input voltage limit control.
        # A value of 6 means 4.36v as volts = 3.88 + value * 0.08
        # This should be the default.
        self.write(0x15,6)

        # Read it back.
        v = self.read(0x15)
        print("[AXP2110] vbus voltage limit set to", 3.88+v*0.08)

        # Set input current limit to 100ma. The value for 100ma is just 0,
        # and the regsiter 0x16 is "Input current limit control".
        self.write(0x16,0)

        # Set the voltage to sense in order to power-off the device.
        # we set it to 2.6 volts, that is the minimum, corresponding
        # to a value of 0 written in the 0x24 register named
        # "Vsys voltage for PWROFF threshold setting".
        self.write(0x24,0)

        # Now we need to set output voltages of the different output
        # "lines" we have. There are successive registers to set the
        # voltage:
        #
        # 0x92 for ALDO1 (RTC)
        # 0x93 for ALDO2 (TFT backlight)
        # 0x94 for ALDO3 (touchscreen driver)
        # 0x95 for ALDO4 (LoRa chip)
        # 0x96 for BLD01 is not used
        # 0x97 for BLD02 (drv2605, that is the haptic motor)
        #
        # We will set a current of 3.3 volts for all those.
        # The registers to value 'v' so that voltage is
        # 0.5 + (0.1*v), so to get 3.3 we need to set the register
        # to the value of 28.
        for reg in [0x92, 0x93, 0x94, 0x95, 0x97]:
            self.write(reg,28)

        # Note that while we set the voltages, currently the
        # output lines (but DC1, that powers the ESP32 and is already
        # enabled at startup) may be off, so we need to enable them.
        # Let's show the current situation by reading the folliwing
        # registers:
        # 0x90, LDOS ON/OFF control 0
        # 0x91, LDOS ON/OFF control 1
        # 0x80, DCDCS ON/OFF and DVM control
        # that is the one controlling what is ON or OFF:
        for reg in [0x90, 0x91, 0x80]:
            b = self.read(reg)
            print(f"[AXP2110] ON/OFF Control value for {hex(reg)}:", bin(b))

        # Only enable DC1 from register 0x80
        self.write(0x80,1)

        # Enable ADLO1, 2, 3, 4, BLDO2 from register 0x90
        # and disable all the rest.
        self.write(0x90,1+2+4+8+32)
        self.write(0x91,0)

        # Disable TS pin measure channel from the ADC, it
        # causes issues while charging the device.
        # This is performed clearing bit 1 from the
        # 0x30 register: ADC channel enable control.
        self.clearbit(0x30,1)

if  __name__ == "__main__":
    twatch_pmu = AXP2101()
    twatch_pmu.twatch_s3_poweron()
