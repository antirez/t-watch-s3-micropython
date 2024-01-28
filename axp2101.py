from machine import Pin, SoftI2C

# Setup the AXP2101 to power the device
def pmu_setup():
    i2c = SoftI2C(sda=Pin(10), scl=Pin(11))

    if False:
        # Set above to True if you want to list all the address
        # of reachable devices. In the t-watch S3 they should
        # be 25, 52, 81, 90. Corresponding (in random order) to
        # the haptic motor, RTC clock, accelerometer and the
        # AXP2101.
        print("i2c devices replying to SDA:10 SCL:11",i2c.scan())

    slave_addr = 0x34 # AXP2101 i2c slave address.

    # Read PMU STATUS 1
    b = i2c.readfrom_mem(slave_addr,0x00,1)
    pmu_status = b[0]
    print("[AXP2110] PMU status 1 at startup", bin(pmu_status))

    # Set vbus voltage limit to 4.36v
    # Register 0x15 is Input voltage limit control.
    # A value of 6 means 4.36v as volts = 3.88 + value * 0.08
    # This should be the default.
    i2c.writeto_mem(slave_addr,0x15,bytearray([6]))

    # Read it back.
    v = i2c.readfrom_mem(slave_addr,0x15,1)
    print("[AXP2110] vbus voltage limit set to", 3.88+v[0]*0.08)

    # Set input current limit to 100ma. The value for 100ma is just 0,
    # and the regsiter 0x16 is "Input current limit control".
    i2c.writeto_mem(slave_addr,0x16,bytearray([0]))

    # Set the voltage to sense in order to power-off the device.
    # we set it to 2.6 volts, that is the minimum, corresponding
    # to a value of 0 written in the 0x24 register named
    # "Vsys voltage for PWROFF threshold setting".
    i2c.writeto_mem(slave_addr,0x24,bytearray([0]))

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
        i2c.writeto_mem(slave_addr,reg,bytearray([28]))

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
        b = i2c.readfrom_mem(slave_addr,reg,1)
        print(f"[AXP2110] ON/OFF Control value for {hex(reg)}:", bin(b[0]))

    # Only enable DC1 from register 0x80
    i2c.writeto_mem(slave_addr,0x80,bytearray([1]))

    # Enable ADLO1, 2, 3, 4, BLDO2 from register 0x90
    # and disable all the rest.
    i2c.writeto_mem(slave_addr,0x90,bytearray([1+2+4+8+32]))
    i2c.writeto_mem(slave_addr,0x91,bytearray([0]))
