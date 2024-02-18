# FT6206/6306 simple driver.
# Copyright (C) 2024 Salvatore Sanfilippo -- All Rights Reserved
# This code is released under the MIT license
# https://opensource.org/license/mit/
#
# Written reading the specification at:
# https://www.displayfuture.com/Display/datasheet/controller/FT6206.pdf

from machine import Pin

REG_DEV_MODE = const(0x00)
REG_GEST_ID = const(0x01)
REG_TD_STATUS = const(0x02)

class FT6206:
    def __init__(self,i2c,*,interrupt_pin=None, callback=None):
        self.myaddr = 0x38  # I2C chip default address.
        self.i2c = i2c
        print("FT6206: scan i2c bus:", [hex(x) for x in i2c.scan()])
        self.callback = callback
        self.interrupt_pin = interrupt_pin
        self.interrupt_pin.irq(handler=self.irq, trigger=Pin.IRQ_FALLING)

    def irq(self,pin):
        if self.callback == None:
            printf("FT6206: not handled IRQ. Pass 'callback' during initialization")
            return
        data = self.get_touch_coords()
        if data == None: return
        self.callback(data)

    # Return the single byte at the specified register
    def get_reg(self, register, count=1):
        if count == 1:
            return self.i2c.readfrom_mem(self.myaddr,register,1)[0]
        else:
            return self.i2c.readfrom_mem(self.myaddr,register,count)

    # Return the number of touches on the screen.
    # The function returns 0 if no finger is on the screen.
    def get_touch_count(self):
        return self.get_reg(REG_TD_STATUS) & 7

    # Return coordiantes and information about touch point "id"
    # The returned data is a tuple with:
    # x, y, event (0 = press down, 1 = lift up), weight, area.
    # In certain devices touch area/weight will be just zero.
    def get_coords_for_p(self,touch_id):
        # Touch information registers start here 0x03.
        # Each touch data is 6 registers.
        start_reg = 0x03 + (6*touch_id)
        data = self.get_reg(start_reg,6)
        event = data[0] >> 6
        x = ((data[0]&7)<<8) | data[1]
        y = ((data[2]&7)<<8) | data[3]
        weight = data[4]
        area = data[5]>>4
        return (x,y,event,weight,area)

    # Return an array of touches (1 or 2 touches) or None if no
    # touch is present on the display right now.
    def get_touch_coords(self):
        touches = self.get_touch_count()
        if touches == 0: return None

        touch_data = []
        for i in range(touches):
            ev = self.get_coords_for_p(i) # Get event data.
            ev_type = "down" if ev[2] == 0 else "up"
            touch_data.append({
                "x":ev[0],
                "y":ev[1],
                "type":ev_type,
                "weight":ev[3],
                "area":ev[4]
            })
        return touch_data

# Example usage and quick test to see if your device is working.
if  __name__ == "__main__":
    from machine import SoftI2C, Pin
    import time

    # This example can use the IRQ or just polling.
    # By default the IRQ usage is demostrated.
    use_irq = True

    i2c = SoftI2C(scl=40,sda=39)
    if use_irq:
        def data_available(data):
            print(data)

        ft = FT6206(i2c,interrupt_pin=Pin(16,Pin.IN),callback=data_available)
        while True: time.sleep(1)
    else:
        ft = FT6206(i2c)
        while True:
            print(ft.get_touch_coords())
            time.sleep(1)
