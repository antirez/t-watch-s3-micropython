# Copyright (C) 2023 Salvatore Sanfilippo <antirez@gmail.com>
# All Rights Reserved
#
# This code is released under the BSD 2 clause license.
# See the LICENSE file for more information

import time

# This class implements terminal like view for any display implementing
# the Framebuffer interface.
class Scroller:
    Font8x8 = 0
    Font4x6 = 1
    StateActive = 0  # Display active
    StateDimmed = 1  # Dispaly still active but minimum contrast set
    StateSaver = 2   # Screen saver: only icons at random places on screen.

    def __init__(self, display, icons=None, dim_time=10, ss_time=120):
        self.display = display  # Display driver
        self.icons = icons
        self.lines = []
        self.xres = 240
        self.yres = 240
        # The framebuffer of MicroPython only supports 8x8 fonts so far, so:
        self.select_font("big")
        self.last_update = time.time()
        # OLED saving system state. We write text at an x,y offset, that
        # can be of 0 or 1 pixels. This way we use pixels more evenly
        # creating a less evident image ghosting effect in the more
        # used pixels.
        self.xoff = 0
        self.yoff = 0
        self.dim_t = dim_time       # Inactivity to set to lower contrast.
        self.screensave_t = ss_time # Inactivity to enable screen saver.
        self.state = self.StateActive
        self.contrast = 255

    # Set maximum display contrast. It will be dimmed after some inactivity
    # time.
    def set_contrast(self,contrast):
        self.contrast = contrast

    # Get current contrast based on inactivity time.
    def get_contrast(self):
        if self.state == self.StateActive:
            return self.contrast
        elif self.state == self.StateDimmed or self.state == self.StateSaver:
            return 1 # Still pretty visible but in direct sunlight

    # Update self.state based on last activity time.
    def update_screensaver_state(self):
        inactivity = time.time() - self.last_update
        if inactivity > self.screensave_t:
            self.state = self.StateSaver
        elif inactivity > self.dim_t:
            self.state = self.StateDimmed
        else:
            self.state = self.StateActive

    def select_font(self,fontname):
        if fontname == "big":
            self.font = self.Font8x8
            self.font_width = 8
            self.font_height = 8
        elif fontname == "small":
            # Use 5/7 to provide the required spacing. The font 8x8
            # already includes spacing.
            self.font = self.Font4x6
            self.font_width = 5
            self.font_height = 7
        self.cols = int(self.xres/self.font_width)
        self.rows = int(self.yres/self.font_height)

    def render_text(self,text,x,y):
        if self.font == self.Font8x8:
            self.display.text(x, y, text, self.display.color(255,255,255),
                                          self.display.color(0,0,0))
        else:
            for c in text:
                self.render_4x6_char(c, x, y)
                x += self.font_width

    def render_4x6_char(self,c,px,py):
        idx = ord(c)
        if idx > len(FontData4x6)/3:
            idx = ord("?")
        for y in range(0,6):
            bits = FontData4x6[idx*3+(int(y/2))]
            if not y & 1: bits >>= 4
            for x in range(0,4):
                if bits & (1<<(3-x)):
                    self.display.pixel(px+x,py+y,self.display.color(255,255,255))

    # Return the number of rows needed to display the current self.lines
    # This number may be > self.rows.
    def rows_needed(self):
        needed = 0
        for l in self.lines:
            needed += int((len(l)+(self.cols-1))/self.cols)
        return needed

    # When displaying images, we need to start from the row edge in order
    # make mixes of images and text well aligned. So we pad the image
    # height to the font height.
    def get_image_padded_height(self,height):
        if height % self.font_height:
            padded_height = height+(self.font_height-(height%self.font_height))
        else:
            padded_height = height
        return padded_height


    # Draw the scroller "terminal" text.
    def draw_text(self):
        # We need to draw the lines backward starting from the last
        # row and going backward. This makes handling line wraps simpler,
        # as we consume from the end of the last line and so forth.
        y = (min(self.rows,self.rows_needed())-1) * self.font_height
        lines = self.lines[:]
        while y >= 0:
            # Handle text
            if len(lines[-1]) == 0:
                # We consumed all the current line. Remove it
                # and start again from the top of the loop, since
                # the next line could be an image.
                lines.pop(-1)
                if len(lines) == 0: return # Should not happen
                continue

            to_consume = len(lines[-1]) % self.cols
            if to_consume == 0: to_consume = self.cols
            rowchars = lines[-1][-to_consume:] # Part to display from the end
            lines[-1]=lines[-1][:-to_consume]  # Remaining part.
            self.render_text(rowchars, 0+self.xoff, y+self.yoff)
            y -= self.font_height

    # Return the minimum time the caller should refresh the screen
    # the next time, in case of no activity. This is useful so that we
    # can dim the screen and update other time-dependent stuff (such status
    # icons) fast enough for the UI behavior to make sense.
    def min_refresh_time(self):
        icon_min_rt = self.icons.min_refresh_time()
        if self.state == self.StateActive:
            rt = self.dim_t+1
        elif self.state == self.StateDimmed:
            rt = self.screensave_t+1
        elif self.state == self.StateSaver:
            rt = 60
        return min(icon_min_rt,rt)

    # Update the screen content.
    def refresh(self):
        if not self.display: return
        self.update_screensaver_state()
        self.display.fill(self.display.color(0,0,0))
        if self.state != self.StateSaver:
            minutes = int(time.time()/60) % 4
            # We use minutes from 0 to 3 to move text one pixel
            # left-right, top-bottom. This saves OLED from overusing
            # always the same set of pixels.
            self.xoff = minutes & 1
            self.yoff = (minutes>>1) & 1
            self.draw_text()
        random_icons_offset = self.state == self.StateSaver
        if self.icons: self.icons.refresh(random_offset=random_icons_offset)

    # Convert certain unicode points to our 4x6 font characters.
    def convert_from_utf8(self,msg):
        msg = msg.replace("è","\x80")
        msg = msg.replace("é","\x81")
        msg = msg.replace("😀","\x96\x97")
        return msg

    # Add a new line, without refreshing the display.
    def print(self,msg):
        if isinstance(msg,str):
            msg = self.convert_from_utf8(msg)
        self.lines.append(msg)
        self.lines = self.lines[-self.rows:]
        self.last_update = time.time()

if  __name__ == "__main__":
    from axp2101 import AXP2101
    from machine import Pin, SPI
    import st7789_base, st7789_ext

    # Setup the PMU chip & turn on the backlight.
    twatch_pmu = AXP2101()
    twatch_pmu.twatch_s3_poweron()
    bl = Pin(45,Pin.OUT)
    bl.on()

    # Setup TFT.
    display = st7789_ext.ST7789(
        SPI(1, baudrate=40000000, phase=0, polarity=1, sck=18, mosi=13, miso=37),
        240, 240,
        reset=False,
        dc=Pin(38, Pin.OUT),
        cs=Pin(12, Pin.OUT),
    )
    display.init(landscape=False,mirror_y=True,mirror_x=True,inversion=True)

    # Use the Scroller.
    scroller = Scroller(display)
    counter = 0
    while True:
        counter += 1
        scroller.print(str(counter))
        scroller.refresh()
        time.sleep(0.1)
