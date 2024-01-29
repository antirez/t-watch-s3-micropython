# SX1262 driver for MicroPython
# Copyright (C) 2024 Salvatore Sanfilippo <antirez@gmail.com>
# All Rights Reserved
#
# This code is released under the BSD 2 clause license.
# See the LICENSE file for more information

from machine import Pin, SoftSPI
from micropython import const
import time, struct, urandom

# SX1262 constants

# Registers IDs and notable values
RegRxGain = const(0x8ac)
RegRxGain_PowerSaving = const(0x94) # Value for RegRxGain
RegRxGain_Boosted = const(0x96)     # Value for RegRxGain
RegLoRaSyncWordMSB = const(0x0740)
RegLoRaSyncWordLSB = const(0x0741)

# Dio0 mapping
IRQSourceNone = const(0)
IRQSourceTxDone = const(1 << 0)
IRQSourceRxDone = const(1 << 1)
IRQSourcePreambleDetected = const(1 << 2)
IRQSourceSyncWordValid = const(1 << 3)
IRQSourceHeaderValid = const(1 << 4)
IRQSourceHeaderErr = const(1 << 5)
IRQSourceCrcErr = const(1 << 6)
IRQSourceCadDone = const(1 << 7)
IRQSourceCadDetected = const(1 << 8)
IRQSourceTimeout = const(1 << 9)

# Commands opcodes
ClearIrqStatusCmd = const(0x02)
SetDioIrqParamsCmd = const(0x08)
WriteRegisterCmd = const(0x0d)
WriteBufferCmd = const(0x0e)
GetIrqStatusCmd = const(0x12)
GetRxBufferStatusCmd = const(0x13)
GetPacketStatusCmd = const(0x14)
ReadRegisterCmd = const(0x1d)
ReadBufferCmd = const(0x1e)
SetStandByCmd = const(0x80)
SetRxCmd = const(0x82)
SleepCmd = const(0x84)
SetRfFrequencyCmd = const(0x86)
SetPacketTypeCmd = const(0x8a)
SetModulationParamsCmd = const(0x8b)
SetPacketParamsCmd = const(0x8c)
SetTxParamsCmd = const(0x8e)
SetBufferBaseAddressCmd = const(0x8f)
SetPaConfigCmd = const(0x95)

class SX1262:
    def __init__(self, pinset, rx_callback, tx_callback = None):
        self.receiving = False
        self.tx_in_progress = False
        self.msg_sent = 0
        self.received_callback = rx_callback
        self.transmitted_callback = tx_callback
        self.busy_pin = Pin(pinset['busy'],Pin.IN)
        self.reset_pin = Pin(pinset['reset'],Pin.OUT)
        self.chipselect_pin = Pin(pinset['chipselect'], Pin.OUT)
        self.clock_pin = Pin(pinset['clock'])
        self.mosi_pin = Pin(pinset['mosi'])
        self.miso_pin = Pin(pinset['miso'])
        self.dio_pin = Pin(pinset['dio0'], Pin.IN)
        self.spi = SoftSPI(baudrate=10000000, polarity=0, phase=0, sck=self.clock_pin, mosi=self.mosi_pin, miso=self.miso_pin)
        self.bw = 0 # Currently set bandwidth. Saved to compute freq error.
         
    def reset(self):
        self.reset_pin.off()
        time.sleep_us(500)
        self.reset_pin.on()
        time.sleep_us(500)
        self.receiving = False
        self.tx_in_progress = False

    def standby(self):
        self.command(SetStandByCmd,0) # argument 0 menas STDBY_RC mode.

    # Note: the CS pin logic is inverted. It requires to be set to low
    # when the chip is NOT selected for data transfer.
    def deselect_chip(self):
        self.chipselect_pin.on()

    def select_chip(self):
        self.chipselect_pin.off()

    # Send a read or write command, and return the reply we
    # got back. 'data' can be both an array of a single integer.
    def command(self, opcode, data=None): 
        if data != None:
            if isinstance(data,int): data = [data]
            payload = bytearray(1+len(data)) # opcode + payload
            payload[0] = opcode
            payload[1:] = bytes(data)
        else:
            payload = bytearray([opcode])
        reply = bytearray(len(payload))

        # Wait for the chip to return available.
        while self.busy_pin.value():
            time.sleep_us(1)

        self.select_chip()
        self.spi.write_readinto(payload,reply)
        self.deselect_chip()

        # Enable this for debugging.
        if False: print(f"Reply for {hex(opcode)} is {repr(reply)}")

        return reply

    def readreg(self, addr, readlen=1):
        payload = bytearray(2+1+readlen) # address + nop + nop*bytes_to_read
        payload[0] = (addr&0xff00)>>8
        payload[1] = addr&0xff
        reply = self.command(ReadRegisterCmd,payload)
        return reply[4:]

    def writereg(self, addr, data):
        if isinstance(data,int): data = bytes([data])
        payload = bytearray(2+len(data)) # address + bytes_to_write
        payload[0] = (addr&0xff00)>>8
        payload[1] = addr&0xff
        payload[2:] = data
        self.command(WriteRegisterCmd,payload)

    def readbuf(self, off, numbytes):
        payload = bytearray(2+numbytes)
        payload[0] = off
        data = self.command(ReadBufferCmd,payload)
        return data[3:]

    def writebuf(self, off, data):
        payload = bytearray(1+len(data))
        payload[0] = off
        payload[1:] = data
        self.command(WriteBufferCmd,payload)

    def set_frequency(self, mhz):
        # The final frequency is (rf_freq * xtal freq) / 2^25.
        oscfreq = 32000000 # Oscillator frequency for registers calculation
        rf_freq = int(mhz * (2**25) / oscfreq)
        arg = [(rf_freq & 0xff000000) >> 24,
               (rf_freq & 0xff0000) >> 16,
               (rf_freq & 0xff00) >> 8,
               (rf_freq & 0xff)]
        self.command(SetRfFrequencyCmd, arg)

    def begin(self):
        self.reset()
        self.deselect_chip()
        self.standby()              # SX126x gets configured in standby.
        self.command(SetPacketTypeCmd,0x01) # Put the chip in LoRa mode.

    # Set the radio parameters. Allowed spreadings are from 6 to 12.
    # Bandwidth and coding rate are listeed below in the dictionaries.
    # TX power is from -9 to +22 dbm.
    def configure(self, freq, bandwidth, rate, spreading, txpower):
        Bw = {   7800: 0,
                10400: 0x8,
                15600: 0x1,
                20800: 0x9,
                31250: 0x2,
                41700: 0xa,
                62500: 0x3,
               125000: 0x4,
               250000: 0x5,
               500000: 0x6}
        CodingRate = {  5:1,
                        6:2,
                        7:3,
                        8:4}

        # Make sure the chip is in standby mode
        # during configuration.
        self.standby()

        # Set LoRa parameters.
        lp = bytearray(4)
        lp[0] = spreading
        lp[1] = Bw[bandwidth]
        lp[2] = CodingRate[rate]
        lp[3] = 1 # Enable low data rate optimization
        self.command(SetModulationParamsCmd,lp)

        # Set packet params.
        pp = bytearray(6)
        preamble_len = 12 # This is a very reasonable value that works well.
        pp[0] = preamble_len >> 8
        pp[1] = preamble_len & 0xff
        pp[2] = 0       # Explicit header
        pp[3] = 0xff    # Max payload length
        pp[4] = 1       # CRC on
        pp[5] = 0       # Standard IQ setup
        self.command(SetPacketParamsCmd,pp)

        # Set RF frequency.
        self.set_frequency(freq)

        # Use maximum sensibility
        lora.writereg(RegRxGain,0x96)

        # Set TCXO voltage to 1.7 with 5000us delay.
        tcxo_delay = int(5000.0 / 15.625)
        tcxo_config = bytearray(4)
        tcxo_config[0] = 1 # 1.7v
        tcxo_config[1] = (tcxo_delay >> 16) & 0xff
        tcxo_config[2] = (tcxo_delay >> 8) & 0xff
        tcxo_config[3] = (tcxo_delay >> 0) & 0xff
        lora.command(0x97,tcxo_config)

        # Set DIO2 as RF switch like in Semtech examples.
        lora.command(0x9d,1)

        # Set the power amplifier configuration.
        paconfig = bytearray(4)
        paconfig[0] = 4 # Duty Cycle of 4
        paconfig[1] = 7 # Max output +22 dBm
        paconfig[2] = 0 # Select PA for SX1262 (1 would be SX1261)
        paconfig[3] = 1 # Always set to 1 as for datasheet
        self.command(SetPaConfigCmd,paconfig)

        # Set TX power and ramping. We always use high power mode.
        txpower = min(max(-9,txpower),22)
        txparams = bytearray(2)
        txparams[0] = (0xF7 + (txpower+9)) % 256
        txparams[1] = 4 # 200us ramping time
        self.command(SetTxParamsCmd,txparams)

        # We either receive or send, so let's use all the 256 bytes
        # of FIFO available by setting both recv and send FIFO address
        # to the base.
        self.command(SetBufferBaseAddressCmd,[0,0])
       
        # Setup the IRQ handler to receive the packet tx/rx and
        # other events. Note that the chip will put the packet
        # on the FIFO even on CRC error.
        # We will enable all DIOs for all the interrputs. In
        # practice most of the times only one chip DIO is connected
        # to the MCU.
        self.dio_pin.irq(handler=self.txrxdone, trigger=Pin.IRQ_RISING)
        self.command(SetDioIrqParamsCmd,[0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff])
        self.clear_irq()

        # Set sync word to 0x12 (private network).
        # Note that "12" is in the most significant hex digits of
        # the two registers: [1]4 and [2]4.
        self.writereg(RegLoRaSyncWordMSB,0x14)
        self.writereg(RegLoRaSyncWordLSB,0x24)

        # TODO: calibrate depending on frequency selected.

    # This is just for debugging. We can understand if a given command
    # caused a failure while debugging the driver since the command status
    # will be set to '5'. We can also observe if the chip is in the
    # right mode (tx, rx, standby...).
    def show_status(self):
        status = lora.command(0xc0,0)[1]
        print("Chip mode  = ", (status >> 4) & 7)
        print("Cmd status = ", (status >> 1) & 7)

    # Put the chip in continuous receive mode.
    # Note that the SX1262 is bugged and if there is a strong
    # nearby signal sometimes it "crashes" and no longer
    # receives anything, so it may be a better approach to
    # set a timeout and re-enter receive from time to time?
    def receive(self):
        self.command(SetRxCmd,[0xff,0xff,0xff])
        self.receiving = True
    
    def spi_write(self, regid, data): 
        # Writes are performed sending as first byte the register
        # we want to address, with the highest bit set.
        if isinstance(data,int):
            spi_payload = bytes([regid|0x80,data])
        elif isinstance(data,str):
            spi_payload = bytes([regid|0x80]) + bytes(data, 'utf-8')
        elif isinstance(data,bytes):
            spi_payload = bytes([regid|0x80]) + data
        else:
            raise Exception("spi_write can only handle integers and strings")
        self.select_chip()
        self.spi.write(spi_payload)
        self.deselect_chip()

    # SPI read. For simplicity in the API, if the read len is one
    # we return the byte value itself (the first byte is not data).
    # However for bulk reads we return the string (minus the first
    # byte, as said).
    def spi_read(self, regid, l=1):
        # Reads are similar to writes but we don't need to set
        # the highest bit of the byte, so the SPI library will take
        # care of writing the register.
        self.select_chip()
        if l == 1:
            rcv = self.spi.read(l+1,regid)[1]
        else:
            rcv = self.spi.read(l+1,regid)[1:]
        self.deselect_chip()
        return rcv

    def get_irq(self):
        reply = self.command(GetIrqStatusCmd,[0,0,0])
        return (reply[2]<<8) | reply[3]

    def clear_irq(self):
        reply = self.command(ClearIrqStatusCmd,[0xff,0xff])

    # This is our IRQ handler. By default we don't mask any interrupt
    # so the function may be called for more events we actually handle.
    def txrxdone(self, pin):
        event = self.get_irq()
        self.clear_irq()

        if event & IRQSourceRxDone:
            # Obtain packet information.
            bs = self.command(GetRxBufferStatusCmd,[0]*3)
            ps = self.command(GetPacketStatusCmd,[0]*4)

            # Extract packet information.
            packet_len = bs[2]
            packet_start = bs[3]
            rssi = -ps[2]/2 # Average RSSI in dB.
            snr = ps[3]-256 if ps[3] > 128 else ps[3] # Convert to unsigned
            snr /= 4 # The reported value is upscaled 4 times.

            print(f"RX packet {packet_len} bytes at {packet_start} RSSI: {rssi}")
            packet = self.readbuf(packet_start,packet_len)
            print("Packet data: ",packet)
            return

            bad_crc = (event & IRQPayloadCrcError) != 0
            # Read data from the FIFO
            addr = self.spi_read(RegFifoRxCurrentAddr)
            self.spi_write(RegFifoAddrPtr, addr) # Read starting from addr
            packet_len = self.spi_read(RegRxNbBytes)
            packet = self.spi_read(RegFifo, packet_len)
            snr = self.spi_read(RegPktSnrValue)
            snr /= 4 # Packet SNR * 0.25, section 3.5.5 of chip spec.
            rssi = self.spi_read(RegPktRssiValue) 

            # Convert RSSI, also taking into account SNR, but only when the
            # message we received has a power under the noise level. Odd
            # but possible with LoRa modulation.
            #
            # We use the formula found in the chip datasheet.
            # Note: this forumla is correct for HF (high frequency) port,
            # but otherwise the constant -157 should be replaced with
            # -164.
            if snr >= 0:
                rssi = round(-157+16/15*rssi,2)
            else:
                rssi = round(-157+rssi+snr,2)

            if bad_crc:
                print("SX1276: packet with bad CRC received")

            # Call the callback the user registered, if any.
            if self.received_callback:
                self.received_callback(self, packet, rssi, bad_crc)
        elif event & IRQSourceTxDone:
            self.msg_sent += 1
            # After sending a message, the chip will return in
            # standby mode. However if we were receiving we
            # need to return back to such state.
            if self.transmitted_callback: self.transmitted_callback()
            if self.receiving: self.receive()
            self.tx_in_progress = False
        else: 
            print("SX1276: not handled event IRQ flags "+bin(event))

    def modem_is_receiving_packet(self):
        # FIXME: use preamble detection + timeout like in
        # the C port of FreakWAN to implement this even if the
        # limits of SX1262. Also consider checking the RSSI
        # instantaneous reading value.
        return False

    def send(self, data): 
        self.tx_in_progress = True
        self.spi_write(RegDioMapping1, Dio0TxDone)
        self.spi_write(RegFifoAddrPtr, 0) # Write data starting from FIFO byte 0
        self.spi_write(RegFifo, data)     # Populate FIFO with message
        self.spi_write(RegPayloadLength, len(data))  # Store len of message
        self.spi_write(RegOpMode, ModeTx) # Switch to TX mode

if  __name__ == "__main__":
    pinset = {
        'busy': 7,
        'reset': 8,
        'chipselect': 5,
        'clock': 3,
        'mosi': 1,
        'miso': 4,
        'dio0': 9
    }

    def onrx(data):
        print(data)

    lora = SX1262(pinset=pinset,rx_callback=onrx)
    lora.begin()
    lora.configure(869500000, 250000, 8, 12, 22)
    lora.receive()
    while True:
        # lora.show_status()
        time.sleep(1)
