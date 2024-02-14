# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_drv2605`
====================================================

Port to Micropython of CircuitPython module for the DRV2605 haptic feedback motor driver.  See
examples/simpletest.py for a demo of the usage.

* Author(s): Tony DiCola
"""
from micropython import const

from machine import I2C

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_DRV2605.git"


# Internal constants:
_DRV2605_ADDR = const(0x5A)
_DRV2605_REG_STATUS = const(0x00)
_DRV2605_REG_MODE = const(0x01)
_DRV2605_REG_RTPIN = const(0x02)
_DRV2605_REG_LIBRARY = const(0x03)
_DRV2605_REG_WAVESEQ1 = const(0x04)
_DRV2605_REG_WAVESEQ2 = const(0x05)
_DRV2605_REG_WAVESEQ3 = const(0x06)
_DRV2605_REG_WAVESEQ4 = const(0x07)
_DRV2605_REG_WAVESEQ5 = const(0x08)
_DRV2605_REG_WAVESEQ6 = const(0x09)
_DRV2605_REG_WAVESEQ7 = const(0x0A)
_DRV2605_REG_WAVESEQ8 = const(0x0B)
_DRV2605_REG_GO = const(0x0C)
_DRV2605_REG_OVERDRIVE = const(0x0D)
_DRV2605_REG_SUSTAINPOS = const(0x0E)
_DRV2605_REG_SUSTAINNEG = const(0x0F)
_DRV2605_REG_BREAK = const(0x10)
_DRV2605_REG_AUDIOCTRL = const(0x11)
_DRV2605_REG_AUDIOLVL = const(0x12)
_DRV2605_REG_AUDIOMAX = const(0x13)
_DRV2605_REG_RATEDV = const(0x16)
_DRV2605_REG_CLAMPV = const(0x17)
_DRV2605_REG_AUTOCALCOMP = const(0x18)
_DRV2605_REG_AUTOCALEMP = const(0x19)
_DRV2605_REG_FEEDBACK = const(0x1A)
_DRV2605_REG_CONTROL1 = const(0x1B)
_DRV2605_REG_CONTROL2 = const(0x1C)
_DRV2605_REG_CONTROL3 = const(0x1D)
_DRV2605_REG_CONTROL4 = const(0x1E)
_DRV2605_REG_VBAT = const(0x21)
_DRV2605_REG_LRARESON = const(0x22)

# User-facing mode value constants:
MODE_INTTRIG = 0x00
MODE_EXTTRIGEDGE = 0x01
MODE_EXTTRIGLVL = 0x02
MODE_PWMANALOG = 0x03
MODE_AUDIOVIBE = 0x04
MODE_REALTIME = 0x05
MODE_DIAGNOS = 0x06
MODE_AUTOCAL = 0x07
LIBRARY_EMPTY = 0x00
LIBRARY_TS2200A = 0x01
LIBRARY_TS2200B = 0x02
LIBRARY_TS2200C = 0x03
LIBRARY_TS2200D = 0x04
LIBRARY_TS2200E = 0x05
LIBRARY_LRA = 0x06


class DRV2605:
    """TI DRV2605 haptic feedback motor driver module.

    :param I2C i2c: The board I2C object
    :param int address: The I2C address
    """

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(1)

    def __init__(self, i2c: I2C, address: int = _DRV2605_ADDR) -> None:
        self._device = i2c
        self._address = address
        # Check chip ID is 3 or 7 (DRV2605 or DRV2605L).
        status = self._read_u8(_DRV2605_REG_STATUS)
        device_id = (status >> 5) & 0x07
        if device_id not in (3, 7):
            raise RuntimeError("Failed to find DRV2605, check wiring!")
        # Configure registers to initialize chip.
        self._write_u8(_DRV2605_REG_MODE, 0x00)  # out of standby
        self._write_u8(_DRV2605_REG_RTPIN, 0x00)  # no real-time-playback
        self._write_u8(_DRV2605_REG_WAVESEQ1, 1)  # strong click
        self._write_u8(_DRV2605_REG_WAVESEQ2, 0)
        self._write_u8(_DRV2605_REG_OVERDRIVE, 0)  # no overdrive
        self._write_u8(_DRV2605_REG_SUSTAINPOS, 0)
        self._write_u8(_DRV2605_REG_SUSTAINNEG, 0)
        self._write_u8(_DRV2605_REG_BREAK, 0)
        self._write_u8(_DRV2605_REG_AUDIOMAX, 0x64)
        # Set ERM open-loop mode.
        self.use_ERM()
        # turn on ERM_OPEN_LOOP
        control3 = self._read_u8(_DRV2605_REG_CONTROL3)
        self._write_u8(_DRV2605_REG_CONTROL3, control3 | 0x20)
        # Default to internal trigger mode and TS2200 A library.
        self.mode = MODE_INTTRIG
        self.library = LIBRARY_TS2200A
        self._sequence = _DRV2605_Sequence(self)

    def _read_u8(self, address: int) -> int:
        # Read an 8-bit unsigned value from the specified 8-bit address.
        self._device.readfrom_mem_into(self._address, address, self._BUFFER)
        return self._BUFFER[0]

    def _write_u8(self, address: int, val: int) -> None:
        # Write an 8-bit unsigned value to the specified 8-bit address.
        self._BUFFER[0] = val & 0xFF
        self._device.writeto_mem(self._address, address, self._BUFFER)

    def play(self) -> None:
        """Play back the select effect(s) on the motor."""
        self._write_u8(_DRV2605_REG_GO, 1)

    def stop(self) -> None:
        """Stop vibrating the motor."""
        self._write_u8(_DRV2605_REG_GO, 0)

    @property
    def mode(self) -> int:
        """
        The mode of the chip. Should be a value of:

          * ``MODE_INTTRIG``: Internal triggering, vibrates as soon as you call
            play().  Default mode.
          * ``MODE_EXTTRIGEDGE``: External triggering, edge mode.
          * ``MODE_EXTTRIGLVL``: External triggering, level mode.
          * ``MODE_PWMANALOG``: PWM/analog input mode.
          * ``MODE_AUDIOVIBE``: Audio-to-vibration mode.
          * ``MODE_REALTIME``: Real-time playback mode.
          * ``MODE_DIAGNOS``: Diagnostics mode.
          * ``MODE_AUTOCAL``: Auto-calibration mode.

        See the datasheet for the meaning of modes beyond MODE_INTTRIG.
        """
        return self._read_u8(_DRV2605_REG_MODE)

    @mode.setter
    def mode(self, val: int) -> None:
        if not 0 <= val <= 7:
            raise ValueError("Mode must be a value within 0-7!")
        self._write_u8(_DRV2605_REG_MODE, val)

    @property
    def library(self) -> int:
        """
        The library selected for waveform playback.  Should be
        a value of:

          * ``LIBRARY_EMPTY``: Empty
          * ``LIBRARY_TS2200A``: TS2200 library A  (the default)
          * ``LIBRARY_TS2200B``: TS2200 library B
          * ``LIBRARY_TS2200C``: TS2200 library C
          * ``LIBRARY_TS2200D``: TS2200 library D
          * ``LIBRARY_TS2200E``: TS2200 library E
          * ``LIBRARY_LRA``: LRA library

        See the datasheet for the meaning and description of effects in each
        library.
        """
        return self._read_u8(_DRV2605_REG_LIBRARY) & 0x07

    @library.setter
    def library(self, val: int) -> None:
        if not 0 <= val <= 6:
            raise ValueError("Library must be a value within 0-6!")
        self._write_u8(_DRV2605_REG_LIBRARY, val)

    @property
    def sequence(self) -> "_DRV2605_Sequence":
        """List-like sequence of waveform effects.
        Get or set an effect waveform for slot 0-7 by indexing the sequence
        property with the slot number. A slot must be set to either an :class:`~Effect`
        or :class:`~Pause` class. See the datasheet for a complete table of effect ID
        values and the associated waveform / effect.

        E.g.:

        .. code-block:: python

            # Getting the effect stored in a slot
            slot_0_effect = drv.sequence[0]

            # Setting an Effect in the first sequence slot
            drv.sequence[0] = Effect(88)
        """
        return self._sequence

    @property
    def realtime_value(self) -> int:
        """The output value used in Real-Time Playback mode. When the device is
        switched to ``MODE_REALTIME``, the motor is driven continuously with an
        amplitude/direction determined by this value.

        By default, the device expects a SIGNED 8-bit integer, and its exact
        effect depends on both the type of motor (ERM/LRA) and whether the device
        is operating in open- or closed-loop (unidirectional/bidirectional) mode.

        See the datasheet for more information!

        E.g.:

        .. code-block:: python

            # Start real-time playback
            drv.realtime_value = 0
            drv.mode = adafruit_drv2605.MODE_REALTIME

            # Buzz the motor briefly at 50% and 100% amplitude
            drv.realtime_value = 64
            time.sleep(0.5)
            drv.realtime_value = 127
            time.sleep(0.5)

            # Stop real-time playback
            drv.realtime_value = 0
            drv.mode = adafruit_drv2605.MODE_INTTRIG
        """
        return self._read_u8(_DRV2605_REG_RTPIN)

    @realtime_value.setter
    def realtime_value(self, val: int) -> None:
        if not -127 <= val <= 255:
            raise ValueError("Real-Time Playback value must be between -127 and 255!")
        self._write_u8(_DRV2605_REG_RTPIN, val)

    def set_waveform(self, effect_id: int, slot: int = 0) -> None:
        """Select an effect waveform for the specified slot (default is slot 0,
        but up to 8 effects can be combined with slot values 0 to 7).  See the
        datasheet for a complete table of effect ID values and the associated
        waveform / effect.

        :param int effect_id: The effect ID of the waveform
        :param int slot: The sequence slot to use
        """
        if not 0 <= effect_id <= 123:
            raise ValueError("Effect ID must be a value within 0-123!")
        if not 0 <= slot <= 7:
            raise ValueError("Slot must be a value within 0-7!")
        self._write_u8(_DRV2605_REG_WAVESEQ1 + slot, effect_id)

    # pylint: disable=invalid-name
    def use_ERM(self) -> None:
        """Use an eccentric rotating mass motor (the default)."""
        feedback = self._read_u8(_DRV2605_REG_FEEDBACK)
        self._write_u8(_DRV2605_REG_FEEDBACK, feedback & 0x7F)

    # pylint: disable=invalid-name
    def use_LRM(self) -> None:
        """Use a linear resonance actuator motor."""
        feedback = self._read_u8(_DRV2605_REG_FEEDBACK)
        self._write_u8(_DRV2605_REG_FEEDBACK, feedback | 0x80)


class Effect:
    """DRV2605 waveform sequence effect.

    :param int effect_id: The ID number of the effect
    """

    def __init__(self, effect_id: int) -> None:
        self._effect_id = 0
        # pylint: disable=invalid-name
        self.id = effect_id

    @property
    def raw_value(self) -> int:
        """Raw effect ID."""
        return self._effect_id

    @property
    # pylint: disable=invalid-name
    def id(self) -> int:
        """Effect ID."""
        return self._effect_id

    @id.setter
    # pylint: disable=invalid-name
    def id(self, effect_id: int) -> None:
        """Set the effect ID."""
        if not 0 <= effect_id <= 123:
            raise ValueError("Effect ID must be a value within 0-123!")
        self._effect_id = effect_id

    def __repr__(self) -> str:
        return "{}({})".format(type(self).__qualname__, self.id)


class Pause:
    """DRV2605 waveform sequence timed delay.

    :param float duration: The duration of the pause in seconds
    """

    def __init__(self, duration: float) -> None:
        # Bit 7 must be set for a slot to be interpreted as a delay
        self._duration = 0x80
        self.duration = duration

    @property
    def raw_value(self) -> int:
        """Raw pause duration."""
        return self._duration

    @property
    def duration(self) -> float:
        """Pause duration in seconds."""
        # Remove wait time flag bit and convert duration to seconds
        return (self._duration & 0x7F) / 100.0

    @duration.setter
    def duration(self, duration: float) -> None:
        """Sets the pause duration in seconds."""
        if not 0.0 <= duration <= 1.27:
            raise ValueError("Pause duration must be a value within 0.0-1.27!")
        # Add wait time flag bit and convert duration to centiseconds
        self._duration = 0x80 | round(duration * 100.0)

    def __repr__(self) -> str:
        return "{}({})".format(type(self).__qualname__, self.duration)


class _DRV2605_Sequence:
    """Class to enable List-like indexing of the waveform sequence slots.

    :param DRV2605 DRV2605_instance: The DRV2605 instance
    """

    def __init__(
        self, DRV2605_instance: DRV2605  # pylint: disable=invalid-name
    ) -> None:
        self._drv2605 = DRV2605_instance

    def __setitem__(self, slot: int, effect: Union[Effect, Pause]) -> None:
        """Write an Effect or Pause to a slot."""
        if not 0 <= slot <= 7:
            raise IndexError("Slot must be a value within 0-7!")
        if not isinstance(effect, (Effect, Pause)):
            raise TypeError("Effect must be either an Effect or Pause!")
        # pylint: disable=protected-access
        self._drv2605._write_u8(_DRV2605_REG_WAVESEQ1 + slot, effect.raw_value)

    def __getitem__(self, slot: int) -> Union[Effect, Pause]:
        """Read an effect ID from a slot. Returns either a Pause or Effect class."""
        if not 0 <= slot <= 7:
            raise IndexError("Slot must be a value within 0-7!")
        # pylint: disable=protected-access
        slot_contents = self._drv2605._read_u8(_DRV2605_REG_WAVESEQ1 + slot)
        if slot_contents & 0x80:
            return Pause((slot_contents & 0x7F) / 100.0)
        return Effect(slot_contents)

    def __iter__(self) -> Union[Effect, Pause]:
        """Returns an iterator over the waveform sequence slots."""
        for slot in range(0, 8):
            yield self[slot]

    def __repr__(self) -> str:
        """Return a string representation of all slot's effects."""
        return repr(list(self))
