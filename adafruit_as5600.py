# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_as5600`
================================================================================

CircuitPython driver for the AS5600 Magnetic Angle Sensor


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Adafruit AS5600 Magnetic Angle Sensor - STEMMA QT <https://www.adafruit.com/product/6357>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_register.i2c_bits import RWBits, ROBits
from adafruit_register.i2c_bit import RWBit, ROBit

try:
    import typing  # pylint: disable=unused-import
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_AS5600.git"

# I2C Address
AS5600_DEFAULT_ADDRESS = const(0x36)

# Register addresses
_AS5600_REG_ZMCO = const(0x00)  # ZMCO register (burn count)
_AS5600_REG_ZPOS_H = const(0x01)  # Zero position high byte
_AS5600_REG_MPOS_H = const(0x03)  # Maximum position high byte
_AS5600_REG_MANG_H = const(0x05)  # Maximum angle high byte
_AS5600_REG_CONF_L = const(0x08)  # Configuration register low byte
_AS5600_REG_CONF_H = const(0x07)  # Configuration register high byte
_AS5600_REG_STATUS = const(0x0B)  # Status register
_AS5600_REG_RAWANGLE_H = const(0x0C)  # Raw angle high byte
_AS5600_REG_ANGLE_H = const(0x0E)  # Scaled angle high byte
_AS5600_REG_AGC = const(0x1A)  # Automatic Gain Control register
_AS5600_REG_MAGNITUDE_H = const(0x1B)  # Magnitude high byte
_AS5600_REG_BURN = const(0xFF)  # Burn command register

# Power mode constants
POWER_MODE_NOM = const(0x00)  # Normal mode (default)
POWER_MODE_LPM1 = const(0x01)  # Low power mode 1
POWER_MODE_LPM2 = const(0x02)  # Low power mode 2
POWER_MODE_LPM3 = const(0x03)  # Low power mode 3

# Hysteresis constants
HYSTERESIS_OFF = const(0x00)  # Hysteresis off (default)
HYSTERESIS_1LSB = const(0x01)  # 1 LSB hysteresis
HYSTERESIS_2LSB = const(0x02)  # 2 LSB hysteresis
HYSTERESIS_3LSB = const(0x03)  # 3 LSB hysteresis

# Output stage constants
OUTPUT_STAGE_ANALOG_FULL = const(0x00)  # Analog (0% to 100%)
OUTPUT_STAGE_ANALOG_REDUCED = const(0x01)  # Analog (10% to 90%)
OUTPUT_STAGE_DIGITAL_PWM = const(0x02)  # Digital PWM
OUTPUT_STAGE_RESERVED = const(0x03)  # Reserved

# PWM frequency constants
PWM_FREQ_115HZ = const(0x00)  # 115 Hz (default)
PWM_FREQ_230HZ = const(0x01)  # 230 Hz
PWM_FREQ_460HZ = const(0x02)  # 460 Hz
PWM_FREQ_920HZ = const(0x03)  # 920 Hz

# Slow filter constants
SLOW_FILTER_16X = const(0x00)  # 16x (default)
SLOW_FILTER_8X = const(0x01)  # 8x
SLOW_FILTER_4X = const(0x02)  # 4x
SLOW_FILTER_2X = const(0x03)  # 2x

# Fast filter threshold constants
FAST_FILTER_THRESH_SLOW_ONLY = const(0x00)  # Slow filter only (default)
FAST_FILTER_THRESH_6LSB = const(0x01)  # 6 LSB
FAST_FILTER_THRESH_7LSB = const(0x02)  # 7 LSB
FAST_FILTER_THRESH_9LSB = const(0x03)  # 9 LSB
FAST_FILTER_THRESH_18LSB = const(0x04)  # 18 LSB
FAST_FILTER_THRESH_21LSB = const(0x05)  # 21 LSB
FAST_FILTER_THRESH_24LSB = const(0x06)  # 24 LSB
FAST_FILTER_THRESH_10LSB = const(0x07)  # 10 LSB


class AS5600:
    """Driver for the AS5600 12-bit contactless position sensor.

    :param ~busio.I2C i2c_bus: The I2C bus the AS5600 is connected to.
    :param int address: The I2C device address. Defaults to :const:`AS5600_DEFAULT_ADDRESS`
    """

    # Register definitions using adafruit_register
    _zmco = ROUnaryStruct(_AS5600_REG_ZMCO, "B")  # Read-only burn count

    # 12-bit position registers (stored as 16-bit big-endian)
    _zpos = UnaryStruct(_AS5600_REG_ZPOS_H, ">H")
    _mpos = UnaryStruct(_AS5600_REG_MPOS_H, ">H")
    _mang = UnaryStruct(_AS5600_REG_MANG_H, ">H")
    _rawangle = ROUnaryStruct(_AS5600_REG_RAWANGLE_H, ">H")
    _angle = ROUnaryStruct(_AS5600_REG_ANGLE_H, ">H")

    # 8-bit registers
    _agc = ROUnaryStruct(_AS5600_REG_AGC, "B")
    _magnitude = ROUnaryStruct(_AS5600_REG_MAGNITUDE_H, ">H")

    # Status register bits
    _mh = ROBit(_AS5600_REG_STATUS, 3)  # MH (magnet too strong)
    _ml = ROBit(_AS5600_REG_STATUS, 4)  # ML (magnet too weak)
    _md = ROBit(_AS5600_REG_STATUS, 5)  # MD (magnet detected)

    # Configuration bits
    _power_mode = RWBits(2, _AS5600_REG_CONF_L, 0)
    _hysteresis = RWBits(2, _AS5600_REG_CONF_L, 2)
    _output_stage = RWBits(2, _AS5600_REG_CONF_L, 4)
    _pwm_freq = RWBits(2, _AS5600_REG_CONF_L, 6)
    _slow_filter = RWBits(2, _AS5600_REG_CONF_H, 0)
    _fast_filter_thresh = RWBits(3, _AS5600_REG_CONF_H, 2)
    _watchdog = RWBit(_AS5600_REG_CONF_H, 5)  # Bit 13 of the 16-bit config register

    def __init__(self, i2c: I2C, address: int = AS5600_DEFAULT_ADDRESS) -> None:
        self.i2c_device = I2CDevice(i2c, address)
        # Check if we can communicate with the device
        # The AS5600 doesn't have a WHO_AM_I register, so we'll just try reading status
        _ = self.magnet_detected

    @property
    def zm_count(self) -> int:
        """The number of times ZPOS and MPOS have been permanently burned (0-3).
        This is read-only."""
        return self._zmco & 0x03

    @property
    def z_position(self) -> int:
        """The zero position (start position) as a 12-bit value (0-4095)."""
        return self._zpos & 0x0FFF

    @z_position.setter
    def z_position(self, value: int) -> None:
        """Set the zero position (start position) as a 12-bit value (0-4095)."""
        if not 0 <= value <= 4095:
            raise ValueError("z_position must be between 0 and 4095")
        self._zpos = value & 0x0FFF

    @property
    def m_position(self) -> int:
        """The maximum position (stop position) as a 12-bit value (0-4095)."""
        return self._mpos & 0x0FFF

    @m_position.setter
    def m_position(self, value: int) -> None:
        """Set the maximum position (stop position) as a 12-bit value (0-4095)."""
        if not 0 <= value <= 4095:
            raise ValueError("m_position must be between 0 and 4095")
        self._mpos = value & 0x0FFF

    @property
    def max_angle(self) -> int:
        """The maximum angle range as a 12-bit value (0-4095).
        This represents 0-360 degrees."""
        return self._mang & 0x0FFF

    @max_angle.setter
    def max_angle(self, value: int) -> None:
        """Set the maximum angle range as a 12-bit value (0-4095).
        This represents 0-360 degrees."""
        if not 0 <= value <= 4095:
            raise ValueError("max_angle must be between 0 and 4095")
        self._mang = value & 0x0FFF

    @property
    def raw_angle(self) -> int:
        """The raw angle reading as a 12-bit value (0-4095).
        This is unscaled and unmodified by ZPOS/MPOS/MANG settings."""
        return self._rawangle & 0x0FFF

    @property
    def angle(self) -> int:
        """The scaled angle reading as a 12-bit value (0-4095).
        This is scaled according to ZPOS/MPOS/MANG settings."""
        return self._angle & 0x0FFF

    @property
    def agc_min_gain_overflow(self) -> bool:
        """True if AGC minimum gain overflow occurred (magnet too strong)."""
        return self._mh

    @property
    def agc_max_gain_overflow(self) -> bool:
        """True if AGC maximum gain overflow occurred (magnet too weak)."""
        return self._ml

    @property
    def magnet_detected(self) -> bool:
        """True if a magnet is detected."""
        return self._md

    @property
    def agc(self) -> int:
        """The current AGC (Automatic Gain Control) value.
        Range is 0-255 in 5V mode, 0-128 in 3.3V mode."""
        return self._agc

    @property
    def magnitude(self) -> int:
        """The magnitude value from the CORDIC processor (0-4095)."""
        return self._magnitude & 0x0FFF

    @property
    def watchdog(self) -> bool:
        """Enable or disable the watchdog timer."""
        return self._watchdog

    @watchdog.setter
    def watchdog(self, value: bool) -> None:
        """Enable or disable the watchdog timer."""
        self._watchdog = value

    @property
    def power_mode(self) -> int:
        """The power mode setting. Use POWER_MODE_* constants."""
        return self._power_mode

    @power_mode.setter
    def power_mode(self, value: int) -> None:
        """Set the power mode. Use POWER_MODE_* constants."""
        if not 0 <= value <= 3:
            raise ValueError("Invalid power mode")
        self._power_mode = value

    @property
    def hysteresis(self) -> int:
        """The hysteresis setting. Use HYSTERESIS_* constants."""
        return self._hysteresis

    @hysteresis.setter
    def hysteresis(self, value: int) -> None:
        """Set the hysteresis. Use HYSTERESIS_* constants."""
        if not 0 <= value <= 3:
            raise ValueError("Invalid hysteresis setting")
        self._hysteresis = value

    @property
    def output_stage(self) -> int:
        """The output stage configuration. Use OUTPUT_STAGE_* constants."""
        return self._output_stage

    @output_stage.setter
    def output_stage(self, value: int) -> None:
        """Set the output stage configuration. Use OUTPUT_STAGE_* constants."""
        if not 0 <= value <= 3:
            raise ValueError("Invalid output stage setting")
        self._output_stage = value

    @property
    def pwm_frequency(self) -> int:
        """The PWM frequency setting. Use PWM_FREQ_* constants."""
        return self._pwm_freq

    @pwm_frequency.setter
    def pwm_frequency(self, value: int) -> None:
        """Set the PWM frequency. Use PWM_FREQ_* constants."""
        if not 0 <= value <= 3:
            raise ValueError("Invalid PWM frequency setting")
        self._pwm_freq = value

    @property
    def slow_filter(self) -> int:
        """The slow filter setting. Use SLOW_FILTER_* constants."""
        return self._slow_filter

    @slow_filter.setter
    def slow_filter(self, value: int) -> None:
        """Set the slow filter. Use SLOW_FILTER_* constants."""
        if not 0 <= value <= 3:
            raise ValueError("Invalid slow filter setting")
        self._slow_filter = value

    @property
    def fast_filter_threshold(self) -> int:
        """The fast filter threshold setting. Use FAST_FILTER_THRESH_* constants."""
        return self._fast_filter_thresh

    @fast_filter_threshold.setter
    def fast_filter_threshold(self, value: int) -> None:
        """Set the fast filter threshold. Use FAST_FILTER_THRESH_* constants."""
        if not 0 <= value <= 7:
            raise ValueError("Invalid fast filter threshold setting")
        self._fast_filter_thresh = value
