# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_as5600

# Create I2C bus and sensor instance
i2c = board.I2C()
sensor = adafruit_as5600.AS5600(i2c)

# Configure sensor
sensor.power_mode = adafruit_as5600.POWER_MODE_NOM
sensor.hysteresis = adafruit_as5600.HYSTERESIS_OFF
sensor.slow_filter = adafruit_as5600.SLOW_FILTER_16X

while True:
    # Read angle values
    print(f"Raw angle: {sensor.raw_angle}")
    print(f"Scaled angle: {sensor.angle}")
    print(f"Magnitude: {sensor.magnitude}")
    print(f"Magnet detected: {sensor.magnet_detected}")
    time.sleep(2)