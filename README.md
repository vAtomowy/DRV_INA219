# INA219 Driver

This is a lightweight driver for the INA219 high-side current, voltage, and power monitor, designed for use with the ESP32-C3 and ESP-IDF.

## Description

The INA219 is a precision digital current and power monitor that communicates via the I²C interface. It allows measurement of voltage, current, and power by sensing the voltage drop across a shunt resistor placed in the power path.

The device integrates a 12-bit delta-sigma ADC and provides direct register access to raw measurements as well as calculated power data. Its compact size and high accuracy make it suitable for power-sensitive embedded applications.

## Features

- Bus voltage measurement up to 26 V
- Current measurement up to ±3.2 A (depends on shunt value)
- 12-bit resolution ADC
- Configurable conversion times and averaging
- High-side or low-side current sensing
- I²C communication (3.3 V and 5 V compatible)
- Power consumption monitoring through internal calculations

## Applications

- Battery-powered systems
- Energy monitoring for IoT devices
- Solar and renewable energy systems
- Robotics and automation
- Industrial equipment diagnostics

## Typical Module

Common INA219 breakout boards include:
- On-board 0.1 ohm shunt resistor
- 4-pin I²C interface: VCC, GND, SDA, SCL
- Operates directly with 3.3 V logic (ESP32-compatible)

## Usage

This driver allows reading of:
- Shunt voltage (voltage drop across the shunt resistor)
- Bus voltage (system power line)
- Current (calculated using calibration parameters)
- Power (calculated by the chip)

## License

MIT License

