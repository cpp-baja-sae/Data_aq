[![BajaSaeLogo](https://images.squarespace-cdn.com/content/v1/62f077e4acc8f152cbea493f/a5773387-fd2a-41fb-9677-72fdadb26182/logo1_blacknwhite.jpg?h=150)](https://www.cppbajasae.com/)
![language](https://img.shields.io/badge/Language-C%2B%2B-red)
[![mcu](https://img.shields.io/badge/MCU-Teensy_4.1-orange)](https://www.pjrc.com/store/teensy41.html)
[![board](https://img.shields.io/badge/PCB_Version-2-yellow)](https://www.altium.com/)
[![college](https://img.shields.io/badge/CAL_POLY-POMONA-green)](https://www.cpp.edu/)
[![ide](https://img.shields.io/badge/IDE-Arduino-blue)](https://www.arduino.cc/en/software/)
[![year](https://img.shields.io/badge/BAJA-2026-purple)](https://www.cppbajasae.com/)

# 📊 Data Aquisition
This repository contains the code that will be flashed onto the MCU's on the 2025-2026 Baja Car.

There are 2 individual ones, communicationg to each other via i2c. They are named "Teensy 2" and "Teensy 3" with their code named the same.


# ☀️ Teensy 2 Sensors: 




|  Pin #   |           Function            |
|----------|-------------------------------|
|        1 | On-board LED                  |
| 10,12,13 | Encoder : Sel Data Clock      |
|    16,17 | Teensy2Teensy I2C : SCL SDA   |
|    18,19 | I2C : SCL SDA                 |
|       21 | Engine RPM                    |
|       23 | Rear Wheel RPM                |
|       26 | Front Brake Pressure          |
|       27 | Rear Break Pressure           |
|       32 | On-Board Button               |
|       38 | Front Left Suspension Travel  |
|       39 | Rear Left Suspension Travel   |
|       40 | Rear Right Suspension Travel  |
|       41 | Front Right Suspension Travel |


# 🌙 Teensy 3 Sensors: 


| Pin # |          Function           |
|-------|-----------------------------|
| 1     | On-Board LED                |
| 16,17 | Teensy2Teensy I2C : SCL SDA |
| 18,19 | I2C : SCL SDA               |
| 20    | Left Wheel RPM              |
| 21    | Right Wheel RPM             |
| 22,23 | Voltage Monitor : 2 1       |
| 27    | Steering Angle              |
| 35    | Screen Reset                |
| 37    | On-Board Button             |


