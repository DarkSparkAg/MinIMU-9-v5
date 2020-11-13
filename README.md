# MinIMU-9-v5 python library for Raspberry Pi

Version: 1.0.0<br>
Release date: 2019 May 21<br>
[https://github.com/DarkSparkAg/MinIMU-9-v5](https://github.com/DarkSparkAg/MinIMU-9-v5)

## Summary

This is a library for the Raspberry Pi that helps interface with Pololu's [MinIMU-9 v5 Gyro, Accelerometer, and Compass (LSM6DS33 and LIS3MDL Carrier)](https://www.pololu.com/product/2738/resources). The library configures the LSM6DS33 and LIS3MDL chips and makes it simple to read the raw accelerometer, gyro, and magnetometer data through I&sup2;C.  The library is also capable of tracking yaw and angle in the background with multi-threading.

## Supported platforms

This library is designed to work with the Raspberry Pi 3 B+; we have not tested it with other versions.

## Getting started

### Hardware

An [MinIMU-9 v5](https://www.pololu.com/product/2738/resources) can be purchased from Pololu's website.  Before continuing, careful reading of the [product page](https://www.pololu.com/product/2738/resources), datasheets and application notes are recommended.

Make the following connections between the Raspberry Pi 3 B+ and the MinIMU-9 v5 board:

      Arduino   LSM6 board
    ---------   ----------
    5V or 3V3 - VIN
          GND - GND
          SDA - SDA
          SCL - SCL

### Software

1. Download the lastest version of this software from github.
2. Copy the MinIMU_v5_pi.py file into your project.

## Examples

An example program using the MinIMU_v5_pi.py file is available that shows how to use the library. You can access it in the 'examples' program.

## Library reference

Todo

## Other

This library is a work in progress and was based on the [LSM6](https://github.com/pololu/lsm6-arduino) and [LIS3MDL](https://github.com/pololu/lis3mdl-arduino) libraries created by [pololu](https://github.com/pololu).
