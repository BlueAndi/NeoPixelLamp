# NeoPixelLamp

A lamp with some NeoPixels and a 3D accelerometer, 3D gyroscope and 3D magnetometer.

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](http://choosealicense.com/licenses/mit/)
[![Release](https://img.shields.io/github/release/BlueAndi/NeoPixelLamp.svg)](https://github.com/BlueAndi/NeoPixelLamp/releases)

1. [Description](https://github.com/BlueAndi/NeoPixelLamp#description)
2. [Issues, Ideas and bugs](https://github.com/BlueAndi/NeoPixelLamp#issues-ideas-and-bugs)
3. [License](https://github.com/BlueAndi/NeoPixelLamp#license)
4. [Contribution](https://github.com/BlueAndi/NeoPixelLamp#contribution)

## Description

This arduino sketch controls a lamp with
* an [Adafruit FLORA V2](https://www.adafruit.com/products/659),
* an [Adafruit NeoPixel Ring - 16](https://www.adafruit.com/product/1463),
* an [Adafruit FLORA 9-DOF Accelerometer/Gyroscope/Magnetometer - LSM9DS0 - v1.0](https://www.adafruit.com/product/2020),
* an [Sparkfun USB LiPoly Charger - Single Cell](https://www.sparkfun.com/products/12711) and
* a LiPo Akku with 400mAh.

Shake the lamp according to the z-axis to switch between different programs.
Turn the lamp about 90 degree and rotate it to increase or decrease the LED brightness.
Turn the lamp about 180 degree and it will jump into deep standby.

Implemented programs:

1. Rainbow colors
2. Theatre-style crawling lights with rainbow effect
3. Color changes dependen on the temperature
4. Color can be changed by rotating around the z-axis
5. Color moves like knight rider

Used 3rd party libraries:

* [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) v1.0.6
* [Adafruit_LSM9DS0_Library](https://github.com/adafruit/Adafruit_LSM9DS0_Library) V1.0.2
* [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor) V1.0.2

Have fun!

## Issues, Ideas and bugs

If you have further ideas or you found some bugs, great! Create a [issue](https://github.com/BlueAndi/NeoPixelLamp/issues) or if
you are able and willing to fix it by yourself, clone the repository and create a pull request.

## License
The whole source code is published under the [MIT license](http://choosealicense.com/licenses/mit/).

## Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, shall be licensed as above, without any
additional terms or conditions.
