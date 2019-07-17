# SerialTracker
Alternative firmware code for the [Relativty](https://www.relativty.net/) head tracker that uses the MPU6050 (or MPU9150) sensor.

## Getting started
Arduino Due (and Uno) boards haven't been tested yet, I don't own a Due so it's up to you.  

### Hardware
Connect the sensor corresponding to the defines in the SerialTracker.ino sketch, here are the defaults:

Sensor | INT | SDA | SCL | VCC | GND 
-------|-----|-----|-----|-----|-----
Board  | INT_PIN | SDA_PIN | SCL_PIN | VCC_PIN | 
[Due](http://www.robgray.com/temp/Due-pinout-WEB.png) | _2_ | 20 | 21 | _5_ | GND
[STM32](https://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif) | PA1 | PB6 | PB7 | _PB5_ | G
[ESP32](https://components101.com/sites/default/files/component_pin/ESP32-Pinout.png) | _IO17_ | _IO18_ | _IO19_ | _IO5_ | GND
[Uno](https://upload.wikimedia.org/wikipedia/commons/c/c9/Pinout_of_ARDUINO_Board_and_ATMega328PU.svg) | 2 | A4 | A5 | _5_ | GND

### Software
1. Install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
1. Install Arduino IDE support for your board:
   * Arduino Due: [Getting started with the Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue#toc2)
   * STM32 Pills: [Arduino for STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki/Installation)
   * ESP32: [Arduino core for ESP32 WiFi chip](https://github.com/espressif/arduino-esp32#arduino-core-for-esp32-wifi-chip----) 
1. Install additional libraries for Arduino IDE
   * [I2CDevLib](https://github.com/jrowberg/i2cdevlib), for installation simply copy the `I2Cdev`, `MPU6050` and `MPU9150` folders (below the Arduino folder in the archive) to your `Arduino/libraries` directory.
1. Put the folder containing this README.md (and SerialTracker.ino) in your `Arduino` directory.
1. Start Arduino IDE, open sketch and upload.

If the I2CDevLib is broken (as currently seems the case) there will be strange compilation errors.  Try the [development branch](https://github.com/jrowberg/i2cdevlib/tree/develop) instead. 

